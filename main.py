# %%
import os
import socket
from enum import Enum

import numpy as np
import signal
import sounddevice as sd
import sys
import datetime
from message_format import MessageType, DecodedPacket, parameter_message, stop_message

TERMINATOR = b'}"}'
audio_data = []
tracker_data = [[], [], [], [], [], [], [], []]
mic_data = None
listening: bool = True


# noinspection PyUnusedLocal
def signal_handler(signal_num: int, frame):
    global current_state
    if signal_num == signal.SIGINT:
        print("You pressed Ctrl+C!")
        current_state = CollectorState.STOP


signal.signal(signal.SIGINT, signal_handler)

buffer_packet_bytes = b''


def find_next_packet():
    global buffer_packet_bytes
    packet_end = buffer_packet_bytes.find(TERMINATOR)  # terminator not found
    while packet_end < 0:
        buffer_packet_bytes += s.recvfrom(65565)[0]
        packet_end = buffer_packet_bytes.find(TERMINATOR)

    # noinspection PyShadowingNames
    current_packet = buffer_packet_bytes[0:packet_end + len(TERMINATOR)]
    buffer_packet_bytes = buffer_packet_bytes[packet_end + len(TERMINATOR):]
    return DecodedPacket.from_str(current_packet)


def mic_handler(in_data, *_):
    global mic_data
    new_timestamp = datetime.datetime.utcnow() - datetime.datetime(1970, 1, 1, 0, 0, 0, 0)
    if prev_timestamp[1] is not None:
        samples = (new_timestamp - prev_timestamp[1]) * AUDIO_RATE
        total_expected_samples[1] += samples
        total_samples[1] += len(in_data)
    prev_timestamp[1] = new_timestamp
    # in_data *= (10 ** 1.45) * (2 ** 31)
    if mic_data is None:
        mic_data = in_data
    else:
        mic_data = np.concatenate((mic_data, in_data), axis=0)


def check_audio_latency():
    global total_expected_samples, total_samples
    if prev_timestamp[0] is not None:
        samples = (current_packet.content['Timestamp'] - prev_timestamp[0]) * TRACKING_RATE
        total_expected_samples[0] += samples
        total_samples[0] += len(current_packet.content['DataAudio'])
        print(samples - len(current_packet.content['DataAudio']))


def check_tracking_latency():
    global total_expected_samples, total_samples
    if prev_timestamp[2] is not None:
        samples = (current_packet.content['Timestamp'] - prev_timestamp[2]) * AUDIO_RATE
        total_expected_samples[2] += samples
        total_samples[2] += len(current_packet.content['DataPositions'])
        print(samples - len(current_packet.content['DataPositions']))


if len(sys.argv) != 3:
    print("Usage: [data_collector.py] user_id trial_id")
    sys.exit(0)

user_id = int(sys.argv[1])
trial_id = int(sys.argv[2])

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("192.168.144.107", 38823))


class CollectorState(Enum):
    START = 0
    START_ACK = 1
    COLLECT = 2
    STOP = 3
    STOP_ACK = 4


current_state = CollectorState.START
mic_stream = None

devices = sd.query_devices()
mic_id = 0
for device in range(len(devices)):
    if devices[device]['name'].startswith('miniDSP VocalFusion Spk (UAC2.0'):
        mic_id = device

data_directory = './data'
start = datetime.datetime.now()
experiment_subdirectory = start.strftime("%Y%m%dT%H%M%S")

experiment_directory = os.path.join(data_directory, experiment_subdirectory)

if not os.path.exists(experiment_directory):
    os.makedirs(experiment_directory)

AUDIO_RATE = 48000
TRACKING_RATE = 72
VR_AUDIO_FILE_NAME = "vr_audio_data.npy"
MICROPHONE_FILE_NAME = "mic_data.npy"
VR_TRACKING_FILE_NAME = "vr_tracking_data.npy"

mic_stream = sd.InputStream(samplerate=48000, channels=devices[mic_id]['max_input_channels'], device=mic_id,
                            callback=mic_handler)

prev_timestamp = [None, None, None]
total_samples = [0, 0, 0]
total_expected_samples = [0, 0, 0]

while True:
    print(current_state)
    if current_state == CollectorState.START:
        s.send(parameter_message(user_id, trial_id).message_str())
        current_state = CollectorState.START_ACK
        mic_stream.start()
    elif current_state == CollectorState.STOP:
        s.send(stop_message().message_str())
        current_state = CollectorState.STOP_ACK
    else:
        current_packet = find_next_packet()
        print(current_packet.message_type)
        if current_state == CollectorState.START_ACK:
            if current_packet.message_type != MessageType.ACKNOWLEDGE:
                print("Error: Cannot start recording!")
                print(current_packet)
                break
            else:
                current_state = CollectorState.COLLECT
        else:
            if current_packet.message_type == MessageType.ERROR:
                print(current_packet)
            elif current_packet.message_type == MessageType.ACKNOWLEDGE:
                if current_state == CollectorState.STOP_ACK:
                    s.close()
                    mic_stream.close()
                    np.save(os.path.join(experiment_directory, VR_AUDIO_FILE_NAME), np.array(audio_data))
                    np.save(os.path.join(experiment_directory, MICROPHONE_FILE_NAME), np.array(mic_data))
                    np.save(os.path.join(experiment_directory, VR_TRACKING_FILE_NAME), np.array(tracker_data).T)
                    print("Data collection ended!")
                    sys.exit(0)
                else:
                    print(f"Warning: Unexpected ACK received! ({current_state})")
            elif current_packet.message_type == MessageType.AUDIO_DATA:
                check_audio_latency()
                prev_timestamp[0] = current_packet.content['Timestamp']
                audio_data = np.concatenate([audio_data, current_packet.content["DataAudio"]], axis=0)
            elif current_packet.message_type == MessageType.TRACKING_DATA:
                check_tracking_latency()
                prev_timestamp[2] = current_packet.content['Timestamp']
                tracker_data[0] = np.concatenate([tracker_data[0], current_packet.content["DataTimeStamps"]], axis=0)
                for sample in current_packet.content['DataPositions']:
                    tracker_data[1].append(sample['x'])
                    tracker_data[2].append(sample['y'])
                    tracker_data[3].append(sample['z'])
                for sample in current_packet.content['DataQuaternions']:
                    tracker_data[4].append(sample['x'])
                    tracker_data[5].append(sample['y'])
                    tracker_data[6].append(sample['z'])
                    tracker_data[7].append(sample['w'])
