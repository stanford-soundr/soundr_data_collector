# %%
import os
import socket
from enum import Enum
from typing import Optional

import numpy as np
import signal
import sounddevice as sd
import sys
import datetime
from message_format import MessageType, DecodedPacket, parameter_message, stop_message, timestamp_now

TERMINATOR = b'}"}'
audio_data: list = []
tracker_data = [[], [], [], [], [], [], [], []]
mic_data: list = []
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


first_microphone_frame = True
microphone_start_timestamp: Optional[float] = None
headset_start_timestamp: Optional[float] = None
microphone_start_ack_received_time_diff: Optional[float] = None
start_ack_received_timestamp: Optional[float] = None
start_ack_diff: Optional[float] = None
microphone_stop_timestamp: Optional[float] = None
headset_audio_start_timestamp: Optional[float] = 0
headset_audio_stop_timestamp: Optional[float] = 0
tracking_start_timestamp: Optional[float] = None
tracking_stop_timestamp: Optional[float] = None


def mic_handler(in_data, *_):
    global microphone_start_timestamp
    global microphone_stop_timestamp
    global mic_data
    global first_microphone_frame
    in_data = in_data.copy()  # in_data will be reused by sounddevice
    if first_microphone_frame:
        microphone_start_timestamp = timestamp_now() - in_data.shape[0] / AUDIO_RATE
    microphone_stop_timestamp = timestamp_now() - in_data.shape[0] / AUDIO_RATE
    first_microphone_frame = False
    new_timestamp = timestamp_now()
    if prev_timestamp[1] is not None:
        samples = (new_timestamp - prev_timestamp[1]) * AUDIO_RATE
        total_expected_samples[1] += samples
        total_samples[1] += len(in_data)
        total_delay[1] = (total_expected_samples[1] - total_samples[1]) / AUDIO_RATE
    prev_timestamp[1] = new_timestamp
    # in_data *= (10 ** 1.45) * (2 ** 31)
    mic_data.append(in_data)


def check_audio_latency():
    global total_expected_samples, total_samples
    if prev_timestamp[0] is not None:
        samples = (current_packet.content['Timestamp'] - prev_timestamp[0]) * AUDIO_RATE
        total_expected_samples[0] += samples
        total_samples[0] += len(current_packet.content['DataAudio'])
        total_delay[0] = (total_expected_samples[0] - total_samples[0]) / AUDIO_RATE


def check_tracking_latency():
    global total_expected_samples, total_samples
    if prev_timestamp[2] is not None:
        samples = (current_packet.content['Timestamp'] - prev_timestamp[2]) * TRACKING_RATE
        total_expected_samples[2] += samples
        total_samples[2] += len(current_packet.content['DataPositions'])
        total_delay[2] = (total_expected_samples[2] - total_samples[2]) / TRACKING_RATE


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
mic_id = -1
for device in range(len(devices)):
    if devices[device]['name'].startswith('miniDSP VocalFusion Spk (UAC2.0'):
        mic_id = device

if mic_id is -1:
    print("Could not find correct mic")
    sys.exit(-1)

data_directory = './data'
start = datetime.datetime.now()
experiment_subdirectory = start.strftime("%Y%m%dT%H%M%S")

experiment_directory = os.path.join(data_directory, experiment_subdirectory)

if not os.path.exists(experiment_directory):
    os.makedirs(experiment_directory)

AUDIO_RATE = 48000.0
TRACKING_RATE = 72.0
VR_AUDIO_FILE_NAME = "vr_audio_data.npy"
MICROPHONE_FILE_NAME = "mic_data.npy"
VR_TRACKING_FILE_NAME = "vr_tracking_data.npy"
OFFSET_FILE_NAME = "offset.npy"

mic_stream = sd.InputStream(samplerate=48000, channels=devices[mic_id]['max_input_channels'], device=mic_id,
                            callback=mic_handler)
mic_stream.start()

prev_timestamp = [None, None, None]
total_samples = [0, 0, 0]
total_expected_samples = [0, 0, 0]
total_delay = [0.0, 0.0, 0.0]

while first_microphone_frame:
    pass

while True:
    print(current_state)
    print(total_delay)
    if current_state == CollectorState.START:
        s.send(parameter_message(user_id, trial_id).message_str())
        current_state = CollectorState.START_ACK
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
                headset_start_timestamp = current_packet.content['Timestamp']
                start_ack_received_timestamp = timestamp_now()
                microphone_start_ack_received_time_diff = start_ack_received_timestamp - microphone_start_timestamp
                start_ack_diff = start_ack_received_timestamp - headset_start_timestamp
                print(f"Start time diff: {start_ack_diff}")
                current_state = CollectorState.COLLECT
        else:
            if current_packet.message_type == MessageType.ERROR:
                print(current_packet)
            elif current_packet.message_type == MessageType.ACKNOWLEDGE:
                if current_state == CollectorState.STOP_ACK:
                    headset_stop_timestamp = current_packet.content['Timestamp']
                    stop_ack_received_timestamp = timestamp_now()
                    stop_ack_diff = stop_ack_received_timestamp - headset_stop_timestamp
                    expected_audio_samples = (headset_audio_stop_timestamp - headset_audio_start_timestamp) * AUDIO_RATE
                    expected_mic_samples = (microphone_stop_timestamp - microphone_start_timestamp) * AUDIO_RATE
                    expected_tracking_samples = (tracking_stop_timestamp - tracking_start_timestamp) * TRACKING_RATE
                    tracker_data = np.array(tracker_data).T
                    offset = [expected_audio_samples - len(audio_data), expected_mic_samples - len(mic_data),
                              expected_tracking_samples - len(tracker_data)]
                    print(f"Stop time diff: {stop_ack_diff}")
                    s.close()
                    mic_stream.close()
                    mic_data_np = np.concatenate(mic_data, axis=0)
                    mic_data_np = mic_data_np[int(microphone_start_ack_received_time_diff * AUDIO_RATE):]
                    # audio_data = np.concatenate(audio_data, axis=0)
                    np.save(os.path.join(experiment_directory, OFFSET_FILE_NAME), offset)
                    np.save(os.path.join(experiment_directory, VR_AUDIO_FILE_NAME), audio_data)
                    np.save(os.path.join(experiment_directory, MICROPHONE_FILE_NAME), mic_data_np)
                    np.save(os.path.join(experiment_directory, VR_TRACKING_FILE_NAME), tracker_data)
                    print("Data collection ended!")
                    assert(abs(stop_ack_diff - start_ack_diff) < 0.5)
                    sys.exit(0)
                else:
                    print(f"Warning: Unexpected ACK received! ({current_state})")
            elif current_packet.message_type == MessageType.AUDIO_DATA:
                if headset_audio_start_timestamp == 0:
                    headset_audio_start_timestamp = current_packet.content['Timestamp'] - \
                                                    len(current_packet.content['DataAudio']) / AUDIO_RATE
                headset_audio_stop_timestamp = current_packet.content['Timestamp']
                check_audio_latency()
                prev_timestamp[0] = current_packet.content['Timestamp']
                audio_data.append(current_packet.content["DataAudio"])
            elif current_packet.message_type == MessageType.TRACKING_DATA:
                if tracking_start_timestamp is None:
                    tracking_start_timestamp = current_packet.content['Timestamp'] - \
                                                    len(current_packet.content['DataTimeStamps']) / TRACKING_RATE
                tracking_stop_timestamp = current_packet.content['Timestamp']
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
