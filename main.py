# %%
import socket
from enum import Enum

import numpy as np
import signal
import sys
from message_format import MessageType, DecodedPacket, parameter_message, stop_message

TERMINATOR = b'}"}'
audio_data = []
tracker_data = [[], [], [], [], [], [], [], []]
listening: bool = True


# noinspection PyUnusedLocal
def signal_handler(signal_num: int, frame):
    if signal_num == signal.SIGINT:
        print("You pressed Ctrl+C!")
        # TODO: change state


signal.signal(signal.SIGINT, signal_handler)

buffer_packet_bytes = b''


def find_next_packet():
    global buffer_packet_bytes
    packet_end = -1  # terminator not found
    while packet_end < 0:
        buffer_packet_bytes += s.recvfrom(65565)[0]
        packet_end = buffer_packet_bytes.find(TERMINATOR)

    # noinspection PyShadowingNames
    current_packet = buffer_packet_bytes[0:packet_end + len(TERMINATOR)]
    buffer_packet_bytes = buffer_packet_bytes[packet_end + len(TERMINATOR):]
    return DecodedPacket.from_str(current_packet)


if len(sys.argv) != 3:
    print("Usage: [data_collector.py] user_id trial_id")

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

while True:
    if current_state == CollectorState.START:
        s.send(parameter_message(user_id, trial_id))
        current_state = CollectorState.START_ACK
    elif current_state == CollectorState.STOP:
        s.send(stop_message())
        current_state = CollectorState.STOP_ACK
    else:
        current_packet = find_next_packet()
        if current_state == CollectorState.START_ACK:
            if current_packet.message_type != MessageType.ACKNOWLEDGE:
                print("Error: Cannot start recording!")
                print(current_packet)
                break
        else:
            if current_packet.message_type == MessageType.ERROR:
                print(current_packet)
            elif current_packet.message_type == MessageType.ACKNOWLEDGE:
                if current_state == MessageType.STOP_ACK:
                    s.close()
                    # TODO: close audio stream
                    # TODO: save files
                    print("Data collection ended!")
                    sys.exit(0)
                else:
                    print(f"Warning: Unexpected ACK received! ({current_state})")
            elif current_packet.message_type == MessageType.AUDIO_DATA:
                audio_data = np.concatenate([audio_data, current_packet.content["DataAudio"]], axis=0)
            elif current_packet.message_type == MessageType.TRACKING_DATA:
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
