#%%
import socket
import json
import numpy as np
import signal
import datetime
import sys

TERMINATOR = b'}"}'
audio_data = []
tracker_data = [[], [], [], [], [], [], [], []]
listening = 1

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("192.168.144.107", 38823))

parameter_message = b'{"MessageType": "PARAMETER", "Content": "{\\\"UserId\\\":0, \\\"TrialId\\\": 0}"}'
#%%
s.send(parameter_message)

#%%

def signal_handler(signal : int, frame):
    print("You pressed Ctrl+C!")
    timestamp = datetime.datetime.utcnow() - datetime.datetime(1970, 1, 1, 0, 0, 0, 0)
    extract_packet_data({'MessageType': 'STOP', 'Content': str(timestamp.total_seconds())})
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def find_next_packet(current_packet_json, packet_num):
    packet_end = -1
    while packet_end < 0:
        packet_num += 1
        current_packet_json += s.recvfrom(65565)[0]
        packet_end = current_packet_json.find(TERMINATOR)

    return current_packet_json, packet_end, packet_num

def get_packet(current_packet_json, packet_num):
    current_packet_json, packet_end, packet_num = find_next_packet(current_packet_json, packet_num)
    current_packet = json.loads(current_packet_json[0:packet_end + len(TERMINATOR)])
    current_packet_json = current_packet_json[packet_end + len(TERMINATOR):]
    extract_packet_data(current_packet)
    return current_packet_json, packet_num

def extract_packet_data(current_packet):
    global audio_data, listening
    print(current_packet)
    content = json.loads(current_packet["Content"])
    if (packet_num == 0) and (current_packet["MessageType"] != "ACKNOWLEDGE"):
        listening = 0
    elif current_packet["MessageType"] == "ERROR":
        print(current_packet)
    elif current_packet["MessageType"] == "STOP":
        print("Sending Final Packets")
        packet_json, num = get_packet(current_packet_json, packet_num)
        packet_json, num = get_packet(packet_json, num)
        sys.exit(0)

    elif current_packet["MessageType"] == "AUDIO_DATA":
        audio_data = np.concatenate([audio_data, content["DataAudio"]], axis=0)
    elif current_packet["MessageType"] == "TRACKING_DATA":
        tracker_data[0] = np.concatenate([tracker_data[0], content["DataTimeStamps"]], axis=0)
        for sample in content['DataPositions']:
            tracker_data[1].append(sample['x'])
            tracker_data[2].append(sample['y'])
            tracker_data[3].append(sample['z'])
        for sample in content['DataQuaternions']:
            tracker_data[4].append(sample['x'])
            tracker_data[5].append(sample['y'])
            tracker_data[6].append(sample['z'])
            tracker_data[7].append(sample['w'])

current_packet_json = b''
packet_num = 0
while listening > 0:
    current_packet_json, packet_num = get_packet(current_packet_json, packet_num)


