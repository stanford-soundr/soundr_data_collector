import datetime
import json
from enum import Enum
from typing import Dict, Union


class MessageType(Enum):
    PARAMETER = 0
    ACKNOWLEDGE = 1
    ERROR = 2
    STOP = 3
    AUDIO_DATA = 4
    TRACKING_DATA = 5


class DecodedPacket:
    message_type: MessageType
    content: Dict

    def __init__(self, message_type, content):
        self.message_type = message_type
        self.content = content

    @staticmethod
    def from_str(message_str: str):
        json_message = json.loads(message_str)
        message_type = MessageType(json_message["MessageType"])
        content = json.loads(json_message["Content"])
        return DecodedPacket(message_type, content)

    def message_str(self):
        content_str = json.dumps(self.content)
        dict_message = {"MessageType": self.message_type.name, "Content": content_str}
        return json.dumps(dict_message)


def parameter_message(user_id: int, trial_id: int):
    return DecodedPacket(MessageType.PARAMETER, {"UserId": user_id, "TrialId": trial_id})


def stop_message(timestamp: Union[None, float] = None):
    if timestamp is None:
        timestamp = datetime.datetime.utcnow() - datetime.datetime(1970, 1, 1, 0, 0, 0, 0)
    return DecodedPacket(MessageType.STOP, {"Timestamp": timestamp.total_seconds()})
