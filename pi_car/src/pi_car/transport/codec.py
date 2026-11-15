import json
from pi_car.common.messages import Pose


def encode_pose(pose):
    return (json.dumps({"type": "pose", "schema_version": 1, **pose.to_dict()},
                       allow_nan=False) + "\n").encode("utf-8")


def decode_pose(line):
    if len(line) > 4096:
        raise ValueError("Message exceeds 4096 bytes")
    message = json.loads(line)
    if message.pop("type") != "pose" or message.pop("schema_version") != 1:
        raise ValueError("Unsupported message schema")
    return Pose(**message)
