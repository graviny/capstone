"""Versioned JSON Lines protocol; no network or robot scheduling logic here."""
import json
import math
import re

MAX_MESSAGE_BYTES = 16384
TERMINAL = frozenset({"ARRIVED", "BLOCKED", "ERROR", "CANCELLED", "STOPPED"})
STATUS = TERMINAL | {"READY", "GOAL_ACCEPTED", "MOVING"}
TYPES = STATUS | {"HELLO", "POSE", "GOAL", "STOP", "PING", "PONG"}


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,128}", value):
        raise ValueError("ID must be 1..128 ASCII letters, digits or _.:-")
    return value


def number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("Expected a finite number (not boolean)")
    return value


def validate(message):
    if (not isinstance(message, dict) or type(message.get("schema_version")) is not int
            or message.get("schema_version") != 1):
        raise ValueError("Expected schema_version=1 JSON object")
    kind = message.get("type")
    if not isinstance(kind, str) or kind not in TYPES:
        raise ValueError("Unsupported message type")
    for name in ("robot_id", "frame_id", "map_version"):
        identifier(message.get(name))
    if kind in {"GOAL", "STOP"} or kind in STATUS - {"READY"}:
        identifier(message.get("command_id"))
    if kind == "GOAL":
        goal = message.get("goal")
        if not isinstance(goal, dict) or set(goal) != {"x", "y"}:
            raise ValueError("goal requires x and y")
        number(goal["x"]); number(goal["y"])
    if kind == "POSE":
        if type(message.get("valid")) is not bool:
            raise ValueError("POSE requires boolean valid")
        for name in ("x", "y", "theta", "timestamp"):
            number(message.get(name))
    return message


def make_message(kind, robot_id, frame_id, map_version, **fields):
    return validate(dict(type=kind, schema_version=1, robot_id=robot_id,
                         frame_id=frame_id, map_version=map_version, **fields))


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate JSON key")
        result[key] = value
    return result


def encode_message(message):
    raw = (json.dumps(validate(message), allow_nan=False, separators=(",", ":"))+"\n").encode()
    if len(raw) > MAX_MESSAGE_BYTES:
        raise ValueError("Message too large")
    return raw


def decode_message(raw):
    if len(raw) > MAX_MESSAGE_BYTES or not raw.endswith(b"\n"):
        raise ValueError("Oversized or unterminated message")
    try:
        message = json.loads(raw, object_pairs_hook=_unique_object,
                             parse_constant=lambda _: (_ for _ in ()).throw(ValueError("Nonfinite JSON")))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ValueError("Invalid UTF-8 JSON") from error
    return validate(message)
