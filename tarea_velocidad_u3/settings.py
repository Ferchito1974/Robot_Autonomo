"""settings.py - Values editable from the interface, saved in settings.json (defaults in config.EDITABLE)."""
import copy
import json
import os

import config

FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "settings.json")


def load():
    s = copy.deepcopy(config.EDITABLE)
    if os.path.exists(FILE):
        with open(FILE) as f:
            s.update({k: v for k, v in json.load(f).items() if k in s})
    return s


def save(new):
    s = load()
    for k, v in new.items():
        if k not in s:
            continue
        if k == "heading_source":
            if v not in ("gyro", "encoders"):
                raise ValueError(k)
        elif isinstance(s[k], list):
            v = [float(x) for x in v]
            if len(v) != 3:
                raise ValueError(k)
        else:
            v = float(v)
        s[k] = v
    with open(FILE, "w") as f:
        json.dump(s, f, indent=2)
    return s
