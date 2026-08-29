#!/usr/bin/env python3
from pathlib import Path

import yaml


class BlueprintLoader(yaml.SafeLoader):
    pass


def load_input(loader, node):
    return {"input": loader.construct_scalar(node)}


BlueprintLoader.add_constructor("!input", load_input)

path = Path("blueprints/automation/fireangel_wisafe2_alarm_notification.yaml")
with path.open(encoding="utf-8") as stream:
    blueprint = yaml.load(stream, Loader=BlueprintLoader)

metadata = blueprint["blueprint"]
assert metadata["domain"] == "automation"
assert metadata["homeassistant"]["min_version"] == "2024.6.0"
assert set(metadata["input"]["monitored_alarm_types"]["default"]) == {
    "smoke",
    "heat",
    "carbon_monoxide",
}
assert blueprint["triggers"] == [
    {"trigger": "event", "event_type": "state_changed"}
]
condition = blueprint["conditions"][0]["value_template"]
for required in ("FireAngel", "device_class", "new.state == 'on'", "old.state != 'on'"):
    assert required in condition
assert blueprint["mode"] == "parallel"
assert blueprint["actions"][-1]["choose"][0]["sequence"] == {
    "input": "notification_actions"
}

print("Home Assistant alarm-notification blueprint structure is valid")
