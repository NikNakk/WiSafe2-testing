#!/usr/bin/env python3
from pathlib import Path

import yaml


class BlueprintLoader(yaml.SafeLoader):
    pass


def load_input(loader, node):
    return {"input": loader.construct_scalar(node)}


BlueprintLoader.add_constructor("!input", load_input)


def load_blueprint(filename):
    path = Path("blueprints/automation") / filename
    with path.open(encoding="utf-8") as stream:
        blueprint = yaml.load(stream, Loader=BlueprintLoader)
    metadata = blueprint["blueprint"]
    assert metadata["domain"] == "automation"
    assert metadata["homeassistant"]["min_version"] == "2024.6.0"
    assert blueprint["triggers"] == [
        {"trigger": "event", "event_type": "state_changed"}
    ]
    assert blueprint["mode"] == "parallel"
    assert blueprint["actions"][-1]["choose"][0]["sequence"] == {
        "input": "notification_actions"
    }
    return blueprint


alarm = load_blueprint("fireangel_wisafe2_alarm_notification.yaml")
assert set(alarm["blueprint"]["input"]["monitored_alarm_types"]["default"]) == {
    "smoke",
    "heat",
    "carbon_monoxide",
}
alarm_condition = alarm["conditions"][0]["value_template"]
for required in (
    "FireAngel",
    "device_class",
    "new.state == 'on'",
    "old.state != 'on'",
):
    assert required in alarm_condition

test = load_blueprint("fireangel_wisafe2_test_notification.yaml")
test_condition = test["conditions"][0]["value_template"]
for required in (
    "FireAngel",
    "device_class == 'timestamp'",
    "old is none or old.state != new.state",
    "age <= 300",
):
    assert required in test_condition
assert test["actions"][0]["delay"] == {"input": "settle_delay"}
test_variables = test["actions"][1]["variables"]
for required in (
    "test_device_name",
    "test_alarm_device_class",
    "test_result",
    "test_event",
    "test_timestamp",
):
    assert required in test_variables

print("Home Assistant notification blueprint structures are valid")
