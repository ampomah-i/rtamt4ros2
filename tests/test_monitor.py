import math
from types import SimpleNamespace

import pytest

from rtamt4ros2.monitor import (
    MonitorConfigurationError,
    OnlineStlMonitor,
    extract_numeric_field,
)


def test_atomic_predicate_robustness_and_tick():
    monitor = OnlineStlMonitor("out = x > 0.5", ["x"], 0.1)

    assert monitor.update({"x": 1.25}) == pytest.approx(0.75)
    assert monitor.tick == 1
    assert monitor.update({"x": 0.25}) == pytest.approx(-0.25)
    assert monitor.tick == 2


def test_bounded_always_is_evaluated_online():
    monitor = OnlineStlMonitor("out = always[0,2](x > 0.5)", ["x"], 0.1)

    robustness = [monitor.update({"x": value}) for value in (1.0, 0.75, 0.0)]

    assert robustness == pytest.approx([0.5, 0.25, -0.5])


@pytest.mark.parametrize(
    ("formula", "variables", "period", "message"),
    [
        ("", ["x"], 0.1, "formula"),
        ("out = x > 0", [], 0.1, "at least one"),
        ("out = x > 0", ["x", "x"], 0.1, "unique"),
        ("out = x > 0", ["out"], 0.1, "reserved"),
        ("out = x > 0", ["x"], 0.0, "positive"),
        ("out = x > 0", ["x"], math.inf, "positive"),
        ("this is not STL", ["x"], 0.1, "invalid STL"),
    ],
)
def test_invalid_configuration_is_rejected(formula, variables, period, message):
    with pytest.raises(MonitorConfigurationError, match=message):
        OnlineStlMonitor(formula, variables, period)


def test_sample_names_must_match_declared_variables():
    monitor = OnlineStlMonitor("out = x + y", ["x", "y"], 0.1)

    with pytest.raises(ValueError, match="missing: y"):
        monitor.update({"x": 1.0})
    with pytest.raises(ValueError, match="unknown: z"):
        monitor.update({"x": 1.0, "y": 2.0, "z": 3.0})


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_non_finite_samples_are_rejected(value):
    monitor = OnlineStlMonitor("out = x", ["x"], 0.1)

    with pytest.raises(ValueError, match="finite"):
        monitor.update({"x": value})
    assert monitor.tick == 0


def test_nested_numeric_field_extraction():
    message = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=2.5)))

    assert extract_numeric_field(message, "pose.position.x") == pytest.approx(2.5)


def test_invalid_sampling_unit_is_rejected():
    with pytest.raises(MonitorConfigurationError, match="sampling_unit"):
        OnlineStlMonitor("out = x", ["x"], 1.0, "minutes")
