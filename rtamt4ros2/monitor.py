"""ROS-independent online STL monitoring primitives."""

from math import isfinite
from typing import Dict, Iterable, Mapping, Optional, Sequence

import rtamt


class MonitorConfigurationError(ValueError):
    """Raised when an STL monitor cannot be configured."""


def extract_numeric_field(message, field_path: str) -> float:
    """Extract a numeric value from a message using a dotted attribute path."""
    value = message
    for attribute in field_path.split("."):
        if not attribute:
            raise AttributeError("field path components must not be empty")
        value = getattr(value, attribute)
    return float(value)


class OnlineStlMonitor:
    """Evaluate a pastified discrete-time STL specification one sample at a time."""

    def __init__(
        self,
        formula: str,
        variables: Sequence[str],
        sampling_period: float,
        sampling_unit: str = "s",
        *,
        specification=None,
    ) -> None:
        self.formula = formula.strip()
        self.variables = self._validate_variables(variables)
        self.sampling_period = float(sampling_period)
        self.sampling_unit = str(sampling_unit).strip()
        if not self.formula:
            raise MonitorConfigurationError("formula must not be empty")
        if not isfinite(self.sampling_period) or self.sampling_period <= 0.0:
            raise MonitorConfigurationError("sampling_period must be a positive finite number")
        if self.sampling_unit not in {"s", "ms", "us", "ns"}:
            raise MonitorConfigurationError("sampling_unit must be one of: s, ms, us, ns")

        spec = specification or rtamt.StlDiscreteTimeSpecification()
        try:
            for variable in self.variables:
                spec.declare_var(variable, "float")
            spec.declare_var("out", "float")
            spec.spec = self.formula
            spec.parse()
            spec.pastify()
            spec.set_sampling_period(self.sampling_period, self.sampling_unit)
        except Exception as exc:
            raise MonitorConfigurationError(
                f"invalid STL specification {self.formula!r}: {exc}"
            ) from exc

        self._spec = spec
        self._tick = 0

    @staticmethod
    def _validate_variables(variables: Iterable[str]) -> tuple[str, ...]:
        names = tuple(str(name).strip() for name in variables)
        if not names:
            raise MonitorConfigurationError("vars must contain at least one variable")
        if any(not name for name in names):
            raise MonitorConfigurationError("variable names must not be empty")
        if "out" in names:
            raise MonitorConfigurationError("'out' is reserved for the specification output")
        if len(names) != len(set(names)):
            raise MonitorConfigurationError("variable names must be unique")
        return names

    @property
    def tick(self) -> int:
        return self._tick

    def update(self, values: Mapping[str, float]) -> float:
        missing = set(self.variables).difference(values)
        extra = set(values).difference(self.variables)
        if missing or extra:
            details = []
            if missing:
                details.append(f"missing: {', '.join(sorted(missing))}")
            if extra:
                details.append(f"unknown: {', '.join(sorted(extra))}")
            raise ValueError("sample variables do not match the specification (" + "; ".join(details) + ")")

        sample: Dict[str, float] = {}
        for name in self.variables:
            value = float(values[name])
            if not isfinite(value):
                raise ValueError(f"sample for {name!r} must be finite")
            sample[name] = value

        robustness = float(self._spec.update(self._tick, list(sample.items())))
        self._tick += 1
        return robustness
