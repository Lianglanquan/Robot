from dataclasses import dataclass


@dataclass(frozen=True)
class FiveBarParameters:
    """Five-bar link lengths in metres."""

    l1: float
    l2: float
    l3: float
    l4: float
    l5: float


DEFAULT_PARAMETERS = FiveBarParameters(
    l1=0.050,
    l2=0.105,
    l3=0.105,
    l4=0.050,
    l5=0.060,
)
