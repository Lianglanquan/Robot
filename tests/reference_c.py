import ctypes
import subprocess
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]
PROJECT_ROOT = Path(__file__).resolve().parents[1]


class CReference:
    """Typed wrapper around the unmodified MATLAB Coder functions."""

    def __init__(self, library_path: Path) -> None:
        self._library = ctypes.CDLL(str(library_path))
        float_pointer = ctypes.POINTER(ctypes.c_float)
        self._library.leg_pos.argtypes = [
            ctypes.c_float,
            ctypes.c_float,
            float_pointer,
        ]
        self._library.leg_pos.restype = None
        self._library.leg_spd.argtypes = [
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            float_pointer,
        ]
        self._library.leg_spd.restype = None
        self._library.leg_conv.argtypes = [
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            float_pointer,
        ]
        self._library.leg_conv.restype = None

    @staticmethod
    def _output_array() -> ctypes.Array[ctypes.c_float]:
        return (ctypes.c_float * 2)()

    @staticmethod
    def _as_numpy(values: ctypes.Array[ctypes.c_float]) -> FloatArray:
        return np.array([values[0], values[1]], dtype=float)

    def leg_pos(self, phi1: float, phi4: float) -> FloatArray:
        output = self._output_array()
        self._library.leg_pos(phi1, phi4, output)
        return self._as_numpy(output)

    def leg_spd(
        self, dphi1: float, dphi4: float, phi1: float, phi4: float
    ) -> FloatArray:
        output = self._output_array()
        self._library.leg_spd(dphi1, dphi4, phi1, phi4, output)
        return self._as_numpy(output)

    def leg_conv(
        self, force: float, virtual_torque: float, phi1: float, phi4: float
    ) -> FloatArray:
        output = self._output_array()
        self._library.leg_conv(force, virtual_torque, phi1, phi4, output)
        return self._as_numpy(output)


def build_reference_library(build_directory: Path) -> CReference:
    build_directory.mkdir(parents=True, exist_ok=True)
    library_path = build_directory / "libwheel_leg_reference.so"
    source_directory = PROJECT_ROOT / "reference" / "c"
    subprocess.run(
        [
            "gcc",
            "-shared",
            "-fPIC",
            "-O2",
            str(source_directory / "leg_pos.c"),
            str(source_directory / "leg_spd.c"),
            str(source_directory / "leg_conv.c"),
            "-lm",
            "-o",
            str(library_path),
        ],
        check=True,
    )
    return CReference(library_path)
