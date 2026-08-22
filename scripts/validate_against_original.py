#!/usr/bin/env python3
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from tests.reference_c import build_reference_library  # noqa: E402
from tests.test_against_original import (  # noqa: E402
    NOTICEABLE_THRESHOLDS,
    OUTPUT_NAMES,
    PASS_THRESHOLDS,
    error_matrix,
    validation_cases,
)


def main() -> int:
    reference = build_reference_library(PROJECT_ROOT / ".build" / "reference-c")
    cases = validation_cases()
    errors = error_matrix(reference)
    maximum = np.max(errors, axis=0)
    mean = np.mean(errors, axis=0)

    print(f"Reference cases: {len(cases)}")
    print("output       max abs error       mean abs error      pass threshold")
    for name, max_error, mean_error, threshold in zip(
        OUTPUT_NAMES, maximum, mean, PASS_THRESHOLDS, strict=True
    ):
        print(f"{name:<8} {max_error:>18.10e} {mean_error:>18.10e} {threshold:>18.10e}")

    noticeable_rows = np.flatnonzero(
        np.any(errors > NOTICEABLE_THRESHOLDS, axis=1)
    )
    print(f"\nNoticeable-difference points: {len(noticeable_rows)}")
    for row in noticeable_rows:
        case = cases[int(row)]
        formatted_errors = ", ".join(
            f"{name}={value:.3e}"
            for name, value in zip(OUTPUT_NAMES, errors[row], strict=True)
        )
        print(
            f"  #{row}: phi1={case.phi1:.9f}, phi4={case.phi4:.9f}; "
            f"{formatted_errors}"
        )

    failed = (not np.all(np.isfinite(errors))) or bool(
        np.any(maximum >= PASS_THRESHOLDS)
    )
    print(f"\nValidation: {'FAILED' if failed else 'PASSED'}")
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
