# Five-Bar Leg Model Design

## Scope

Reimplement the five-bar leg model from
`Skythinker616/foc-wheel-legged-robot` at commit
`e2444395dd3a76c20b0683fbb1e123c21186a502`. This phase covers forward
kinematics, the analytic Jacobian, velocity mapping, VMC torque mapping,
numerical validation against the generated C functions, and a static geometry
plot. Workspace analysis, optimization, simulation, control, and robot redesign
are out of scope.

## Geometry And Conventions

The fixed active pivots are A = `(0, 0)` and E = `(l5, 0)`. The moving points
are B = A + `l1 * [cos(phi1), sin(phi1)]`, D = E +
`l4 * [cos(phi4), sin(phi4)]`, and foot point C. Links BC and DC have lengths
`l2` and `l3`. All angles are radians and increase counterclockwise from the
positive x axis.

The C point must use the same assembly branch as `matlab/leg_func_calc.m`:

```text
A0 = 2*l2*(xd-xb)
B0 = 2*l2*(yd-yb)
C0 = l2**2 + (xd-xb)**2 + (yd-yb)**2 - l3**2
phi2 = 2*atan((B0 + sqrt(A0**2 + B0**2 - C0**2)) / (A0 + C0))
C = B + l2*[cos(phi2), sin(phi2)]
```

The virtual leg starts at O = `(l5/2, 0)` and ends at C. Its pose is
`l0 = hypot(xc-l5/2, yc)` and `phi0 = atan2(yc, xc-l5/2)`.

## Components

- `src/parameters.py`: immutable link-length parameters and the upstream
  defaults.
- `src/kinematics.py`: symbolic source expressions, cached NumPy callables,
  forward kinematics, joint positions, analytic Jacobian, and finite-difference
  Jacobian used only as a check.
- `src/vmc.py`: velocity and virtual-force mappings using the analytic
  Jacobian.
- `tests/reference_c.py`: compile and load the unmodified generated C reference
  functions through `ctypes`.
- `tests/test_against_original.py`: deterministic and seeded-random legal
  configurations, finite-difference checks, and C comparisons.
- `scripts/validate_against_original.py`: repeatable error-statistics report,
  including all points above a documented threshold.
- `scripts/visualize_leg.py`: command-line static plot of the five-bar geometry.
- `reference/`: upstream MATLAB/C sources, license, provenance, and validation
  notes.

## Numerical Behavior

The Python implementation evaluates in NumPy `float64`. The reference C API
accepts and returns IEEE-754 `float`; test inputs are explicitly rounded to
`float32` before both implementations are called so input values are identical.
Samples are legal when the MATLAB closure radicand is positive and every
requested Python and C result is finite. Points near closure and Jacobian
singularities are reported separately rather than used to set loose tolerances.

The analytic Jacobian is created by applying SymPy `diff` to exactly the two
pose expressions. Finite differences use central differences away from angle
branch cuts and singularities. Velocity is `J @ [dphi1, dphi4]`; torque is
`J.T @ [F, Tp]`.

## Verification

The original C files are compiled as a shared library with GCC and `-lm`.
Tests compare each scalar output for position, velocity, and torque over fixed
representative points plus a deterministic random legal sample. The validation
script reports maximum and mean absolute error per output and lists every point
that exceeds its stated threshold. Tests also check link closure, the analytic
Jacobian against finite differences, and virtual-work consistency.

The README records actual results from a fresh validation run, the exact
upstream commit, known branch/singularity behavior, commands, and one generated
PNG. No claim of equivalence is made until the C comparison and full test suite
pass.
