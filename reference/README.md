# Upstream reference

The files in this directory are unmodified copies from:

- Repository: <https://github.com/Skythinker616/foc-wheel-legged-robot>
- Commit: `e2444395dd3a76c20b0683fbb1e123c21186a502`
- Commit date: 2023-11-23
- Retrieved: 2026-08-23
- License: GNU General Public License v3.0 (see `LICENSE.upstream`)

`matlab/leg_func_calc.m` is the mathematical source of the five-bar model.
The three pairs under `c/` are the single-precision functions produced by
MATLAB Coder and used by the ESP32 controller. Tests compile those C files
without changing their expressions and call them through `ctypes`.

The upstream `matlab/README.md` is retained for context. Only files required to
understand or execute this phase's reference model are included here; the full
upstream repository was inspected at the commit above.

## Phase 3 evidence audit

The continuous-stroke study also inspected the complete upstream tree at the
same pinned commit. The relevant evidence is deliberately separated from the
mathematical conclusions:

- The controller initializes `target.legLength` to 0.070 m and maps the Android
  slider to 0.070--0.090 m:
  [`main.cpp` lines 542--545](https://github.com/Skythinker616/foc-wheel-legged-robot/blob/e2444395dd3a76c20b0683fbb1e123c21186a502/esp32-controller/software/src/main.cpp#L542-L545),
  [`main.cpp` lines 786--790](https://github.com/Skythinker616/foc-wheel-legged-robot/blob/e2444395dd3a76c20b0683fbb1e123c21186a502/esp32-controller/software/src/main.cpp#L786-L790),
  [`CtrlView.java` lines 184--188](https://github.com/Skythinker616/foc-wheel-legged-robot/blob/e2444395dd3a76c20b0683fbb1e123c21186a502/android/app/src/main/java/com/skythinker/balancebot/CtrlView.java#L184-L188),
  and
  [`MainActivity.java` lines 234--240](https://github.com/Skythinker616/foc-wheel-legged-robot/blob/e2444395dd3a76c20b0683fbb1e123c21186a502/android/app/src/main/java/com/skythinker/balancebot/MainActivity.java#L234-L240).
- When airborne or cushioning, the controller commands 0.120 m and adds a soft
  restoring force above 0.120 m:
  [`main.cpp` lines 610--620](https://github.com/Skythinker616/foc-wheel-legged-robot/blob/e2444395dd3a76c20b0683fbb1e123c21186a502/esp32-controller/software/src/main.cpp#L610-L620).
  This is control intent, not a measured hard stop or collision limit.
- The controller's angle protection limits virtual-leg angle relative to the
  body and body pitch, not the two active-joint angles:
  [`main.cpp` lines 653--658](https://github.com/Skythinker616/foc-wheel-legged-robot/blob/e2444395dd3a76c20b0683fbb1e123c21186a502/esp32-controller/software/src/main.cpp#L653-L658).
- The SolidWorks assembly and renders identify the installed assembly mode:
  both short actuated links point outward and the two long links converge on
  the wheel axle. They do not document a checked collision envelope or active
  joint hard limits.
- `matlab/leg_sim.slx` was inspected as a ZIP/XML Simulink archive. All revolute
  joint limit flags in `simulink/systems/system_59.xml` are off. Its saved scope
  display bounds, 0.05431--0.12523 m, are plot settings rather than mechanical
  limits.
- `matlab/sys_calc.m` evaluates an LQR model from 0.04 to 0.14 m. That is a
  model-fitting interval, not evidence that every length is mechanically usable.

These distinctions are used in the Phase 3 report: 70--90 mm is an
evidence-backed user command range, 120 mm is an evidence-backed controller
target/soft guard, while the actual collision-free continuous stroke remains
unverified without CAD collision checks or hardware measurements.
