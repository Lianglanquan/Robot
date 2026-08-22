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
