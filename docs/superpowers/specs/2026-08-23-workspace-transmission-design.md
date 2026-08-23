# Workspace And Transmission Analysis Design

## Scope

Analyze the verified original five-bar geometry without changing link lengths or
adding actuator-specific constraints. Scan `phi1, phi4` on a deterministic
`360 x 360` grid over `[-pi, pi)`, retain the positive-root assembly branch,
and produce posture data, five explanatory figures, tests, and README findings.

## Stable Geometry And Jacobians

The scan uses the same C-point branch as `leg_func_calc.m`, evaluated with the
equivalent two-circle intersection formula. This avoids treating the half-angle
coordinate chart's `A0+C0=0` cancellation as a mechanical singularity.

For each posture, differentiate both loop constraints to obtain the Cartesian
Jacobian `Jxy = d(xc,yc)/d(phi1,phi4)`. Convert it to the verified virtual-leg
Jacobian:

```text
J = [e_r^T; e_t^T/l0] Jxy
```

Regular samples must match the first-phase SymPy Jacobian. The raw `J` metrics
are exported because they were requested, but its rows have different units.
Primary singularity analysis therefore uses the dimensionally homogeneous
physical Jacobian:

```text
J_phys = diag(1,l0) J = [e_r^T; e_t^T] Jxy
```

`J_phys` maps joint speed to radial and tangential foot speed. Its smallest
singular value has units m/rad and its 2-norm condition number is dimensionless.
It has exactly the same rank-loss locations as `J` for `l0>0`.

## Metrics

Every scan row contains joint angles, C coordinates, `l0`, `phi0`, raw and
physical singular values/condition numbers, determinant, classification, and:

```text
F_axial_max = 1 / max(|J11|, |J12|)
v_extension_max = |J11| + |J12|
```

The force formula imposes `Tp=0` and `|T1|,|T2|<=1 N m`. The speed formula
maximizes `|dL|` under `|dphi1|,|dphi4|<=1 rad/s`; simultaneous `dPhi` is
allowed as requested. Infinite theoretical force at exactly zero radial row is
retained in data but clipped only for plot color scales.

The upright subset is `|wrap(phi0-pi/2)|<=5 deg`. Workspace classes are purely
kinematic, since collision and hard joint limits are unknown:

- recommended: `condition_number<=5` and `sigma_min>=0.01 m/rad`;
- usable: `condition_number<20` and `sigma_min>=0.002 m/rad`;
- near-singular/not recommended: every other legal posture.

## Artifacts

- `src/analysis.py`: stable single-posture metrics and grid scan.
- `scripts/analyze_workspace.py`: regenerate compressed CSV, summary JSON, and
  figures.
- `artifacts/phase2/workspace_pose.png`: XY workspace with `l0` and `phi0`.
- `artifacts/phase2/jacobian_singularity.png`: physical `sigma_min`, condition
  number, and highlighted near-singular samples in joint and XY space.
- `artifacts/phase2/upright_condition.png`: `l0` versus condition number for the
  upright band, including binned median and 10-90% envelope.
- `artifacts/phase2/force_speed_transmission.png`: normalized pure axial force
  and maximum extension speed maps.
- `artifacts/phase2/workspace_classification.png`: simple three-class summary in
  joint space and XY space.
- `artifacts/phase2/workspace_scan.csv.gz`: all posture rows.
- `artifacts/phase2/summary.json`: exact scan configuration and aggregate facts.

## Visualization Rules

Use equal XY axes, perceptually ordered sequential maps, logarithmic scales for
wide-range condition/force values, direct threshold annotations, and a fixed
green/amber/red category palette backed by labels. Plot clipping never changes
CSV values. Static PNG is the primary delivery; README text provides the main
takeaway and physical meaning for every figure.

## Verification

Tests cover circle closure, stable-vs-SymPy pose and Jacobian agreement,
singular-value ordering, force/velocity optimization formulas against corner
enumeration, exact grid row count, classification boundaries, artifact schema,
and CLI figure generation. Final checks are `ruff`, `mypy`, full pytest, a
fresh 360-grid analysis run, nonempty image inspection, and README/result
consistency.
