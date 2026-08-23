# Candidate actuator evidence

Retrieved: 2026-08-23

This file records source facts used by the stair decision gate. Product claims
are not treated as independent test results.

## EduLite 05 / EL05

- Manufacturer: RobStride Dynamics
- Official product page:
  <https://www.robstride.com/products/eduLite05>
- Official product-information repository:
  <https://github.com/RobStride/Product_Information>
- Inspected repository commit:
  `6ad12f50006273b7ea4eea88980f927d97c22f0d`
- Inspected manual:
  `产品资料/EL05/EL05使用说明书260713.pdf`

The 2026-07-13 manual states:

| Item | Published value |
|---|---:|
| Size | diameter 46 x 44 mm |
| Mass | 242 +/- 3 g |
| Reduction | 9:1 |
| Rated voltage | 48 V |
| Voltage range | 15--60 V |
| Rated torque | 1.8 N m at 100 rpm, with 70 x 70 mm aluminium heat sink |
| Rated output power | approximately 19 W from the rated point |
| Peak torque | 6 N m |
| No-load speed | 430 rpm +/- 10% |
| Rated / peak phase current | 2.6 / 11 Apk |

The manual includes a 48 V torque-speed plot but no numerical table. The
decision-gate figure visually digitizes that plot only for screening. It also
shows rotating overload durations of 5 s at 6 N m, 7 s at 5 N m, 14 s at
4 N m, 44 s at 3 N m and 300 s at 2 N m under its stated cooling conditions.
These are thermal protection data, not gearbox shock ratings.

The manual explicitly says that externally driven damping generates electrical
energy and requires a power supply capable of accepting it to prevent
overvoltage. Its fault table trips above 60 V. RobStride's official discharge
module documentation provides separate 24 V and 48 V clamp modes. Therefore a
landing calculation must include the DC-bus energy path; motor torque alone is
not an adequate acceptance check.

## QD4310

- Manufacturer page: <https://www.qdrive.com.cn/products/qd4310/>
- Official machine-readable specification:
  <https://www.qdrive.com.cn/downloads/products/qd4310/page-data.json>
- Official manual:
  <https://www.qdrive.com.cn/downloads/products/qd4310/QD4310使用手册.pdf>

The 2026-07-28 manual and specification state:

| Item | Published value |
|---|---:|
| Mass | approximately 127 g |
| Voltage range | 7--26 V |
| Rated / peak current | 1 / 2 A |
| Rated / peak torque | 0.2 / 0.3 N m |
| Rated / peak speed | 500 / 800 rpm |
| Torque constant | 0.27 N m/A |

No manufacturer torque-speed curve or impact/edge-climbing rating was found.
The peak values must not be assumed to occur simultaneously.
The current product page publishes `contact@qdrive.example`, which is not a
deliverable email domain. This does not invalidate the manual, but vendor
support and hardware-revision maturity should be verified before bulk purchase.

## Evidence limits

- The candidate six motors have a published combined mass of 1.222 kg before
  brackets, wiring, heat sinks, power conversion or a braking resistor.
- EL05 performance is specified at 48 V while QD4310 is limited to 26 V. A
  single unconverted bus cannot operate both at their published voltage ranges.
- Neither manufacturer specification proves collision-free installation in the
  upstream SolidWorks assembly.
- Neither specification is a substitute for a measured torque-speed curve,
  impact test, efficiency map, backlash measurement or regeneration test on the
  purchased hardware revision.
