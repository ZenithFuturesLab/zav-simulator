# high_speed / Super-LIO penalty tuning — 2026-11-13

GUI Isaac Sim + RViz only. Repeated verification is required before final adoption. Failed or partially landed flights are excluded from time ranking.

| Trial | v/a/j caps | Healthy complete | Time sim s (eligible only) | Actual peak / p95 m/s | Peak/cap | LIO max m | Clearance lower bound m |
|---|---|---|---:|---|---:|---:|---:|
| [winner-v8](winner-v8/evaluation.json) | 8.0/2.5/10.0 | False | — | 8.371 / 8.112 | 1.046 | 2.296 | -0.285 |
| [baseline-positive](baseline-positive/evaluation.json) | 5.5/2.0/8.0 | True | 45.30 | 5.667 / 5.567 | 1.030 | 0.416 | 0.472 |
| [time_pressure-v65](time_pressure-v65/evaluation.json) | 6.5/2.5/10.0 | False | — | 6.676 / 6.607 | 1.027 | 0.435 | 0.296 |
| [balanced-v725-omg06](balanced-v725-omg06/evaluation.json) | 7.25/2.5/10.0 | False | — | 7.601 / 7.388 | 1.048 | 2.054 | 0.189 |
| [verify-baseline-positive-3](verify-baseline-positive-3/evaluation.json) | 5.5/2.0/8.0 | True | 45.89 | 5.640 / 5.566 | 1.025 | 0.351 | 0.568 |
| [winner-v725](winner-v725/evaluation.json) | 7.25/2.5/10.0 | False | — | 7.678 / 7.325 | 1.059 | 0.521 | 0.041 |
| [smooth_constraints-v65](smooth_constraints-v65/evaluation.json) | 6.5/2.5/10.0 | False | — | 6.801 / 6.559 | 1.046 | 1.970 | -0.229 |
| [verify-balanced_smooth-v65-2](verify-balanced_smooth-v65-2/evaluation.json) | 6.5/2.5/10.0 | True | 39.80 | 6.765 / 6.623 | 1.041 | 0.414 | 0.686 |
| [original-v65](original-v65/evaluation.json) | 6.5/2.5/10.0 | False | — | 6.785 / 6.683 | 1.044 | 2.360 | 0.536 |
| [route_fidelity-v65](route_fidelity-v65/evaluation.json) | 6.5/2.5/10.0 | False | — | 6.371 / 6.123 | 0.980 | 0.297 | 0.544 |
| [verify-balanced_smooth-v65-1](verify-balanced_smooth-v65-1/evaluation.json) | 6.5/2.5/10.0 | True | 39.80 | 6.943 / 6.703 | 1.068 | 0.352 | 0.376 |
| [backup_smooth-v65](backup_smooth-v65/evaluation.json) | 6.5/2.5/10.0 | False | — | 6.783 / 6.640 | 1.044 | 2.297 | 0.384 |
| [time_pressure_exp-v65](time_pressure_exp-v65/evaluation.json) | 6.5/2.5/10.0 | True | 39.80 | 6.762 / 6.588 | 1.040 | 0.467 | 0.392 |
| [balanced-v8-mpc02](balanced-v8-mpc02/evaluation.json) | 8.0/2.5/10.0 | False | — | 8.190 / 7.996 | 1.024 | 2.097 | 0.027 |
| [relaxed_jerk-v65](relaxed_jerk-v65/evaluation.json) | 6.5/2.5/10.0 | False | — | 6.758 / 6.521 | 1.040 | 0.513 | 0.151 |
| [verify-time_pressure_exp-v65-1](verify-time_pressure_exp-v65-1/evaluation.json) | 6.5/2.5/10.0 | False | — | 6.941 / 6.585 | 1.068 | 0.483 | 0.080 |
| [balanced-v75](balanced-v75/evaluation.json) | 7.5/2.5/10.0 | True | 40.70 | 7.823 / 7.635 | 1.043 | 0.770 | 0.235 |
| [balanced-v8](balanced-v8/evaluation.json) | 8.0/2.5/10.0 | False | — | 8.126 / 8.043 | 1.016 | 2.171 | 0.668 |
| [balanced_smooth-v65](balanced_smooth-v65/evaluation.json) | 6.5/2.5/10.0 | True | 41.15 | 6.746 / 6.587 | 1.038 | 0.373 | 0.258 |
| [verify-baseline-positive-1](verify-baseline-positive-1/evaluation.json) | 5.5/2.0/8.0 | True | 45.69 | 5.723 / 5.616 | 1.041 | 0.389 | 0.532 |
| [verify-baseline-positive-2](verify-baseline-positive-2/evaluation.json) | 5.5/2.0/8.0 | True | 45.25 | 5.655 / 5.582 | 1.028 | 0.464 | 0.541 |
| [balanced-v725](balanced-v725/evaluation.json) | 7.25/2.5/10.0 | True | 38.10 | 7.470 / 7.355 | 1.030 | 0.376 | 0.593 |
| [verify-balanced-v725-1](verify-balanced-v725-1/evaluation.json) | 7.25/2.5/10.0 | False | — | 7.586 / 7.419 | 1.046 | 2.488 | -0.098 |
| [verify-balanced_smooth-v65-3](verify-balanced_smooth-v65-3/evaluation.json) | 6.5/2.5/10.0 | True | 39.20 | 6.930 / 6.624 | 1.066 | 0.565 | 0.662 |
