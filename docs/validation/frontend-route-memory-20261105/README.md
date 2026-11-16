# Frontend route persistence check

`off-01.jsonl.gz` is the reference case: OMMPC, dense, 1m/s, yaw rate cap 1rad/s, route memory OFF, original cylinder/0.6m inflation. It reached the 180s outbound goal timeout, then landed and disarmed without forest contacts. Actual frontend MarkerArray paths were recorded.

The opt-in route-memory implementation passed its forward-join, pose-jitter, horizon, obstacle invalidation and goal-reset unit test. There is **no route-memory ON flight validation in this folder yet**. The user then requested inspection of physical dimensions and inflation; see [footprint audit](../zav-footprint-20261105/README.md). Do not claim this experiment proves route switching was fixed.
