These two original PCD files are copied without modification from
hku-mars/SUPER commit 2ad3419c127a617c6d7df6925e81a14175a9c096,
mars_uav_sim/perfect_drone_sim/pcd/:

- random_map_24_6635.pcd — high_speed example
- random_map_2_26609.pcd — dense example

Only these two benchmark inputs are vendored. Generated USD/JSON cache files
are ignored by Git and recreated by simulator/tools/super/pcd_scene.py.
The generated mixed map adds `mixed_sparse_dense.pcd` and `mixed_recipe.json`.
`simulator/tools/super/mixed_scene.py` selects whole connected obstacle groups:
dense source y=[-40,-4] shifted +22m to the center, sparse groups outside
y=[-22,22], with clear endpoint pads. Original inputs remain unchanged.
The simulator creates this reproducible asset and its USD cache on first use.
See docs/super-mixed-map.md for the common-YAML GUI mission and measured results.

The planner never reads these PCDs; Isaac's mounted LiDAR observes the converted
physical surfaces. See docs/super-navigation.md for reconstruction error and
continuous-floor handling.
