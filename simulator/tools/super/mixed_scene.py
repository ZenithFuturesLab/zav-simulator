"""Reproducible open/dense/open benchmark, made of whole upstream obstacles.

The PCD is an Isaac geometry input only. No map, zone label or ground truth is
sent to SUPER; the same onboard sensors and common YAML serve every zone.
"""
from pathlib import Path
import hashlib
import json
import numpy as np
from scipy.ndimage import label

try:
    from .pcd_scene import read_pcd
except ImportError:
    from pcd_scene import read_pcd


FILENAME = 'mixed_sparse_dense.pcd'
SOURCES = {'sparse': 'random_map_24_6635.pcd', 'dense': 'random_map_2_26609.pcd'}
ZONES = [dict(name='open_south', y_min=-55., y_max=-22.),
         dict(name='transition_south', y_min=-22., y_max=-18.),
         dict(name='dense_center', y_min=-18., y_max=18.),
         dict(name='transition_north', y_min=18., y_max=22.),
         dict(name='open_north', y_min=22., y_max=55.)]


def components(points, resolution=.2):
    """Group touching XY silhouettes, preserving merged trees as a whole."""
    q = np.floor(points[:, :2].astype(np.float64) / resolution).astype(int)
    low = q.min(axis=0)
    ij = q - low
    grid = np.zeros(tuple(ij.max(axis=0) + 1), dtype=bool)
    grid[ij[:, 0], ij[:, 1]] = True
    labels, count = label(grid, np.ones((3, 3), dtype=int))
    ids = labels[ij[:, 0], ij[:, 1]]
    return [points[ids == i] for i in range(1, count + 1)]


def ensure_mixed_map(directory):
    directory = Path(directory)
    output, recipe_path = directory / FILENAME, directory / 'mixed_recipe.json'
    signature = dict(format_version=1, component_resolution_m=.2,
                     source_sha256={k: hashlib.sha256((directory / f).read_bytes()).hexdigest()
                                    for k, f in SOURCES.items()})
    if output.exists() and recipe_path.exists():
        old = json.loads(recipe_path.read_text())
        if all(old.get(k) == v for k, v in signature.items()) and old.get(
                'generated_pcd_sha256') == hashlib.sha256(output.read_bytes()).hexdigest():
            return output
    sparse = read_pcd(directory / SOURCES['sparse'])
    dense = read_pcd(directory / SOURCES['dense'])
    floor = sparse[np.abs(sparse[:, 2]) < 1e-7]
    selected, groups = [], []
    for origin, source in [('sparse', sparse), ('dense', dense)]:
        obstacles = source[np.abs(source[:, 2]) >= 1e-7]
        for group_id, p in enumerate(components(obstacles), 1):
            low, high = p.min(axis=0), p.max(axis=0)
            if origin == 'sparse':
                if not (high[1] <= -22. or low[1] >= 22.):
                    continue
                # Valid mission pads, not an obstacle-free through-corridor.
                if any(np.linalg.norm(p[:, :2] - [0., y], axis=1).min() < 2.5
                       for y in [-50., 50.]):
                    continue
                translated = p.copy()
            else:
                # This source interval contains a connected dense forest block.
                # Keep complete groups: never slice a trunk at the seam.
                if low[1] < -40. or high[1] > -4.:
                    continue
                translated = p.copy()
                translated[:, 1] += 22.
            selected.append(translated)
            groups.append(dict(source=origin, group_id=group_id, points=len(p),
                               bounds_min=translated.min(axis=0).tolist(),
                               bounds_max=translated.max(axis=0).tolist()))
    result = np.concatenate([floor] + selected).astype(np.float32)
    temporary = output.with_suffix('.tmp.pcd')
    with temporary.open('w') as f:
        f.write('# Reproducible SUPER mixed sparse/dense surface map\nVERSION .7\n'
                'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                f'WIDTH {len(result)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                f'POINTS {len(result)}\nDATA ascii\n')
        np.savetxt(f, result, fmt='%.9g')
    temporary.replace(output)
    recipe = dict(signature, zones=ZONES, groups=groups, points=len(result),
                  generated_pcd_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
                  endpoint_clearance_radius_m=2.5,
                  endpoint_positions=[[0., -50., 1.5], [0., 50., 1.5]],
                  source_dense_y_interval=[-40., -4.], dense_translation=[0., 22., 0.],
                  selection='Complete connected XY obstacle groups; no central corridor cleared',
                  planner_access='Sensor observations only; no PCD injection or density switching')
    recipe_path.write_text(json.dumps(recipe, indent=2) + '\n')
    return output


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path,
                        default=Path(__file__).resolve().parents[2] / 'assets/super')
    args = parser.parse_args()
    print(ensure_mixed_map(args.directory))
