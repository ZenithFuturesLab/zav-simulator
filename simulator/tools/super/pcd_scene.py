"""Turn upstream and generated mixed PCD surfaces into physical USD meshes.

No point cloud is injected into the planner. Each occupied 5 cm surface voxel
becomes geometry at its original map coordinate; only internal faces are culled.
The PCD is a sampled surface, so reconstruction has a documented voxel error.
"""
from pathlib import Path
import hashlib
import json
import numpy as np

MAPS = {'high_speed': 'random_map_24_6635.pcd', 'dense': 'random_map_2_26609.pcd',
        'mixed': 'mixed_sparse_dense.pcd'}


def scene_pcd(directory, scenario):
    if scenario == 'mixed':
        try:
            from .mixed_scene import ensure_mixed_map
        except ImportError:
            from mixed_scene import ensure_mixed_map
        return ensure_mixed_map(directory)
    return Path(directory) / MAPS[scenario]


def read_pcd(path):
    with Path(path).open('rb') as f:
        header = {}
        while True:
            line = f.readline().decode('ascii').strip()
            if not line or line.startswith('#'):
                continue
            key, *values = line.split()
            header[key] = values
            if key == 'DATA':
                break
        if header['DATA'] != ['ascii']:
            raise ValueError('These upstream benchmark maps must be ASCII PCD')
        data = np.loadtxt(f, dtype=np.float32, ndmin=2)
    fields = header['FIELDS']
    points = data[:, [fields.index(k) for k in ('x', 'y', 'z')]]
    if len(points) != int(header['POINTS'][0]):
        raise ValueError('PCD point count or coordinates invalid')
    return points[np.isfinite(points).all(axis=1)]


def surface_voxels(points, resolution):
    if not 0 < resolution <= .1:
        raise ValueError('Surface resolution must be in (0, 0.1] metres')
    return np.unique(np.floor(points.astype(np.float64) / resolution).astype(np.int32), axis=0)


def build_scene(pcd, output, resolution=.05):
    from pxr import Usd, UsdGeom, UsdPhysics, Vt
    pcd, output = Path(pcd), Path(output)
    signature = {'source_sha256': hashlib.sha256(pcd.read_bytes()).hexdigest(),
                 'resolution_m': resolution, 'format_version': 2}
    manifest = output.with_suffix('.json')
    if output.exists() and manifest.exists() and all(
            json.loads(manifest.read_text()).get(k) == v for k, v in signature.items()):
        return output
    points = read_pcd(pcd)
    # The source contains a regular 10 cm sampling of a perfectly flat floor.
    # Reconstruct that plane exactly; isolated cubes would leave artificial
    # holes/steps and can destabilize the resting vehicle's IMU initialization.
    floor_points = points[np.abs(points[:,2]) < 1e-7]
    voxels = surface_voxels(points[np.abs(points[:,2]) >= 1e-7], resolution)
    occupied = {tuple(v) for v in voxels}
    # Outward winding, one quad per exposed voxel face.
    faces = [((1,0,0), [(1,0,0),(1,1,0),(1,1,1),(1,0,1)]),
             ((-1,0,0), [(0,0,0),(0,0,1),(0,1,1),(0,1,0)]),
             ((0,1,0), [(0,1,0),(0,1,1),(1,1,1),(1,1,0)]),
             ((0,-1,0), [(0,0,0),(1,0,0),(1,0,1),(0,0,1)]),
             ((0,0,1), [(0,0,1),(1,0,1),(1,1,1),(0,1,1)]),
             ((0,0,-1), [(0,0,0),(0,1,0),(1,1,0),(1,0,0)])]
    chunks = {}
    for x,y,z in occupied:
        chunk = (x//160, y//160, z//160)
        vertices = chunks.setdefault(chunk, [])
        for (dx,dy,dz), corners in faces:
            if (x+dx,y+dy,z+dz) not in occupied:
                vertices.extend(((x+a)*resolution,(y+b)*resolution,(z+c)*resolution)
                                for a,b,c in corners)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.stem + '.tmp.usdc')
    stage = Usd.Stage.CreateNew(str(temporary))
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, '/SUPER').GetPrim()
    stage.SetDefaultPrim(root)
    if len(floor_points):
        lower=floor_points.min(axis=0);upper=floor_points.max(axis=0)
        floor=UsdGeom.Mesh.Define(stage, '/SUPER/floor')
        floor.CreatePointsAttr([(float(lower[0]),float(lower[1]),0.),
                                (float(upper[0]),float(lower[1]),0.),
                                (float(upper[0]),float(upper[1]),0.),
                                (float(lower[0]),float(upper[1]),0.)])
        floor.CreateFaceVertexCountsAttr([4]);floor.CreateFaceVertexIndicesAttr([0,1,2,3])
        floor.CreateSubdivisionSchemeAttr('none');floor.CreateDoubleSidedAttr(True)
        floor.CreateDisplayColorAttr([(.25,.28,.3)])
        UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(floor.GetPrim()).CreateApproximationAttr('none')
    for i, vertices in enumerate(chunks.values()):
        mesh = UsdGeom.Mesh.Define(stage, f'/SUPER/surface_{i}')
        values = np.asarray(vertices, dtype=np.float32)
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(values))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(values)//4,4,np.int32)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.arange(len(values),dtype=np.int32)))
        mesh.CreateExtentAttr(UsdGeom.PointBased(mesh).ComputeExtent(values))
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorAttr([(0.42,0.52,0.58)])
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('none')
    stage.GetRootLayer().Save()
    temporary.replace(output)
    signature.update(points=len(points), occupied_voxels=len(voxels), chunks=len(chunks),
                     bounds_min=points.min(axis=0).tolist(), bounds_max=points.max(axis=0).tolist(),
                     max_surface_error_m=float(np.sqrt(3)*resolution))
    manifest.write_text(json.dumps(signature, indent=2)+'\n')
    return output


def add_super_scene(world, scenario):
    from pxr import UsdGeom
    directory = Path(__file__).resolve().parents[2] / 'assets/super'
    path = build_scene(scene_pcd(directory, scenario), directory / f'{scenario}.usdc')
    world.stage.DefinePrim('/World/layout','Xform').GetReferences().AddReference(str(path))
    print(f'[SUPER] Physical PCD surface scene: {path}', flush=True)


if __name__ == '__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', choices=MAPS)
    args=parser.parse_args()
    directory=Path(__file__).resolve().parents[2]/'assets/super'
    print(build_scene(scene_pcd(directory, args.scenario), directory/f'{args.scenario}.usdc'))
