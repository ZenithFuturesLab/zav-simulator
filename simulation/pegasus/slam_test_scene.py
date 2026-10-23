# [ZAV 변경] 평면에서 관측 불가능한 수평 이동을 검증하기 위한 선택적 테스트 장면입니다.
"""Small deterministic scene with geometry and visual features for SLAM checks."""
import random
from pxr import Gf, UsdGeom, UsdPhysics, Vt


def add_slam_test_scene(stage):
    rng = random.Random(42)

    def panel(name, origin, u, v, nu, nv):
        points, colors = [], []
        for j in range(nv):
            for i in range(nu):
                shade = rng.choice((0.07, 0.2, 0.5, 0.8, 0.95))
                tint = (shade, shade * rng.uniform(.7, 1), shade * rng.uniform(.7, 1))
                for a, b in ((i,j), (i+1,j), (i+1,j+1), (i,j+1)):
                    points.append(Gf.Vec3f(*(origin[k] + u[k]*a/nu + v[k]*b/nv for k in range(3))))
                    colors.append(Gf.Vec3f(*tint))
        mesh = UsdGeom.Mesh.Define(stage, '/World/SLAMTest/' + name)
        mesh.CreatePointsAttr(Vt.Vec3fArray(points))
        mesh.CreateFaceVertexCountsAttr([4] * (nu*nv))
        mesh.CreateFaceVertexIndicesAttr(list(range(len(points))))
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.faceVarying).Set(Vt.Vec3fArray(colors))
        if name != 'FloorPattern':
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())

    panel('FloorPattern', (-10,-10,.002), (20,0,0), (0,20,0), 50,50)
    panel('FrontWall', (-7,8,0), (14,0,0), (0,0,5), 35,12)
    panel('LeftWall', (-7,-7,0), (0,15,0), (0,0,5), 37,12)
    panel('RightWall', (7,-7,0), (0,15,0), (0,0,5), 37,12)
    panel('BackWall', (-7,-7,0), (14,0,0), (0,0,5), 35,12)
    # Asymmetric pillars remove the repeated-wall ambiguity; clear central flight area.
    for index, (x,y,height,width) in enumerate(((-3,3,2.5,.6),(3,4,3.2,.8),(-4,-2,1.8,1.),(4,-3,2.8,.5))):
        cube = UsdGeom.Cube.Define(stage, f'/World/SLAMTest/Pillar{index}')
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(x,y,height/2))
        cube.AddScaleOp().Set(Gf.Vec3f(width,width,height))
        cube.CreateDisplayColorAttr([Gf.Vec3f(rng.random(),rng.random(),rng.random())])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    print('[SLAM scene] textured floor, four walls and four pillars; central area clear', flush=True)
