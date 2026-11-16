"""Export the existing USD's visible ZAV CAD to a ROS/RViz binary STL."""
from pathlib import Path
import struct
import numpy as np
from pxr import Usd, UsdGeom


def export(source, output):
    stage=Usd.Stage.Open(str(source));cache=UsdGeom.XformCache()
    triangles=[]
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh): continue
        mesh=UsdGeom.Mesh(prim)
        if mesh.ComputeVisibility()=='invisible': continue
        local=np.asarray(mesh.GetPointsAttr().Get(),dtype=np.float64)
        transform=np.asarray(cache.GetLocalToWorldTransform(prim))
        points=(np.column_stack((local,np.ones(len(local))))@transform)[:,:3]
        indices=np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
        offset=0
        for count in mesh.GetFaceVertexCountsAttr().Get():
            face=indices[offset:offset+count];offset+=count
            for i in range(1,count-1): triangles.append(points[face[[0,i,i+1]]])
    vertices=np.asarray(triangles,dtype=np.float32)
    normals=np.cross(vertices[:,1]-vertices[:,0],vertices[:,2]-vertices[:,0])
    length=np.linalg.norm(normals,axis=1);normals/=np.maximum(length,1e-12)[:,None]
    dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
    records=np.zeros(len(vertices),dtype=dtype);records['normal']=normals;records['vertices']=vertices
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    output.write_bytes(b'ZAV CAD from workspace USD'.ljust(80,b'\0')+struct.pack('<I',len(vertices))+records.tobytes())
    print(f'{len(vertices)} triangles; {output}; radial extent {np.linalg.norm(vertices[:,:,:2],axis=2).max():.4f} m')

if __name__=='__main__':
    ws=Path(__file__).resolve().parents[3]
    export(ws/'simulator/pegasus/simulator/assets/Robots/ZAV_v1/ZAV_v1.usd',ws/'src/zav_super/meshes/zav.stl')
