"""Generate static, metre/Z-up collision scenes for EPIC integration trials.

The geometry is a simulator fixture, never an input to the exploration planner.
"""
from pathlib import Path
import argparse

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output-dir',type=Path,required=True)
scenes=['loop','dead_end_rooms','multiple_doors','high_shelves','split_level_ramp','enclosed_ramp']
parser.add_argument('scenes',nargs='*',metavar='SCENE')
args=parser.parse_args()
if set(args.scenes)-set(scenes):
 parser.error('unknown scene; choose from: '+', '.join(scenes))
root=args.output_dir.resolve()
root.mkdir(parents=True,exist_ok=True)
for name in (args.scenes or scenes):
 parts=['#usda 1.0\n(defaultPrim = "Environment"\n metersPerUnit = 1\n upAxis = "Z")\ndef Xform "Environment" {']
 count=0
 def box(center,size):
  global count
  count+=1
  parts.append(f'''def Cube "box{count}" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {{
 double size = 1
 double3 xformOp:translate = {tuple(center)}
 float3 xformOp:scale = {tuple(size)}
 uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
 color3f[] primvars:displayColor = [(0.6, 0.65, 0.7)]
 }}''')
 ceiling=10 if name=='high_shelves' else 7 if name in ('split_level_ramp','enclosed_ramp') else 5
 box((-1,2,-.1),(28,28,.2));box((-1,2,ceiling+.1),(28,28,.2))
 for x in [-15,13]:box((x,2,ceiling/2),(.2,28,ceiling))
 for y in [-12,16]:box((-1,y,ceiling/2),(28,.2,ceiling))
 if name=='loop':box((3,6,2.5),(8,8,5))
 if name=='dead_end_rooms':
  for y in [-6,4,11]:box((5,y,2.5),(16,.2,5))
 if name=='multiple_doors':
  for y in [-6,5]:
   for x,w in [(-12,6),(0,10),(10,6)]:box((x,y,2.5),(w,.2,5))
 if name=='high_shelves':
  for x in [-10,1,8]:
   for y in [-5,6]:box((x,y,3),(1.4,12,6))
 if name in ('split_level_ramp','enclosed_ramp'):
  parts.append('''def Mesh "ramp" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {
 point3f[] points = [(0,-8,0),(10,-8,0),(10,-4,0),(0,-4,0),(0,-8,0.2),(10,-8,3.8),(10,-4,3.8),(0,-4,0.2)]
 int[] faceVertexCounts = [4,4,4,4,4,4]
 int[] faceVertexIndices = [0,3,2,1,4,5,6,7,0,1,5,4,1,2,6,5,2,3,7,6,3,0,4,7]
 uniform token subdivisionScheme = "none"
 }''')
  box((11,4,3.7),(4,20,.2))
  if name=='enclosed_ramp':
   # The upper room has one entrance, through the sloped passage. Walls
   # stop flight around the outside from replacing the actual height change.
   box((6.5,-8.1,3.5),(13,.2,7))
   box((4.5,-3.9,3.5),(9,.2,7))
   box((8.9,5.05,5.35),(.2,18.1,3.3))
   box((11,14.1,5.35),(4,.2,3.3))
 parts.append('}')
 (root/(name+'.usda')).write_text('\n'.join(parts)+'\n')
 print(name,count)
