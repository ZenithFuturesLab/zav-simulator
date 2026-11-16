"""Positive/negative PhysX contact fixtures in a standalone GUI, not a flight benchmark."""
import json
import sys
from pathlib import Path

output = Path(sys.argv[1]).resolve()
output.mkdir(parents=True, exist_ok=False)
from isaacsim import SimulationApp
app = SimulationApp({"headless": False})
from isaacsim.core.api import World
from pxr import Gf, UsdGeom, UsdPhysics
import carb.settings

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/super"))
from contact_diagnostics import ContactDiagnostics

world = World(physics_dt=.005, rendering_dt=.05, stage_units_in_meters=1.)
settings = carb.settings.get_settings()
settings.set("/app/runLoops/main/rateLimitEnabled", True)
settings.set("/app/runLoops/main/rateLimitFrequency", 20)


def cube(path, size, position, dynamic=False):
    geom = UsdGeom.Cube.Define(world.stage, path)
    geom.CreateSizeAttr(1.)
    geom.AddTranslateOp().Set(Gf.Vec3d(*position))
    geom.AddScaleOp().Set(Gf.Vec3f(*size))
    prim = geom.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    if dynamic:
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(1.)


cube("/World/layout/floor", (10., 10., .1), (0., 0., -.05))
cube("/World/layout/surface_0", (.5, .5, .5), (0., 0., .25))
UsdGeom.Xform.Define(world.stage, "/World/quadrotor")
cube("/World/quadrotor/probe_obstacle", (.2, .2, .2), (0., 0., 1.2), True)
cube("/World/quadrotor/probe_floor", (.2, .2, .2), (2., 0., 1.2), True)
diag = ContactDiagnostics(world.stage, "/World/quadrotor", lambda: world.current_time, output / "contacts.jsonl")
try:
    world.reset()
    world.play()
    while app.is_running() and world.current_time < 3.:
        world.step(render=True)
        diag.flush()
    result = diag.summary()
    assert result["counts"].get("floor_found", 0) > 0, result
    assert result["counts"].get("obstacle_found", 0) > 0, result
    assert result["callback_errors"] == result["dropped_events"] == 0, result
    result["passed"] = True
    result["purpose"] = "GUI controlled contacts, not benchmark performance"
    (output / "verification.json").write_text(json.dumps(result, indent=2))
    print("CONTACT_FIXTURE", json.dumps(result), flush=True)
finally:
    diag.close()
    world.stop()
    app.close()
