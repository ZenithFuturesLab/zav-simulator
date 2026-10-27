"""Check a movable source tree without an installed Pegasus checkout."""
import ast
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import types

root = Path(__file__).resolve().parents[1]
for name in ('run_isaac_sim.sh', 'run_zav_sim.sh'):
    assert 'PEGASUS_DIR' not in (root/'scripts'/name).read_text()
assert 'deploy_sim.py' not in (root.parent/'scripts/configure.py').read_text()

with tempfile.TemporaryDirectory(prefix="ZAV checkout with spaces ") as directory:
    checkout = Path(directory) / "checkout"
    sim = checkout / "simulator"
    shutil.copytree(root, sim, ignore=shutil.ignore_patterns("__pycache__"))
    assert not (checkout / "src/PegasusSimulator").exists()
    # Evaluate the real params with only NVIDIA's storage query stubbed.
    storage = types.SimpleNamespace(get_assets_root_path=lambda: "unused")
    for name in ("isaacsim", "isaacsim.storage"):
        module = types.ModuleType(name); module.__path__ = []; sys.modules[name] = module
    sys.modules["isaacsim.storage.native"] = storage
    params = {"__file__": str(sim / "pegasus/simulator/params.py")}
    exec(compile(Path(params["__file__"]).read_text(), params["__file__"], "exec"), params)
    assert Path(params["CONFIG_FILE"]).is_file()
    assert Path(params["ROBOTS"]["ZAV_v1"]).is_file()
    assert (Path(params["ASSET_PATH"]) / "Sensors/Mid-360s.usda").is_file()
    app = ast.parse((sim / "zav_sim.py").read_text())
    values = {n.targets[0].id: n.value.value for n in app.body
              if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name)
              and isinstance(n.value, ast.Constant)}
    assert values["PHYSICS_HZ"] == 160 and values["RENDER_HZ"] == 20
    # Resolve upstream and customized Python modules using only the new source path.
    code = """
import importlib.machinery as m, pathlib, sys
base=pathlib.Path(sys.argv[1])
package=m.PathFinder.find_spec('pegasus',[str(base)])
assert pathlib.Path(package.origin).is_relative_to(base)
for relative in ('logic/backends/px4_mavlink_backend.py', 'logic/backends/ros2_backend.py',
                 'logic/sensors/mounted_imu.py', 'logic/vehicles/multirotors/zav_v1.py'):
    assert (base/'pegasus/simulator'/relative).is_file()
"""
    subprocess.run([sys.executable, "-c", code, str(sim)], check=True)
print("SOURCE_LAYOUT_OK: movable checkout, local Pegasus, assets, config, preserved rates; no deploy")
