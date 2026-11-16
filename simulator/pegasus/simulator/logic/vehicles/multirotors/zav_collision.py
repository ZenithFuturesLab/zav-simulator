"""Compact collision geometry for the ZAV CAD asset (authored at spawn time)."""

from pxr import Sdf, Usd, UsdGeom, UsdPhysics


def configure_zav_collision(stage, vehicle_path):
    """Use structural CAD parts instead of the asset's bottom-only cylinder.

    Keep the authored mass/inertia and rotor joints. Small electronic components
    and rotating propeller visuals do not need separate collision shapes.
    """
    body = stage.GetPrimAtPath(f"{vehicle_path}/body")
    visuals = stage.GetPrimAtPath(f"{vehicle_path}/body/visual_mesh")
    if not body or not visuals:
        raise RuntimeError(f"ZAV collision geometry missing at {vehicle_path}")

    # Frame, guard plates/posts, landing base, camera, LiDAR/housing and motors.
    parts = {f"MeshInstance_{i}" for i in (
        *range(1, 24), 26, 42, 151, 173, 174, 175, 176, 177, 178,
        585, 586, 587, 588,
    )} | {"body"}
    meshes = [p for p in Usd.PrimRange(visuals)
              if p.IsA(UsdGeom.Mesh) and p.GetParent().GetName() in parts]
    if len(meshes) != len(parts):
        raise RuntimeError("ZAV CAD structure changed; review collision part selection")

    for prim in Usd.PrimRange(body):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)

    for prim in meshes:
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
        # Decompose the wide guard/frame plates so their empty spaces remain
        # open; convex hulls suffice for the small, solid components.
        name = prim.GetParent().GetName()
        approximation = "convexDecomposition" if name in {
            "MeshInstance_1", "MeshInstance_2", "MeshInstance_8",
            "MeshInstance_9", "MeshInstance_10", "body",
        } else "convexHull"
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approximation)
        # Two millimetres of contact margin, rather than PhysX's centimetre
        # scale default which is large relative to these thin CAD parts.
        schemas = prim.GetAppliedSchemas()
        if "PhysxCollisionAPI" not in schemas:
            prim.AddAppliedSchema("PhysxCollisionAPI")
        prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(0.002)
        prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)

    return len(meshes)
