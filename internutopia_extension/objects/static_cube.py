import numpy as np

from internutopia.core.object import BaseObject
from internutopia.core.scene.scene import IScene
from internutopia_extension.configs.objects import StaticCubeCfg


@BaseObject.register('StaticCube')
class StaticCube(BaseObject):
    def __init__(self, config: StaticCubeCfg, scene: IScene):
        super().__init__(config, scene)
        self._config = config

    def set_up_to_scene(self, scene: IScene):
        try:
            from omni.isaac.core.objects import FixedCuboid
        except ImportError:
            from omni.isaac.core.objects.cuboid import FixedCuboid

        static_cube = FixedCuboid(
            prim_path=self._config.prim_path,
            name=self._config.name,
            position=np.array(self._config.position),
            orientation=np.array(self._config.orientation),
            scale=np.array(self._config.scale),
            color=np.array(self._config.color),
            visible=self._config.visible is not False,
        )
        try:
            # Isaac's FixedCuboid constructor accidentally calls
            # set_collision_enabled("boundingCube"); set the approximation explicitly.
            static_cube.set_collision_approximation('boundingCube')
            static_cube.set_collision_enabled(True)
        except Exception:
            pass
        if (
            self._config.static_friction is not None
            or self._config.dynamic_friction is not None
            or self._config.restitution is not None
        ):
            try:
                from isaacsim.core.api.materials import PhysicsMaterial

                material_name = self._config.name.replace('/', '_')
                physics_material = PhysicsMaterial(
                    prim_path=f'/World/Physics_Materials/{material_name}_physics_material',
                    name=f'{material_name}_physics_material',
                    static_friction=self._config.static_friction,
                    dynamic_friction=self._config.dynamic_friction,
                    restitution=self._config.restitution,
                )
                static_cube.apply_physics_material(physics_material)
                combine_mode = getattr(self._config, 'friction_combine_mode', None)
                if combine_mode:
                    from pxr import PhysxSchema

                    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(physics_material.prim)
                    physx_material.CreateFrictionCombineModeAttr().Set(str(combine_mode))
            except Exception:
                pass
        scene.add(static_cube)
        self._lock_fixed_cuboid(static_cube)

    @staticmethod
    def _lock_fixed_cuboid(static_cube) -> None:
        """Isaac FixedCuboid still has RigidBodyAPI; keep it kinematic or the deck falls."""
        try:
            from pxr import PhysxSchema, UsdPhysics
        except Exception:
            return
        try:
            prim = static_cube.prim
        except Exception:
            return
        if prim is None or not prim.IsValid():
            return
        try:
            rigid_body_api = (
                UsdPhysics.RigidBodyAPI(prim)
                if prim.HasAPI(UsdPhysics.RigidBodyAPI)
                else UsdPhysics.RigidBodyAPI.Apply(prim)
            )
            kinematic_attr = rigid_body_api.GetKinematicEnabledAttr()
            if kinematic_attr is None or not kinematic_attr.IsValid():
                kinematic_attr = rigid_body_api.CreateKinematicEnabledAttr()
            kinematic_attr.Set(True)
            enabled_attr = rigid_body_api.GetRigidBodyEnabledAttr()
            if enabled_attr is not None and enabled_attr.IsValid():
                enabled_attr.Set(True)
        except Exception:
            pass
        try:
            physx_api = (
                PhysxSchema.PhysxRigidBodyAPI(prim)
                if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI)
                else PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            )
            gravity_attr = physx_api.GetDisableGravityAttr()
            if gravity_attr is None or not gravity_attr.IsValid():
                gravity_attr = physx_api.CreateDisableGravityAttr()
            gravity_attr.Set(True)
        except Exception:
            return
