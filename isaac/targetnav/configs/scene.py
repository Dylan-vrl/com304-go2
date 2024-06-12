from omni.isaac.lab.assets import AssetBaseCfg, RigidObjectCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sensors import ContactSensorCfg, CameraCfg
from omni.isaac.lab.sim import GroundPlaneCfg, UsdFileCfg
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.utils import configclass


@configclass
class MySceneCfg(InteractiveSceneCfg):
    """Configuration for the terrain scene with a legged robot."""

    ground = AssetBaseCfg(prim_path="/World/ground", spawn=GroundPlaneCfg())

    room = AssetBaseCfg(prim_path="{ENV_REGEX_NS}/room",
                        spawn=UsdFileCfg(usd_path="omniverse://localhost/Library/assets/simple_room/simple_room.usd",
                                         collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True)),
                        init_state=AssetBaseCfg.InitialStateCfg(pos=(0, 0, 0.8)))

    # add cube
    cube: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/robot",
        spawn=sim_utils.CuboidCfg(
            size=(0.70, 0.31, 0.40),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=1.0,
                                                         max_linear_velocity=3,
                                                         max_angular_velocity=80,
                                                         disable_gravity=False),
            mass_props=sim_utils.MassPropertiesCfg(mass=3.0),
            physics_material=sim_utils.RigidBodyMaterialCfg(),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.5, 0.0)),
            activate_contact_sensors=True
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 0.605), lin_vel=(0, 0, 0)),
    )

    ball = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/ball",
                          spawn=sim_utils.SphereCfg(radius=0.1,
                                                    rigid_props=sim_utils.RigidBodyPropertiesCfg(),
                                                    mass_props=sim_utils.MassPropertiesCfg(mass=10),
                                                    collision_props=sim_utils.CollisionPropertiesCfg(),
                                                    visual_material=sim_utils.PreviewSurfaceCfg(
                                                        diffuse_color=(1.0, 0.0, 0.0), metallic=0),
                                                    ),
                          init_state=RigidObjectCfg.InitialStateCfg(pos=(1, 1, 0.515)))

    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/robot", update_period=0.0, history_length=6, debug_vis=False
    )

    # #sensors
    # camera = CameraCfg(
    #     prim_path="{ENV_REGEX_NS}/robot/front_cam",
    #     update_period=0.1,
    #     height=480,
    #     width=640,
    #     data_types=["rgb", "distance_to_image_plane"],
    #     spawn=sim_utils.PinholeCameraCfg(
    #         focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)
    #     ),
    #     offset=CameraCfg.OffsetCfg(pos=(0.4, 0.0, 0.23), rot=(0.5, -0.5, 0.5, -0.5), convention="ros"),
    # )

    # sensors
    camera = CameraCfg(
        prim_path="{ENV_REGEX_NS}/robot/front_cam",
        update_period=0.1,
        height=128,
        width=128,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)
        ),
        offset=CameraCfg.OffsetCfg(pos=(0.4, 0.0, 0.23), rot=(0.5, -0.5, 0.5, -0.5), convention="ros"),
    )

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(intensity=1000.0)
    )