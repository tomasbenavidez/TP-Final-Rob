from dataclasses import dataclass


SIMULATION_TB3 = 'simulation_tb3'
BAG_TB4 = 'bag_tb4'
REAL_TB4 = 'real_tb4'


@dataclass(frozen=True)
class PlatformProfile:
    profile: str
    robot_namespace: str
    map_yaml: str
    odom_topic: str
    reference_odom_topic: str
    scan_topic: str
    cmd_vel_topic: str
    rgb_topic: str
    depth_topic: str
    camera_info_topic: str
    base_frame: str
    odom_frame: str
    camera_frame: str
    use_sim_time: bool
    landmark_source: str
    launch_virtual_landmarks: bool


_PROFILES = {
    SIMULATION_TB3: PlatformProfile(
        profile=SIMULATION_TB3,
        robot_namespace='',
        map_yaml='map_sim.yaml',
        odom_topic='/calc_odom',
        reference_odom_topic='/odom',
        scan_topic='/scan',
        cmd_vel_topic='/cmd_vel',
        rgb_topic='/camera/rgb/image_raw',
        depth_topic='/camera/depth/image_raw',
        camera_info_topic='/camera/rgb/camera_info',
        base_frame='base_footprint',
        odom_frame='odom',
        camera_frame='',
        use_sim_time=True,
        landmark_source='virtual',
        launch_virtual_landmarks=True,
    ),
    BAG_TB4: PlatformProfile(
        profile=BAG_TB4,
        robot_namespace='/tb4_0',
        map_yaml='map_sim.yaml',
        odom_topic='/tb4_0/odom',
        reference_odom_topic='/tb4_0/odom',
        scan_topic='/tb4_0/scan',
        cmd_vel_topic='/tb4_0/cmd_vel',
        rgb_topic='/tb4_0/oakd/rgb/preview/image_raw',
        depth_topic='/tb4_0/oakd/stereo/image_raw',
        camera_info_topic='/tb4_0/oakd/rgb/preview/camera_info',
        base_frame='base_link',
        odom_frame='odom',
        camera_frame='',
        use_sim_time=True,
        landmark_source='aruco',
        launch_virtual_landmarks=False,
    ),
    REAL_TB4: PlatformProfile(
        profile=REAL_TB4,
        robot_namespace='/tb4_0',
        map_yaml='map_sim.yaml',
        odom_topic='/tb4_0/odom',
        reference_odom_topic='/tb4_0/odom',
        scan_topic='/tb4_0/scan',
        cmd_vel_topic='/tb4_0/cmd_vel',
        rgb_topic='/tb4_0/oakd/rgb/preview/image_raw',
        depth_topic='/tb4_0/oakd/stereo/image_raw',
        camera_info_topic='/tb4_0/oakd/rgb/preview/camera_info',
        base_frame='base_link',
        odom_frame='odom',
        camera_frame='',
        use_sim_time=False,
        landmark_source='aruco',
        launch_virtual_landmarks=False,
    ),
}


def supported_profiles():
    return tuple(_PROFILES.keys())


def resolve_profile(name: str) -> PlatformProfile:
    try:
        return _PROFILES[name]
    except KeyError as exc:
        supported = ', '.join(supported_profiles())
        raise ValueError(
            f'Perfil de plataforma desconocido "{name}". Soportados: {supported}'
        ) from exc
