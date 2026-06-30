from dataclasses import dataclass, replace
from pathlib import Path


SIMULATION_TB3 = 'simulation_tb3'
BAG_TB4 = 'bag_tb4'
REAL_TB4 = 'real_tb4'
SUPPORTED_TB4_NAMESPACES = ('tb4_0', 'tb4_1')


@dataclass(frozen=True)
class Tb4Topics:
    robot_namespace: str
    odom_topic: str
    scan_topic: str
    tf_topic: str
    tf_static_topic: str
    cmd_vel_topic: str
    rgb_topic: str
    depth_topic: str
    camera_info_topic: str


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
    tf_topic: str = ''
    tf_static_topic: str = ''


def normalize_robot_namespace(robot_namespace: str | None) -> str:
    value = (robot_namespace or 'tb4_0').strip()
    if value.startswith('/'):
        value = value[1:]
    if value not in SUPPORTED_TB4_NAMESPACES:
        supported = ', '.join(SUPPORTED_TB4_NAMESPACES)
        raise ValueError(
            f'robot_namespace desconocido "{robot_namespace}". Soportados: {supported}'
        )
    return f'/{value}'


def tb4_topics(robot_namespace: str | None = 'tb4_0') -> Tb4Topics:
    ns = normalize_robot_namespace(robot_namespace)
    return Tb4Topics(
        robot_namespace=ns,
        odom_topic=f'{ns}/odom',
        scan_topic=f'{ns}/scan',
        tf_topic=f'{ns}/tf',
        tf_static_topic=f'{ns}/tf_static',
        cmd_vel_topic=f'{ns}/cmd_vel',
        rgb_topic=f'{ns}/oakd/rgb/preview/image_raw',
        depth_topic=f'{ns}/oakd/stereo/image_raw',
        camera_info_topic=f'{ns}/oakd/rgb/preview/camera_info',
    )


def validate_topic_namespace(topic: str, robot_namespace: str | None, field_name: str):
    if not topic:
        return
    ns = normalize_robot_namespace(robot_namespace)
    other_namespaces = [
        f'/{name}' for name in SUPPORTED_TB4_NAMESPACES if f'/{name}' != ns
    ]
    for other_ns in other_namespaces:
        if topic == other_ns or topic.startswith(f'{other_ns}/'):
            raise ValueError(
                f'{field_name}="{topic}" apunta a {other_ns}, '
                f'pero robot_namespace={ns}'
            )


def validate_tb4_topics(robot_namespace: str | None, **topics):
    for field_name, topic in topics.items():
        validate_topic_namespace(str(topic), robot_namespace, field_name)


_TB4_DEFAULT = tb4_topics('tb4_0')

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
        tf_topic='/tf',
        tf_static_topic='/tf_static',
    ),
    BAG_TB4: PlatformProfile(
        profile=BAG_TB4,
        robot_namespace=_TB4_DEFAULT.robot_namespace,
        map_yaml='map_sim.yaml',
        odom_topic=_TB4_DEFAULT.odom_topic,
        reference_odom_topic=_TB4_DEFAULT.odom_topic,
        scan_topic=_TB4_DEFAULT.scan_topic,
        cmd_vel_topic=_TB4_DEFAULT.cmd_vel_topic,
        rgb_topic=_TB4_DEFAULT.rgb_topic,
        depth_topic=_TB4_DEFAULT.depth_topic,
        camera_info_topic=_TB4_DEFAULT.camera_info_topic,
        base_frame='base_link',
        odom_frame='odom',
        camera_frame='',
        use_sim_time=True,
        landmark_source='aruco',
        launch_virtual_landmarks=False,
        tf_topic=_TB4_DEFAULT.tf_topic,
        tf_static_topic=_TB4_DEFAULT.tf_static_topic,
    ),
    REAL_TB4: PlatformProfile(
        profile=REAL_TB4,
        robot_namespace=_TB4_DEFAULT.robot_namespace,
        map_yaml='map_sim.yaml',
        odom_topic=_TB4_DEFAULT.odom_topic,
        reference_odom_topic=_TB4_DEFAULT.odom_topic,
        scan_topic=_TB4_DEFAULT.scan_topic,
        cmd_vel_topic=_TB4_DEFAULT.cmd_vel_topic,
        rgb_topic=_TB4_DEFAULT.rgb_topic,
        depth_topic=_TB4_DEFAULT.depth_topic,
        camera_info_topic=_TB4_DEFAULT.camera_info_topic,
        base_frame='base_link',
        odom_frame='odom',
        camera_frame='',
        use_sim_time=False,
        landmark_source='aruco',
        launch_virtual_landmarks=False,
        tf_topic=_TB4_DEFAULT.tf_topic,
        tf_static_topic=_TB4_DEFAULT.tf_static_topic,
    ),
}


def supported_profiles():
    return tuple(_PROFILES.keys())


def resolve_profile(
    name: str,
    robot_namespace: str | None = None,
) -> PlatformProfile:
    try:
        profile = _PROFILES[name]
    except KeyError as exc:
        supported = ', '.join(supported_profiles())
        raise ValueError(
            f'Perfil de plataforma desconocido "{name}". Soportados: {supported}'
        ) from exc

    if name == SIMULATION_TB3:
        return profile

    topics = tb4_topics(robot_namespace or profile.robot_namespace)
    return replace(
        profile,
        robot_namespace=topics.robot_namespace,
        odom_topic=topics.odom_topic,
        reference_odom_topic=topics.odom_topic,
        scan_topic=topics.scan_topic,
        cmd_vel_topic=topics.cmd_vel_topic,
        rgb_topic=topics.rgb_topic,
        depth_topic=topics.depth_topic,
        camera_info_topic=topics.camera_info_topic,
        tf_topic=topics.tf_topic,
        tf_static_topic=topics.tf_static_topic,
    )


def _yaml_scalar(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if value is None:
        return ''
    return str(value)


def _write_yaml_mapping(lines, name, values):
    lines.append(f'{name}:')
    for key in sorted(values):
        lines.append(f'  {key}: {_yaml_scalar(values[key])}')


def write_resolved_platform(
    artifact_dir,
    profile: PlatformProfile,
    topics=None,
    frames=None,
    artifacts=None,
):
    root = Path(artifact_dir or '/tmp/tp_platform')
    output = root / 'config' / 'platform-resolved.yaml'
    output.parent.mkdir(parents=True, exist_ok=True)

    topic_values = {
        'odom_topic': profile.odom_topic,
        'reference_odom_topic': profile.reference_odom_topic,
        'scan_topic': profile.scan_topic,
        'cmd_vel_topic': profile.cmd_vel_topic,
        'rgb_topic': profile.rgb_topic,
        'depth_topic': profile.depth_topic,
        'camera_info_topic': profile.camera_info_topic,
        'tf_topic': profile.tf_topic,
        'tf_static_topic': profile.tf_static_topic,
    }
    if topics:
        topic_values.update(topics)

    frame_values = {
        'base_frame': profile.base_frame,
        'odom_frame': profile.odom_frame,
        'camera_frame': profile.camera_frame,
    }
    if frames:
        frame_values.update(frames)

    artifact_values = artifacts or {}

    lines = [
        f'profile: {profile.profile}',
        f'robot_namespace: {profile.robot_namespace}',
        f'use_sim_time: {_yaml_scalar(profile.use_sim_time)}',
        f'landmark_source: {profile.landmark_source}',
        f'launch_virtual_landmarks: {_yaml_scalar(profile.launch_virtual_landmarks)}',
    ]
    _write_yaml_mapping(lines, 'topics', topic_values)
    _write_yaml_mapping(lines, 'frames', frame_values)
    _write_yaml_mapping(lines, 'artifacts', artifact_values)
    output.write_text('\n'.join(lines) + '\n')
    return output
