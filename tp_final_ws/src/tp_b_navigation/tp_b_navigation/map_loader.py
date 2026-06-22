#!/usr/bin/env python3
"""map_loader: carga un mapa de ocupación (formato map_server: .yaml + .pgm) y lo
publica como nav_msgs/OccupancyGrid en /map de forma latcheada (transient_local),
para que cualquier nodo que se suscriba después (RViz, planner, MCL) lo reciba.

Replica la semántica estándar de nav2_map_server (modo 'trinary') pero sin depender
del stack de lifecycle de nav2, manteniendo el paquete autocontenido.
"""

import os

import numpy as np
import yaml

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion


def _read_pgm(path):
    """Lee un PGM (P5 binario o P2 ASCII) y devuelve (ancho, alto, array uint8 [alto, ancho]).

    El array se devuelve en el mismo orden que la imagen: fila 0 = arriba.
    """
    with open(path, 'rb') as f:
        data = f.read()

    # Tokenizador simple que ignora comentarios (#...) del header.
    tokens = []
    idx = 0
    n = len(data)
    while len(tokens) < 4 and idx < n:
        # saltar espacios en blanco
        while idx < n and data[idx:idx + 1].isspace():
            idx += 1
        # saltar comentarios
        if idx < n and data[idx:idx + 1] == b'#':
            while idx < n and data[idx:idx + 1] not in (b'\n', b'\r'):
                idx += 1
            continue
        start = idx
        while idx < n and not data[idx:idx + 1].isspace():
            idx += 1
        tokens.append(data[start:idx])

    magic = tokens[0]
    width = int(tokens[1])
    height = int(tokens[2])
    maxval = int(tokens[3])
    # Tras el maxval hay exactamente un carácter de whitespace antes de los datos.
    idx += 1

    if magic == b'P5':
        pixels = np.frombuffer(data, dtype=np.uint8, count=width * height, offset=idx)
        img = pixels.reshape((height, width)).copy()
    elif magic == b'P2':
        vals = np.array([int(t) for t in data[idx:].split()], dtype=np.int32)
        img = vals[:width * height].reshape((height, width)).astype(np.uint8)
    else:
        raise ValueError(f'Formato PGM no soportado: {magic}')

    if maxval != 255:
        img = (img.astype(np.float32) * 255.0 / maxval).astype(np.uint8)
    return width, height, img


def _yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = float(np.sin(yaw * 0.5))
    q.w = float(np.cos(yaw * 0.5))
    return q


class MapLoader(Node):
    def __init__(self):
        super().__init__('map_loader')

        # Recurso instalado con el paquete; funciona sin conocer la ruta del checkout.
        package_share = get_package_share_directory('tp_b_navigation')
        default_yaml = os.path.join(package_share, 'maps', 'map_sim.yaml')
        self.declare_parameter('map_yaml', default_yaml)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_period_sec', 2.0)

        self.frame_id = self.get_parameter('frame_id').value
        yaml_path = self.get_parameter('map_yaml').value

        self.grid = self._build_grid(yaml_path)

        # QoS latcheado: el mapa es estático, lo entregamos a suscriptores tardíos.
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(OccupancyGrid, '/map', qos)
        self.pub.publish(self.grid)
        self.get_logger().info(
            f'Mapa publicado en /map: {self.grid.info.width}x{self.grid.info.height} '
            f'@ {self.grid.info.resolution} m/celda (frame {self.frame_id}).')

        # Re-publicación periódica por robustez (RViz reiniciado, etc.).
        period = float(self.get_parameter('publish_period_sec').value)
        if period > 0.0:
            self.create_timer(period, lambda: self.pub.publish(self.grid))

    def _build_grid(self, yaml_path):
        with open(yaml_path, 'r') as f:
            meta = yaml.safe_load(f)

        resolution = float(meta['resolution'])
        origin = meta['origin']
        negate = int(meta.get('negate', 0))
        occ_th = float(meta.get('occupied_thresh', 0.65))
        free_th = float(meta.get('free_thresh', 0.25))

        # La imagen puede estar referida relativa al .yaml.
        image_rel = meta['image']
        image_path = image_rel
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_rel)

        width, height, img = _read_pgm(image_path)

        # Conversión a probabilidad de ocupación (semántica map_server).
        # p = pixel/255 ; si negate=0 -> occ = 1-p (oscuro = ocupado), si negate=1 -> occ = p
        p = img.astype(np.float32) / 255.0
        occ = p if negate else (1.0 - p)

        # OccupancyGrid: fila 0 = origen (abajo-izquierda) -> volteamos verticalmente.
        occ = np.flipud(occ)

        grid_vals = np.full(occ.shape, -1, dtype=np.int8)       # desconocido por defecto
        grid_vals[occ > occ_th] = 100                            # ocupado
        grid_vals[occ < free_th] = 0                             # libre

        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = float(origin[0])
        msg.info.origin.position.y = float(origin[1])
        msg.info.origin.position.z = 0.0
        yaw = float(origin[2]) if len(origin) > 2 else 0.0
        msg.info.origin.orientation = _yaw_to_quaternion(yaw)
        msg.data = grid_vals.flatten(order='C').tolist()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = MapLoader()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
