"""
aruco_utils.py
==============
Funciones auxiliares para ArUco que NO dependen de ROS: carga de la
calibración de la cámara y estimación de pose de los marcadores.

Separarlas del nodo nos deja testear la detección con imágenes sueltas
(p. ej. las del bag aruco_estimation.zip) sin levantar ROS.
"""

import numpy as np
import cv2
import yaml


def load_camera_calibration(path: str):
    """
    Carga la matriz intrínseca K (3x3) y los coeficientes de distorsión
    desde un archivo YAML.

    La cátedra provee K y los coeficientes del TB4 #0. Soportamos dos
    formatos comunes de YAML de calibración:
      1) formato OpenCV  (camera_matrix / distortion_coefficients con 'data')
      2) formato simple  (camera_matrix: [[...]], dist_coeffs: [...])

    Retorna (camera_matrix: np.ndarray 3x3, dist_coeffs: np.ndarray).
    """
    with open(path, 'r') as f:
        data = yaml.safe_load(f)

    # Extraemos también el tamaño del marcador si está presente
    marker_size = None

    # Formato OpenCV
    if 'camera_matrix' in data and isinstance(data['camera_matrix'], dict):
        k = data['camera_matrix']
        camera_matrix = np.array(k['data'], dtype=np.float64).reshape(
            k.get('rows', 3), k.get('cols', 3)
        )
        d = data['distortion_coefficients']
        dist_coeffs = np.array(d['data'], dtype=np.float64).reshape(1, -1)
        # marker_size puede estar en la raíz del YAML
        marker_size = data.get('marker_size_m', None)
        return camera_matrix, dist_coeffs, marker_size

    # Formato simple
    camera_matrix = np.array(data['camera_matrix'], dtype=np.float64).reshape(3, 3)
    dist_key = 'dist_coeffs' if 'dist_coeffs' in data else 'distortion_coefficients'
    dist_coeffs = np.array(data[dist_key], dtype=np.float64).reshape(1, -1)
    marker_size = data.get('marker_size_m', None)
    return camera_matrix, dist_coeffs, marker_size


def estimate_marker_poses(corners, ids, marker_length,
                          camera_matrix, dist_coeffs):
    """
    Estima la pose 3D (rvec, tvec) de cada marcador detectado.

    Usamos solvePnP sobre los 4 puntos del marcador en su sistema propio.
    Esto reemplaza a la antigua estimatePoseSingleMarkers (deprecada en
    OpenCV reciente) y es más explícito sobre lo que pasa por dentro.

    Parámetros
    ----------
    corners : lista de arrays con las 4 esquinas (px) de cada marcador
    ids     : array de IDs detectados
    marker_length : lado del marcador en metros (escala el resultado a metros)
    camera_matrix, dist_coeffs : calibración

    Retorna
    -------
    lista de dicts: {'id', 'rvec', 'tvec'}
        rvec : vector de rotación (Rodrigues) marcador->cámara
        tvec : traslación (x, y, z) del marcador en el frame de la cámara [m]
    """
    # Puntos 3D del marcador en SU PROPIO sistema de coordenadas,
    # centrado en el medio del marcador, plano Z=0. El orden coincide
    # con el que devuelve detectMarkers (sentido horario desde arriba-izq).
    half = marker_length / 2.0
    object_points = np.array([
        [-half,  half, 0.0],
        [ half,  half, 0.0],
        [ half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float64)

    detections = []
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        image_points = marker_corners.reshape(4, 2).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            object_points, image_points,
            camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,  # óptimo para marcadores planos cuadrados
        )
        if ok:
            detections.append({
                'id': int(marker_id),
                'rvec': rvec.flatten(),
                'tvec': tvec.flatten(),
                'image_points': image_points,
            })
    return detections


def camera_to_robot_observation(tvec):
    """
    Convierte la posición de un marcador en el frame de la CÁMARA a una
    observación en el plano del ROBOT (rango, ángulo de orientación).

    Convención del frame óptico de la cámara (OpenCV/ROS):
        x -> derecha,  y -> abajo,  z -> hacia adelante (profundidad)

    Para SLAM 2D nos quedamos con el plano horizontal: la profundidad (z)
    es el "adelante" del robot y x es el "lateral". El rango y el bearing
    (ángulo) son lo que el Graph SLAM usa como observación del landmark.

    Retorna (range, bearing) con bearing en rad (positivo a la izquierda).

    NOTA: esto asume cámara mirando al frente y alineada con el robot.
    Si hay un offset/rotación cámara->base_link, hay que componer esa
    transformación acá (lo afinamos con el extrínseco real del TB4).
    """
    x_cam, _y_cam, z_cam = tvec
    range_ = float(np.hypot(x_cam, z_cam))
    bearing = float(np.arctan2(-x_cam, z_cam))  # -x: izquierda positiva
    return range_, bearing
