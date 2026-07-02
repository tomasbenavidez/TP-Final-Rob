#!/usr/bin/env python3
"""Plot MCL correction diagnostics over a map and compare against Parte A.

Example:
  python3 docs/parte_b/scripts/plot_mcl_diagnostics.py \
    --diagnostics-csv /tmp/tp_mcl_laberinto_oos_v1.csv \
    --trajectory-json runs/laboratorio_tb4_1/parte_a/trajectory.json \
    --map-yaml runs/laboratorio_tb4_1/parte_a/map_res003_keep_thin.yaml \
    --output-prefix /tmp/mcl_laboratorio_oos
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml


PALETTE = {
    'laser': '#4C78A8',
    'landmark': '#F58518',
    'landmark_oos': '#B279A2',
    'landmark_oos_drop': '#E45756',
}


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    tokens = []
    idx = 0
    while len(tokens) < 4 and idx < len(data):
        while idx < len(data) and data[idx:idx + 1].isspace():
            idx += 1
        if idx < len(data) and data[idx:idx + 1] == b'#':
            while idx < len(data) and data[idx:idx + 1] not in (b'\n', b'\r'):
                idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx:idx + 1].isspace():
            idx += 1
        tokens.append(data[start:idx])
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    idx += 1
    if magic == b'P5':
        pixels = np.frombuffer(data, dtype=np.uint8, count=width * height, offset=idx)
        img = pixels.reshape((height, width)).copy()
    elif magic == b'P2':
        vals = np.array([int(t) for t in data[idx:].split()], dtype=np.int32)
        img = vals[:width * height].reshape((height, width)).astype(np.uint8)
    else:
        raise ValueError(f'Unsupported PGM format: {magic!r}')
    if maxval != 255:
        img = (img.astype(np.float32) * 255.0 / maxval).astype(np.uint8)
    return img


def load_map(map_yaml):
    with open(map_yaml, 'r') as f:
        meta = yaml.safe_load(f)
    image_path = Path(meta['image'])
    if not image_path.is_absolute():
        image_path = Path(map_yaml).parent / image_path
    img = read_pgm(image_path)
    resolution = float(meta['resolution'])
    origin = meta['origin']
    origin_x = float(origin[0])
    origin_y = float(origin[1])
    height, width = img.shape
    extent = [
        origin_x,
        origin_x + width * resolution,
        origin_y,
        origin_y + height * resolution,
    ]
    return img, extent


def load_trajectory(path):
    data = json.loads(Path(path).read_text())
    traj = sorted(data['trajectory'], key=lambda p: float(p['stamp']))
    stamps = np.array([float(p['stamp']) for p in traj], dtype=float)
    poses = np.array([
        [float(p['x']), float(p['y']), float(p['theta'])]
        for p in traj
    ], dtype=float)
    return stamps, poses


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def interp_pose(stamps, poses, stamp):
    if stamp < stamps[0] or stamp > stamps[-1]:
        return None
    idx = int(np.searchsorted(stamps, stamp))
    if idx <= 0:
        return poses[0].copy()
    if idx >= len(stamps):
        return poses[-1].copy()
    t0, t1 = stamps[idx - 1], stamps[idx]
    ratio = (stamp - t0) / (t1 - t0) if t1 > t0 else 0.0
    p0, p1 = poses[idx - 1], poses[idx]
    yaw = p0[2] + ratio * angle_diff(p1[2], p0[2])
    return np.array([
        p0[0] + ratio * (p1[0] - p0[0]),
        p0[1] + ratio * (p1[1] - p0[1]),
        math.atan2(math.sin(yaw), math.cos(yaw)),
    ])


def parse_float(row, key):
    value = row.get(key, '')
    if value == '':
        return None
    try:
        number = float(value)
    except ValueError:
        return None
    return number if math.isfinite(number) else None


def load_diagnostics(path, events):
    rows = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            if events and row.get('event') not in events:
                continue
            stamp = parse_float(row, 'stamp_sec')
            x = parse_float(row, 'estimate_x_after')
            y = parse_float(row, 'estimate_y_after')
            yaw = parse_float(row, 'estimate_yaw_after')
            delta_xy = parse_float(row, 'delta_xy')
            if None in (stamp, x, y, yaw, delta_xy):
                continue
            rows.append({
                'stamp_sec': stamp,
                'measurement_stamp_sec': parse_float(row, 'measurement_stamp_sec'),
                'event': row.get('event', ''),
                'x': x,
                'y': y,
                'yaw': yaw,
                'delta_xy': delta_xy,
                'delta_yaw': parse_float(row, 'delta_yaw') or 0.0,
                'detail': row.get('detail', ''),
            })
    return rows


def comparison_records(rows, stamps, poses):
    records = []
    for row in rows:
        ref = interp_pose(stamps, poses, row['stamp_sec'])
        if ref is None:
            continue
        error_xy = math.hypot(row['x'] - ref[0], row['y'] - ref[1])
        error_yaw = angle_diff(row['yaw'], ref[2])
        records.append({
            **row,
            'reference_x': float(ref[0]),
            'reference_y': float(ref[1]),
            'reference_yaw': float(ref[2]),
            'error_xy': error_xy,
            'error_yaw': error_yaw,
        })
    return records


def write_comparison(records, output_csv):
    with open(output_csv, 'w', newline='') as f:
        fieldnames = [
            'stamp_sec',
            'measurement_stamp_sec',
            'event',
            'estimate_x',
            'estimate_y',
            'estimate_yaw',
            'reference_x',
            'reference_y',
            'reference_yaw',
            'error_xy',
            'error_yaw',
            'delta_xy',
            'delta_yaw',
            'detail',
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in records:
            writer.writerow({
                'stamp_sec': f'{row["stamp_sec"]:.9f}',
                'measurement_stamp_sec': (
                    '' if row['measurement_stamp_sec'] is None
                    else f'{row["measurement_stamp_sec"]:.9f}'
                ),
                'event': row['event'],
                'estimate_x': f'{row["x"]:.9g}',
                'estimate_y': f'{row["y"]:.9g}',
                'estimate_yaw': f'{row["yaw"]:.9g}',
                'reference_x': f'{row["reference_x"]:.9g}',
                'reference_y': f'{row["reference_y"]:.9g}',
                'reference_yaw': f'{row["reference_yaw"]:.9g}',
                'error_xy': f'{row["error_xy"]:.9g}',
                'error_yaw': f'{row["error_yaw"]:.9g}',
                'delta_xy': f'{row["delta_xy"]:.9g}',
                'delta_yaw': f'{row["delta_yaw"]:.9g}',
                'detail': row['detail'],
            })


def quantiles(values):
    values = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not values:
        return {}
    def q(p):
        return values[int(p * (len(values) - 1))]
    return {
        'n': len(values),
        'p50': q(0.50),
        'p90': q(0.90),
        'p95': q(0.95),
        'p99': q(0.99),
        'max': values[-1],
    }


def write_summary(comparison_csv, output_txt):
    rows = list(csv.DictReader(open(comparison_csv)))
    with open(output_txt, 'w') as f:
        f.write(f'rows: {len(rows)}\n')
        for event in sorted(set(r['event'] for r in rows)):
            subset = [r for r in rows if r['event'] == event]
            f.write(f'\n[{event}]\n')
            for key in ('error_xy', 'error_yaw', 'delta_xy', 'delta_yaw'):
                stats = quantiles(float(r[key]) for r in subset if r.get(key))
                f.write(f'{key}: {stats}\n')
        largest = sorted(
            rows,
            key=lambda r: float(r['delta_xy']) if r.get('delta_xy') else 0.0,
            reverse=True,
        )[:20]
        f.write('\n[top_delta_xy]\n')
        for row in largest:
            f.write(
                f"{row['stamp_sec']} event={row['event']} "
                f"delta_xy={row['delta_xy']} error_xy={row['error_xy']} "
                f"x={row['estimate_x']} y={row['estimate_y']} {row['detail']}\n"
            )


def _map_axes_limits(records, poses, extent, margin=0.6):
    # Center the view on the CSV being analyzed. The full Parte A trajectory is still
    # drawn for context, but a partial diagnostic run should not force a full-map zoom.
    if records:
        xs = [r['x'] for r in records]
        ys = [r['y'] for r in records]
    else:
        xs = poses[:, 0].tolist()
        ys = poses[:, 1].tolist()
    if not xs or not ys:
        return extent[0], extent[1], extent[2], extent[3]
    x0 = max(extent[0], min(xs) - margin)
    x1 = min(extent[1], max(xs) + margin)
    y0 = max(extent[2], min(ys) - margin)
    y1 = min(extent[3], max(ys) + margin)
    return x0, x1, y0, y1


def plot_map(records, poses, map_yaml, output_png, title, top_n):
    img, extent = load_map(map_yaml)
    fig, ax = plt.subplots(figsize=(11, 10), constrained_layout=True)
    ax.imshow(img, cmap='gray', origin='upper', extent=extent, alpha=0.78)
    ax.plot(
        poses[:, 0], poses[:, 1],
        color='#1f77b4', linewidth=1.2, linestyle='--',
        label='Parte A trajectory',
    )
    if records:
        mcl_x = [r['x'] for r in records]
        mcl_y = [r['y'] for r in records]
        ax.plot(mcl_x, mcl_y, color='#2ca02c', linewidth=1.2, alpha=0.8, label='MCL estimate')
        deltas = np.array([r['delta_xy'] for r in records], dtype=float)
        threshold = float(np.percentile(deltas, 90)) if len(deltas) > 10 else 0.0
        large = [r for r in records if r['delta_xy'] >= threshold]
        for event in sorted(set(r['event'] for r in large)):
            subset = [r for r in large if r['event'] == event]
            if not subset:
                continue
            event_deltas = np.array([r['delta_xy'] for r in subset], dtype=float)
            sizes = 32.0 + 260.0 * np.clip(
                event_deltas / max(0.01, float(np.percentile(deltas, 95))),
                0.0,
                1.0,
            )
            ax.scatter(
                [r['x'] for r in subset],
                [r['y'] for r in subset],
                s=sizes,
                color=PALETTE.get(event, '#666666'),
                alpha=0.72,
                edgecolors='white',
                linewidths=0.35,
                label=f'{event} top 10%',
            )
        for idx, row in enumerate(sorted(records, key=lambda r: r['delta_xy'], reverse=True)[:top_n], 1):
            ax.annotate(
                str(idx),
                (row['x'], row['y']),
                color='black',
                fontsize=7,
                ha='center',
                va='center',
                bbox={'boxstyle': 'circle,pad=0.16', 'facecolor': 'white', 'alpha': 0.75, 'linewidth': 0.0},
            )
        x0, x1, y0, y1 = _map_axes_limits(records, poses, extent)
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
    ax.set_title(title)
    ax.set_xlabel('map x [m]')
    ax.set_ylabel('map y [m]')
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.2)
    ax.legend(loc='best')
    fig.savefig(output_png, dpi=220)
    plt.close(fig)


def plot_dashboard(records, poses, map_yaml, output_png, title, top_n):
    img, extent = load_map(map_yaml)
    plt.style.use('seaborn-v0_8-whitegrid')
    fig = plt.figure(figsize=(15, 10), constrained_layout=True)
    mosaic = [['map', 'hist'], ['map', 'time']]
    axes = fig.subplot_mosaic(mosaic, width_ratios=[1.45, 1.0])
    ax_map = axes['map']
    ax_hist = axes['hist']
    ax_time = axes['time']

    ax_map.imshow(img, cmap='gray', origin='upper', extent=extent, alpha=0.78)
    ax_map.plot(
        poses[:, 0], poses[:, 1],
        color='#1f77b4', linewidth=1.1, linestyle='--',
        label='Parte A',
    )
    if records:
        t0 = min(r['stamp_sec'] for r in records)
        mcl_x = [r['x'] for r in records]
        mcl_y = [r['y'] for r in records]
        ax_map.plot(mcl_x, mcl_y, color='#2ca02c', linewidth=1.35, label='MCL')
        deltas = np.array([r['delta_xy'] for r in records], dtype=float)
        threshold = float(np.percentile(deltas, 90)) if len(deltas) > 10 else 0.0
        large = [r for r in records if r['delta_xy'] >= threshold]
        for event in sorted(set(r['event'] for r in large)):
            subset = [r for r in large if r['event'] == event]
            ax_map.scatter(
                [r['x'] for r in subset],
                [r['y'] for r in subset],
                s=70,
                color=PALETTE.get(event, '#666666'),
                edgecolor='white',
                linewidth=0.4,
                alpha=0.78,
                label=f'{event} top 10%',
            )
        for idx, row in enumerate(sorted(records, key=lambda r: r['delta_xy'], reverse=True)[:top_n], 1):
            ax_map.annotate(
                str(idx),
                (row['x'], row['y']),
                fontsize=7,
                ha='center',
                va='center',
                bbox={'boxstyle': 'circle,pad=0.16', 'facecolor': 'white', 'alpha': 0.78, 'linewidth': 0.0},
            )
        x0, x1, y0, y1 = _map_axes_limits(records, poses, extent)
        ax_map.set_xlim(x0, x1)
        ax_map.set_ylim(y0, y1)

        bins = np.linspace(0.0, max(0.02, float(np.percentile(deltas, 99))), 32)
        for event in sorted(set(r['event'] for r in records)):
            vals = [r['delta_xy'] for r in records if r['event'] == event]
            ax_hist.hist(
                vals,
                bins=bins,
                alpha=0.58,
                color=PALETTE.get(event, '#666666'),
                label=event,
            )
        ax_hist.set_title('Correction size distribution')
        ax_hist.set_xlabel('delta_xy [m]')
        ax_hist.set_ylabel('updates')
        ax_hist.legend(fontsize=8)

        for event in sorted(set(r['event'] for r in records)):
            subset = [r for r in records if r['event'] == event]
            ax_time.scatter(
                [(r['stamp_sec'] - t0) / 60.0 for r in subset],
                [r['delta_xy'] for r in subset],
                s=14,
                alpha=0.55,
                color=PALETTE.get(event, '#666666'),
                label=event,
            )
        ax_time.set_title('Correction size over bag time')
        ax_time.set_xlabel('minutes since first MCL update')
        ax_time.set_ylabel('delta_xy [m]')
        ax_time.legend(fontsize=8)

    ax_map.set_title('Where the largest MCL corrections happened')
    ax_map.set_xlabel('map x [m]')
    ax_map.set_ylabel('map y [m]')
    ax_map.set_aspect('equal', adjustable='box')
    ax_map.legend(loc='best', fontsize=8)
    fig.suptitle(title, fontsize=15, fontweight='bold')
    fig.savefig(output_png, dpi=220, bbox_inches='tight', facecolor='white')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--diagnostics-csv', required=True)
    parser.add_argument('--trajectory-json', required=True)
    parser.add_argument('--map-yaml', required=True)
    parser.add_argument('--output-prefix', required=True)
    parser.add_argument(
        '--events',
        default='laser,landmark,landmark_oos',
        help='Comma-separated events to include.',
    )
    parser.add_argument('--top-n', type=int, default=20)
    args = parser.parse_args()

    events = {event.strip() for event in args.events.split(',') if event.strip()}
    rows = load_diagnostics(args.diagnostics_csv, events)
    stamps, poses = load_trajectory(args.trajectory_json)

    prefix = Path(args.output_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    output_png = str(prefix) + '.png'
    output_map_png = str(prefix) + '_map.png'
    output_dashboard_png = str(prefix) + '_dashboard.png'
    output_csv = str(prefix) + '_comparison.csv'
    output_txt = str(prefix) + '_summary.txt'

    records = comparison_records(rows, stamps, poses)
    write_comparison(records, output_csv)
    write_summary(output_csv, output_txt)
    plot_map(
        records,
        poses,
        args.map_yaml,
        output_map_png,
        title=f'MCL diagnostics: {Path(args.diagnostics_csv).name}',
        top_n=args.top_n,
    )
    plot_dashboard(
        records,
        poses,
        args.map_yaml,
        output_dashboard_png,
        title=f'MCL diagnostics: {Path(args.diagnostics_csv).name}',
        top_n=args.top_n,
    )
    # Backward-compatible main output: the dashboard is usually the most useful.
    Path(output_png).write_bytes(Path(output_dashboard_png).read_bytes())
    print(output_png)
    print(output_map_png)
    print(output_dashboard_png)
    print(output_csv)
    print(output_txt)


if __name__ == '__main__':
    main()
