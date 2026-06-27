import argparse
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]


def _read_pgm(path):
    """Lee PGM P5/P2 sin importar ROS, para que el generador sea portable."""
    data = path.read_bytes()
    tokens = []
    index = 0
    while len(tokens) < 4 and index < len(data):
        while index < len(data) and data[index:index + 1].isspace():
            index += 1
        if data[index:index + 1] == b'#':
            while index < len(data) and data[index:index + 1] not in (b'\n', b'\r'):
                index += 1
            continue
        start = index
        while index < len(data) and not data[index:index + 1].isspace():
            index += 1
        tokens.append(data[start:index])
    magic, width, height, maxval = tokens
    width, height, maxval = int(width), int(height), int(maxval)
    index += 1
    if magic == b'P5':
        image = np.frombuffer(
            data, dtype=np.uint8, count=width * height, offset=index
        ).reshape((height, width)).copy()
    elif magic == b'P2':
        image = np.array(
            [int(token) for token in data[index:].split()],
            dtype=np.uint8,
        )[:width * height].reshape((height, width))
    else:
        raise ValueError(f'PGM no soportado: {magic!r}')
    if maxval != 255:
        image = (image.astype(float) * 255.0 / maxval).astype(np.uint8)
    return width, height, image
parser = argparse.ArgumentParser(
    description='Genera landmarks virtuales sobre superficies del mapa.')
parser.add_argument(
    'map_yaml', nargs='?', type=Path,
    default=REPO_ROOT / 'mapas' / 'map_sim.yaml')
parser.add_argument('--count', type=int, default=60)
parser.add_argument(
    '--output', type=Path,
    default=(REPO_ROOT / 'tp_final_ws' / 'src' / 'tp_b_navigation'
             / 'config' / 'landmarks.yaml'))
args = parser.parse_args()
if args.count <= 0:
    parser.error('--count debe ser positivo')
ypath = args.map_yaml.expanduser()
meta=yaml.safe_load(open(ypath))
res=float(meta['resolution']); ox,oy=meta['origin'][0],meta['origin'][1]
img_rel=meta['image']; ipath=Path(img_rel) if Path(img_rel).is_absolute() else ypath.parent / img_rel
W,H,img=_read_pgm(ipath)
p=img.astype(float)/255.0; occ=(1.0-p)>0.65   # occupied
occ=np.flipud(occ)  # row0=bottom like OccupancyGrid
Hh,Ww=occ.shape
# wall-surface cells: occupied cell with >=1 free 4-neighbor (faces open space -> visible)
free=~occ
surf=np.zeros_like(occ)
surf[1:-1,1:-1]= occ[1:-1,1:-1] & (free[:-2,1:-1]|free[2:,1:-1]|free[1:-1,:-2]|free[1:-1,2:])
ys,xs=np.where(surf)
pts=np.stack([ox+(xs+0.5)*res, oy+(ys+0.5)*res],axis=1)
# farthest-point sampling for even spread
def fps(P,k):
    idx=[0]; d=np.full(len(P),1e9)
    for _ in range(k-1):
        last=P[idx[-1]]
        d=np.minimum(d,np.hypot(P[:,0]-last[0],P[:,1]-last[1]))
        idx.append(int(np.argmax(d)))
    return P[idx]
np.random.seed(1)
# start from a central-ish surface point
if args.count > len(pts):
    parser.error(f'--count excede las {len(pts)} superficies disponibles')
sel=fps(pts,args.count)
flat=[]
for x,y in sel: flat += [round(float(x),2),round(float(y),2)]
print("N=",len(sel))
print(flat)
# write yaml
out={'landmark_publisher':{'ros__parameters':{'landmarks':flat}}}
cfg = args.output.expanduser()
with open(cfg,'w') as f:
    f.write("# Landmarks virtuales (Sistema 3): puntos sobre superficies de pared visibles.\n")
    f.write(f"# Densificados ({len(sel)}) para coherencia con la densidad de ArUco real y para romper\n")
    f.write("# la ambiguedad multimodal del MCL. Generados por farthest-point sampling del mapa.\n")
    yaml.safe_dump(out,f,default_flow_style=False)
print("wrote",cfg)
