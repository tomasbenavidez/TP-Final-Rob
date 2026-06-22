from pathlib import Path

import numpy as np
import yaml

from tp_b_navigation.map_loader import _read_pgm

REPO_ROOT = Path(__file__).resolve().parents[3]
ypath = REPO_ROOT / 'mapas' / 'map.yaml'
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
sel=fps(pts,36)
flat=[]
for x,y in sel: flat += [round(float(x),2),round(float(y),2)]
print("N=",len(sel))
print(flat)
# write yaml
out={'landmark_publisher':{'ros__parameters':{'landmarks':flat}}}
cfg = REPO_ROOT / 'tp_final_ws' / 'src' / 'tp_b_navigation' / 'config' / 'landmarks.yaml'
with open(cfg,'w') as f:
    f.write("# Landmarks virtuales (Sistema 3): puntos sobre superficies de pared visibles.\n")
    f.write("# Densificados (~36) para coherencia con la densidad de ArUco real y para romper\n")
    f.write("# la ambiguedad multimodal del MCL. Generados por farthest-point sampling del mapa.\n")
    yaml.safe_dump(out,f,default_flow_style=False)
print("wrote",cfg)
