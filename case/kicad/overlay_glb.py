import trimesh, numpy as np, sys
from PIL import Image, ImageDraw
from scipy.spatial import ConvexHull
S = sys.argv[1] if len(sys.argv) > 1 else "."   # folder holding rect_top.png / rect_bot.png (rectified photos)
R=40; E=18
s=trimesh.load('goku.glb')
groups={}
for n in s.graph.nodes_geometry:
    T,g=s.graph[n]
    if 'PCB' in g or 'pcb' in g.lower(): continue
    m=s.geometry[g].copy(); m.apply_transform(T); v=m.vertices*1000
    side='top' if v[:,1].mean()>0.75 else 'bot'
    groups.setdefault((side, n.split('_')[0] if False else n), []).append(v)
for side in ('top','bot'):
    im=Image.open(f'{S}/rect_{side}.png').convert('RGB'); d=ImageDraw.Draw(im)
    for (sd,n),vs in groups.items():
        if sd!=side: continue
        v=np.vstack(vs); p=np.c_[(v[:,0]-100+E)*R,(v[:,2]-100+E)*R]
        if len(p)<3: continue
        try: h=ConvexHull(p)
        except Exception: continue
        q=[tuple(p[i]) for i in h.vertices]; d.line(q+[q[0]],fill=(0,255,255),width=2)
    # outline
    for (sd,n),vs in []: pass
    pcb=[s.geometry[g].copy().apply_transform(s.graph[n][0]) for n in s.graph.nodes_geometry for g in [s.graph[n][1]] if 'PCB' in g]
    for m in pcb:
        v=m.vertices*1000
        if np.ptp(v[:,0])+np.ptp(v[:,2])<20:
            p=v[v[:,1]>1.4]
            if len(p)>1:
                q=[((a-100+E)*R,(c-100+E)*R) for a,_,c in p[:2]]; d.line(q,fill=(255,0,0),width=3)
    im.resize((900,900)).save(f'{S}/glb_{side}.png')
