"""Static engineering preview of the actual R1 CAD, not a concept rendering."""
import json
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from revision import build

HERE=Path(__file__).resolve().parent


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--inputs',type=Path,default=HERE/'inputs.json');parser.add_argument('--output',type=Path,default=HERE/'output')
    args=parser.parse_args()
    config=json.loads(args.inputs.read_text());root,_=build(config)
    fig=plt.figure(figsize=(14,7),facecolor='#fafafa')
    colors={'TPU95A':'#56b2b0','PETG':'#ed9d3e','6061':'#a6b4c5','steel':'#667381','purchased':'#566ca3','belt':'#343b45','foam':'#c6aada'}
    for index,title,lid in [(1,'Assembly — revised lid / motor roof',True),(2,'Interior — battery tray / electronics platform',False)]:
        ax=fig.add_subplot(1,2,index,projection='3d')
        for b in root.bodies():
            if b.material=='keepout' or (not lid and b.name.startswith('Lid_')):continue
            vertices,triangles=b.shape.tessellate(0.5,0.3)
            v=np.array([p.toTuple() for p in vertices]);tri=v[np.array(triangles)]
            alpha=0.25 if b.name.startswith('Lid_') else 0.42 if b.name=='Tub_TPU' else 0.92
            surface=Poly3DCollection(tri,facecolors=colors.get(b.material,'#aaa'),edgecolors='none',alpha=alpha)
            ax.add_collection3d(surface)
        ax.set_xlim(-110,110);ax.set_ylim(-125,135);ax.set_zlim(0,80);ax.set_box_aspect((220,260,80));ax.view_init(elev=32,azim=-55)
        ax.set_xlabel('X (mm)');ax.set_ylabel('Front +Y (mm)');ax.set_zlabel('Z (mm)');ax.set_title(title,fontsize=12,pad=15)
    fig.suptitle(config['revision']+' — DRAFT / UNMEASURED HARDWARE',fontsize=16,fontweight='bold',y=0.98)
    fig.text(0.5,0.03,'TPU: teal  |  PETG: orange  |  Metal: grey  |  Purchased envelopes: blue',ha='center',fontsize=10)
    fig.tight_layout(rect=(0,0.05,1,0.93));fig.savefig(args.output/'assembly_preview.png',dpi=160);plt.close(fig)


if __name__=='__main__':main()
