#!/usr/bin/env python3
"""
run_foam_on_visual.py:
Runs CoMMALab/foam Medial Axis Transform (MAT) on all 7 Fanuc R-2000iC/165F visual DAE meshes.
Extracts high-fidelity spherical decomposition capturing fine surface details and fillets.
"""

import os
import sys
import json
import trimesh
from pathlib import Path
import numpy as np

from foam import spherize_mesh, SphereEncoder

def main():
    script_dir = Path(__file__).resolve().parent
    models_dir = script_dir.parent / "models"
    vis_dir = models_dir / "meshes" / "r2000ic_165f" / "visual"
    out_dir = models_dir / "foam_visual_spheres"
    out_dir.mkdir(exist_ok=True)
    temp_dir = models_dir / "temp_obj"
    temp_dir.mkdir(exist_ok=True)

    # 连杆定制化参数配置：
    # 针对各连杆截然不同的几何特征(大底座、细长臂筒、扁平法兰)进行专门适配
    links = [
        ("base_link", "base.dae", 4, {"expand": False}),
        ("J1_link",   "j1.dae",   4, {"expand": False}),
        ("J2_link",   "j2.dae",   6, {"expand": False}),
        ("J3_link",   "j3.dae",   9, {"expand": False, "balExcess": 0.08}),  # 4 肘部 + 5 前臂筒段，彻底解决前臂太粗问题
        ("J4_link",   "j4.dae",   4, {"expand": False}),
        ("J5_link",   "j5.dae",   3, {"expand": False}),
        ("J6_link",   "j6.dae",   2, {"expand": True}),   # J6 为 3.6cm 极薄法兰盘，expand=True 确保端面完全覆盖
    ]

    all_spheres = {}

    for item in links:
        link_name = item[0]
        filename = item[1]
        branch = item[2]
        custom_kwargs = item[3] if len(item) > 3 else {}

        dae_path = vis_dir / filename
        obj_path = temp_dir / f"{filename[:-4]}.obj"
        out_json = out_dir / f"{filename[:-4]}_spheres.json"

        print(f"\n=======================================================", flush=True)
        print(f"Converting and Spherizing Visual Mesh: {link_name} ({filename})", flush=True)
        print(f"Target branch: {branch} spheres, Custom options: {custom_kwargs}", flush=True)
        print(f"=======================================================", flush=True)

        # 1. Load DAE and convert to clean OBJ
        scene_or_mesh = trimesh.load(dae_path)
        if isinstance(scene_or_mesh, trimesh.Scene):
            mesh = scene_or_mesh.dump(concatenate=True)
        else:
            mesh = scene_or_mesh
        mesh.export(obj_path)
        print(f"  Mesh vertices: {len(mesh.vertices)}, faces: {len(mesh.faces)}", flush=True)

        spherization_kwargs = {
            # 1. 树结构控制
            'depth': 1,             # 球体分层树(Sphere-Tree)的最大深度(0-indexed)。1 表示生成两层，最终提取叶子层作为碰撞球集合
            'branch': branch,       # 树的分支因子/叶子节点目标球体数量。即期望该连杆最终保留的最大球数(如 4、6、9 等)
            
            # 2. 算法核心方法
            'method': 'medial',     # 球体化算法内核：可选 'medial'(中轴变换法，最符合机械臂轮廓)、'grid'、'spawn'、'octree'、'hubbard'
            'testerLevels': 2,      # 误差评估细度级别(1 或 2)。级别越高表面采样测试点越密集，拟合精度越高，离线计算略微增加
            
            # 3. 表面采样覆盖设置(高分辨率升级)
            'numCover': 5000,       # 整个网格表面撒布的覆盖测试采样点总数(提升至 5000)，大幅提高对复杂 CAD 细节的覆盖评估
            'minCover': 5,          # 每个三角面片上强制保留的最少采样点数，防止倒角、小法兰等细小面片因面积过小被忽略
            
            # 4. 中轴种子点与细化(高分辨率升级)
            'initSpheres': 1000,    # 中轴骨架(Medial Axis)初始生成的内切球种子数量(提升至 1000)，更细致捕捉臂身渐缩特征
            'minSpheres': 200,      # 细分过程中各局部区域保留的最小候选球数阈值(提升至 200)，避免关键局部特征被过早抹平
            'erFact': 2,            # 误差缩减因子(Error Reduction Factor)，控制中轴迭代收敛细化过程中误差容限的收紧倍率
            
            # 5. 膨胀与合并后处理
            'expand': False,        # 是否启用 EXPAND(向外膨胀)算法：
                                    #   True:  强制扩大球体半径以绝对包住所有外凸角点(适合极薄法兰 J6)；
                                    #   False: 严格保持中轴内切球尺寸，不盲目虚胀，球体极其贴合金属表面(适合绝大部分臂身)
            'merge': True,          # 是否启用 MERGE(合并)算法：若相邻两球高度重叠或覆盖冗余，自动融合为一个最优球以精简数量
            'burst': False,         # 是否启用 BURST(突发分裂)算法：对覆盖不良区域强行分裂额外小球补洞(一般设为 False 保持球数紧凑)
            
            # 6. 数值松弛与平衡优化
            'optimise': True,       # 是否对最终球心坐标与半径启用全局局部优化(基于 Simplex/Powell 松弛，提升覆盖贴合度)
            'maxOptLevel': 1,       # 优化算法作用的最大层级(0 表示仅优化根节点，1 表示对最终叶子球集合全部进行数值微调)
            'balExcess': 0.05,      # BALANCE 平衡树优化过程中允许的超额误差裕度(通常取 5% 即 0.05)
            'verify': True,         # 是否在计算前对网格的水密性(Watertightness)与流形拓扑进行严格合法性校验
        }
        # 合并连杆专用定制项(如 J3 的 balExcess: 0.08、J6 的 expand: True)
        spherization_kwargs.update(custom_kwargs)

        process_kwargs = {
            'manifold_leaves': 1000,# 将非流形 CAD 网格修复为连续水密流形时，Octree 细分的最大叶子面片数
            'ratio': 0.2,           # 网格修复与抽稀时允许保留的细节比例
        }

        try:
            spheres_levels = spherize_mesh(
                str(obj_path),
                obj_path,
                scale=np.array([1.0, 1.0, 1.0]),
                spherization_kwargs=spherization_kwargs,
                process_kwargs=process_kwargs
            )

            with open(out_json, 'w') as f:
                f.write(json.dumps(spheres_levels, indent=4, cls=SphereEncoder))

            # Pick deepest level
            sph_level = spheres_levels[-1]
            spheres_list = sph_level.spheres
            all_spheres[link_name] = [
                {'origin': list(s.origin), 'radius': float(s.radius)}
                for s in spheres_list
            ]

            print(f"  -> SUCCESS! Found {len(spheres_list)} spheres:", flush=True)
            for idx, s in enumerate(spheres_list):
                print(f"     Sphere {idx}: center=({s.origin[0]:+.4f}, {s.origin[1]:+.4f}, {s.origin[2]:+.4f}), radius={s.radius:.4f} m", flush=True)

        except Exception as e:
            print(f"  -> Error processing {link_name}: {e}", flush=True)

    # Clean up temp obj and material files
    for p in temp_dir.iterdir():
        try:
            p.unlink()
        except:
            pass
    try:
        temp_dir.rmdir()
    except:
        pass

    summary_path = models_dir / "r2000ic_165f_visual_foam_spheres.json"
    with open(summary_path, 'w') as f:
        json.dump(all_spheres, f, indent=2)
    print(f"\nAll FOAM visual spheres saved to: {summary_path}", flush=True)

if __name__ == "__main__":
    main()
