import numpy as np
from PIL import Image
import os

def create_arena_map():
    # 分辨率 0.05m/pixel，10m x 10m 的场地对应 200 x 200 像素
    # 为了留出一点安全余量，我们建一张 12m x 12m 的地图 (240 x 240 像素)
    size = 240
    
    # 254 代表完全空闲的可用区域 (白色)
    map_data = np.full((size, size), 254, dtype=np.uint8)
    
    # 画出 10x10m 的边界墙壁 (0 代表致命障碍物，黑色)
    # 墙厚大概 4 个像素 (0.2米)
    wall_thickness = 4
    margin = 20 # 上下左右各留 1 米的边距 (20 * 0.05 = 1.0m)
    
    # 上下墙壁
    map_data[margin:margin+wall_thickness, margin:-margin] = 0
    map_data[-margin-wall_thickness:-margin, margin:-margin] = 0
    # 左右墙壁
    map_data[margin:-margin, margin:margin+wall_thickness] = 0
    map_data[margin:-margin, -margin-wall_thickness:-margin] = 0

    # 保存为 PGM 图片格式
    img = Image.fromarray(map_data)
    img.save('arena_map.pgm')

    # 生成配套的 YAML 配置文件
    yaml_content = """image: arena_map.pgm
resolution: 0.05
origin: [-6.0, -6.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    with open('arena_map.yaml', 'w') as f:
        f.write(yaml_content)
        
    print("✅ 完美仿真地图 arena_map.pgm 和 arena_map.yaml 已生成！")

if __name__ == '__main__':
    create_arena_map()