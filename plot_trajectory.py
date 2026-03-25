import matplotlib.pyplot as plt
import numpy as np
# 注意：你需要安装 rclpy 和 rosbags 库来读取 db3 文件
# pip install rosbags matplotlib numpy

def plot_paper_figure():
    # 模拟两组跑出来的轨迹数据 (实际使用时，请替换为你从 rosbag 里抽出来的真实 x, y 坐标)
    # 传统 DWB 的轨迹（红色虚线，带有急刹车的折角）
    dwa_x = np.linspace(-4, 4, 100)
    dwa_y = np.where(dwa_x < 0, 0, np.where(dwa_x < 1, 1.5 * dwa_x, 1.5)) 
    
    # 你的 TCPA-DCPA 轨迹（蓝色实线，极其平滑的侧滑绕行）
    tcpa_x = np.linspace(-4, 4, 100)
    tcpa_y = 1.2 * np.exp(-(tcpa_x - 0.5)**2 / 1.5)  # 用高斯曲线模拟平滑侧滑

    # 动态障碍物的穿插路线 (绿色点划线)
    obs_x = np.ones(50) * 0.5
    obs_y = np.linspace(3, -1, 50)

    # 开始 IEEE 风格的高级绘图
    plt.figure(figsize=(8, 5))
    
    # 开启背景网格
    plt.grid(True, linestyle='--', alpha=0.6)
    
    # 画轨迹
    plt.plot(dwa_x, dwa_y, color='#d62728', linestyle='--', linewidth=2.5, label='Standard DWB Planner')
    plt.plot(tcpa_x, tcpa_y, color='#1f77b4', linestyle='-', linewidth=2.5, label='Proposed TCPA-DCPA Planner')
    plt.plot(obs_x, obs_y, color='#2ca02c', linestyle='-.', linewidth=2, label='Dynamic Obstacle Path')
    
    # 标注相遇/危险点
    plt.scatter(0.5, 1.0, color='red', s=100, zorder=5, marker='X')
    plt.annotate('Freezing / Emergency Brake', xy=(0.5, 1.0), xytext=(-1.5, 2.0),
                 arrowprops=dict(facecolor='black', shrink=0.05, width=1, headwidth=5),
                 fontsize=10, fontweight='bold')

    plt.scatter(0.5, 1.2, color='blue', s=80, zorder=5, marker='o')
    plt.annotate('Smooth Evasion', xy=(0.5, 1.2), xytext=(2.0, 1.8),
                 arrowprops=dict(facecolor='black', shrink=0.05, width=1, headwidth=5),
                 fontsize=10, fontweight='bold')

    # 设置极其学术的图例和坐标轴
    plt.title('Qualitative Trajectory Comparison in Dynamic Scenario', fontsize=14, fontweight='bold')
    plt.xlabel('X Coordinate (m)', fontsize=12, fontweight='bold')
    plt.ylabel('Y Coordinate (m)', fontsize=12, fontweight='bold')
    plt.legend(loc='lower right', fontsize=10, framealpha=0.9)
    
    # 限制坐标轴范围以贴合你的 10x10 arena
    plt.xlim(-4.5, 4.5)
    plt.ylim(-1.5, 3.5)
    
    # 导出高分辨率原图（可以直接插到 LaTeX 里）
    plt.savefig('trajectory_comparison.pdf', format='pdf', bbox_inches='tight')
    plt.savefig('trajectory_comparison.png', dpi=300, bbox_inches='tight')
    plt.show()

if __name__ == '__main__':
    plot_paper_figure()