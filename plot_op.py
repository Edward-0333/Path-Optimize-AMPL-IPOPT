import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
import matplotlib.patches as patches
import numpy as np


def plot_op(obs_list, initial_guess, op_solve, start, goal):
    # initial_guess  -->  x_new, y_new, theta_new, v, phy,  a,   w
    # op_solve       -->  xs,    ys,    thetas,   vs, phys, a_s, w_s
    fig, axs = plt.subplots(2, 3)
    plot_path(axs[0, 0], obs_list, initial_guess, op_solve, start, goal)
    plot_theta(axs[0, 1], initial_guess[2], op_solve[2])
    plot_v(axs[0, 2], initial_guess[3], op_solve[3])
    plot_phy(axs[1, 0], initial_guess[4], op_solve[4])
    plot_a(axs[1, 1], initial_guess[5], op_solve[5])
    plot_w(axs[1, 2], initial_guess[6], initial_guess[6])

    plt.show()


def plot_path(ax, raw_obs, initial_guess, op_solve, start, goal):
    # 绘制障碍物
    for o in raw_obs:
        quadrilateral = patches.Polygon(xy=o, linewidth=1,
                                        closed=True, facecolor='gainsboro', edgecolor='silver', linestyle='dashed')
        ax.add_patch(quadrilateral)

    # 绘制初始解
    ax.plot(initial_guess[0], initial_guess[1], color='c')
    length = 5
    start_arrow = FancyArrowPatch((start[0], start[1]), (start[0] + length * np.cos(start[2]),
                                                         start[1] + length * np.sin(start[2])),
                                  arrowstyle='->', mutation_scale=10, color='c')
    goal_arrow = FancyArrowPatch((goal[0], goal[1]), (goal[0] + length * np.cos(goal[2]),
                                                      goal[1] + length * np.sin(goal[2])),
                                 arrowstyle='->', mutation_scale=10, color='c')
    ax.add_patch(start_arrow)
    ax.add_patch(goal_arrow)

    # 绘制优化解
    ax.plot(op_solve[0], op_solve[1],linestyle='--', color='teal')
    op_start = [op_solve[0][0], op_solve[1][0], op_solve[2][0]]
    op_goal = [op_solve[0][-1], op_solve[1][-1], op_solve[2][-1]]
    start_arrow = FancyArrowPatch((op_start[0], op_start[1]), (op_start[0] + length * np.cos(op_start[2]),
                                                               op_start[1] + length * np.sin(op_start[2])),
                                  arrowstyle='->', mutation_scale=10, color='seagreen')
    goal_arrow = FancyArrowPatch((op_goal[0], op_goal[1]), (op_goal[0] + length * np.cos(op_goal[2]),
                                                            op_goal[1] + length * np.sin(op_goal[2])),
                                 arrowstyle='->', mutation_scale=10, color='seagreen')
    ax.add_patch(start_arrow)
    ax.add_patch(goal_arrow)
    # 注释起点和终点位置
    ax.scatter(op_start[0], op_start[1], c='r',s=5)
    ax.scatter(op_goal[0], op_goal[1], c='g',s=5)

    # ax.text(op_start[0], op_start[1], 's', fontsize=10, ha='center')
    # ax.text(op_goal[0], op_goal[1], 'g', fontsize=10, ha='center')
    # 设置坐标轴范围
    # ax.set_xlim(0 - 1, 40 + 1)
    # ax.set_ylim(0 - 1, 40 + 1)
    # 设置坐标轴等其他属性
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('path')


def plot_theta(ax, init_theta, thetas):
    range_theta = list(range(len(init_theta)))
    ax.plot(range_theta, init_theta)
    ax.plot(range_theta, thetas, color='g')
    ax.set_title('theta')


def plot_v(ax, init_v, vs):
    range_v = list(range(len(init_v)))
    ax.plot(range_v, init_v)
    ax.plot(range_v, vs)
    ax.set_title('v')


def plot_phy(ax, init_phy, phys):
    range_phy = list(range(len(init_phy)))
    ax.plot(range_phy, init_phy)
    ax.plot(range_phy, phys)
    ax.set_title('phy')


def plot_a(ax, init_a, a_s):
    range_a = list(range(len(init_a)))
    ax.plot(range_a, init_a)
    ax.plot(range_a, a_s)
    ax.set_title('a')


def plot_w(ax, init_w, w_s):
    range_a = list(range(len(init_w)))
    ax.plot(range_a, init_w)
    ax.plot(range_a, w_s)
    ax.set_title('w')
