import argparse
import math


def args_setting():
    parser = argparse.ArgumentParser('Set default parameter', add_help=False)

    # 车辆参数 --几何尺寸
    parser.add_argument('--vehicle_wheelbase', default=2.8, type=float)  # 前后轮轴距
    parser.add_argument('--vehicle_front_hang', default=0.96, type=float)  # 前悬距离
    parser.add_argument('--vehicle_rear_hang', default=0.929, type=float)  # 后悬距离
    parser.add_argument('--vehicle_width', default=1.942, type=float)  # 车辆宽度
    # 车身长度 vehicle_wheelbase + vehicle_front_hang + vehicle_rear_hang
    parser.add_argument('--vehicle_length', default=4.689, type=float)
    # 车辆双圆半径 math.sqrt((0.25 * vehicle_length) ** 2 + (0.5 * vehicle_width) ** 2)
    parser.add_argument('--radius', default=1.5222, type=float)
    # 车辆后部圆心 至 后轴距离  0.25 * vehicle_length - vehicle_rear_hang
    parser.add_argument('--f2x', default=2.58775, type=float)
    # 车辆前部圆心 至 后轴距离  0.75 * vehicle_length - vehicle_rear_hang
    parser.add_argument('--r2x', default=0.24325, type=float)
    parser.add_argument('--AreaVehicle', default=9.106, type=float)

    # 车辆参数 --运动能力尺寸
    parser.add_argument('--vehicle_phy_max', default=0.7, type=float)  # 前轮最大转弯角度
    parser.add_argument('--vehicle_kappa_max', default=0.30082, type=float)  # 转弯曲率
    parser.add_argument('--vehicle_turning_radius_min', default=3.3243, type=float)  # 最小转弯半径 1 / vehicle_kappa_max
    parser.add_argument('--vehicle_w_max', default=0.5, type=float)
    parser.add_argument('--vehicle_a_max', default=0.5, type=float)
    parser.add_argument('--vehicle_v_max', default=2.5, type=float)

    # 环境参数
    parser.add_argument('--env_x_min', default=0, type=float)
    parser.add_argument('--env_x_max', default=40, type=float)
    parser.add_argument('--env_y_min', default=0, type=float)
    parser.add_argument('--env_y_max', default=40, type=float)
    parser.add_argument('--resolution_xy', default=0.2, type=float)
    parser.add_argument('--resolution_theta', default=math.pi/4, type=float)

    # 优化算法参数
    parser.add_argument('--Nfe', default=40, type=int)
    parser.add_argument('--num_nodes_s', default=120, type=int)
    parser.add_argument('--K_radau', default=3, type=int)
    parser.add_argument('--tf', default=40, type=int)

    args = parser.parse_args()
    return args



