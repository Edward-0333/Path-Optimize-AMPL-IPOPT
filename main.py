from args import args_setting
import math
import numpy as np
from py_ampl import py_ampl
from plot_op import plot_op


def cla_path_length(xs, ys):
    length = 0
    for i in range(len(xs) - 1):
        length += math.sqrt((xs[i + 1] - xs[i]) ** 2 + (ys[i + 1] - ys[i]) ** 2)
    return length


def resample_path(path, num_nodes_s):
    for i in range(1, len(path[2])):
        while path[2][i] - path[2][i - 1] > math.pi:
            path[2][i] = path[2][i] - 2 * math.pi
        while path[2][i] - path[2][i - 1] < -math.pi:
            path[2][i] = path[2][i] + 2 * math.pi

    x_extended = np.array([])
    y_extended = np.array([])
    theta_extended = np.array([])
    for i in range(len(path[0]) - 1):
        # if i == 95:
        #     print(1)
        distance = math.sqrt((path[0][i + 1] - path[0][i]) ** 2 + (path[1][i + 1] - path[1][i]) ** 2)
        LARGE_NUM = round(distance * 100)
        temp = np.linspace(path[0][i], path[0][i + 1], LARGE_NUM)
        temp = temp[:-1]
        x_extended = np.concatenate((x_extended, temp))

        temp = np.linspace(path[1][i], path[1][i + 1], LARGE_NUM)
        temp = temp[:-1]
        y_extended = np.concatenate((y_extended, temp))

        temp = np.linspace(path[2][i], path[2][i + 1], LARGE_NUM)
        temp = temp[:-1]
        theta_extended = np.concatenate((theta_extended, temp))

    x_extended = np.append(x_extended, path[0][-1])
    y_extended = np.append(y_extended, path[1][-1])
    theta_extended = np.append(theta_extended, path[2][-1])
    index = np.linspace(0, len(x_extended) - 1, num_nodes_s)
    ind_index = [round(x) for x in index]
    x = [x_extended[i] for i in ind_index]
    y = [y_extended[i] for i in ind_index]
    theta = [theta_extended[i] for i in ind_index]
    path_length = 0
    for i in range(len(x) - 1):
        path_length = path_length + math.sqrt((x[i + 1] - x[i]) ** 2 + (y[i + 1] - y[i]) ** 2)
    return [x, y, theta], path_length


def resample_path2(x, y, theta):
    for i in range(1, len(theta)):
        while theta[i] - theta[i - 1] > math.pi:
            print(1)
            theta[i] = theta[i] - 2 * math.pi
        while theta[i] - theta[i - 1] < - math.pi:
            print(1)
            theta[i] = theta[i] + 2 * math.pi

    x_extended = []
    y_extended = []
    theta_extended = []
    distance_all = []
    for i in range(len(x) - 1):
        distance = math.sqrt((x[i + 1] - x[i]) ** 2 + (y[i + 1] - y[i]) ** 2)
        LARGE_NUM = round(distance * 100)
        temp = np.linspace(x[i], x[i + 1], LARGE_NUM)
        temp = temp[:-1]
        x_extended = np.concatenate((x_extended, temp))

        temp = np.linspace(y[i], y[i + 1], LARGE_NUM)
        temp = temp[:-1]
        y_extended = np.concatenate((y_extended, temp))

        temp = np.linspace(theta[i], theta[i + 1], LARGE_NUM)
        temp = temp[:-1]
        theta_extended = np.concatenate((theta_extended, temp))

    x_extended = np.append(x_extended, x[-1])
    y_extended = np.append(y_extended, y[-1])
    theta_extended = np.append(theta_extended, theta[-1])
    index = np.linspace(0, len(x_extended) - 1, 2000)
    ind_index = [round(x) for x in index]

    x = [x_extended[i] for i in ind_index]
    y = [y_extended[i] for i in ind_index]
    theta = [theta_extended[i] for i in ind_index]
    return x, y, theta


def fulfill_profiles(x, y, theta, num_nodes_t, max_t, car_args):
    Nfe = num_nodes_t
    vdr = np.zeros(Nfe)
    for i in range(1, Nfe - 1):
        addition = ((x[i + 1] - x[i]) * math.cos(theta[i])) + ((y[i + 1] - y[i]) * math.sin(theta[i]))
        if addition > 0:
            vdr[i] = 1
        else:
            vdr[i] = -1
    v = np.zeros(Nfe)
    a = np.zeros(Nfe)
    dt = max_t / Nfe
    for i in range(1, Nfe):
        v[i] = vdr[i] * math.sqrt(((x[i] - x[i - 1]) / dt) ** 2 + ((y[i] - y[i - 1]) / dt) ** 2)
    for i in range(1, Nfe):
        a[i] = (v[i] - v[i - 1]) / dt
    phy = np.zeros(Nfe)
    w = np.zeros(Nfe)
    for i in range(1, Nfe - 1):
        phy[i] = math.atan((theta[i + 1] - theta[i]) * car_args.vehicle_wheelbase / (dt * v[i]))
        if phy[i] == None:
            phy[i] = 0
        if phy[i] > car_args.vehicle_phy_max:
            phy[i] = car_args.vehicle_phy_max
        elif phy[i] < -car_args.vehicle_phy_max:
            phy[i] = -car_args.vehicle_phy_max

    for i in range(1, Nfe - 1):
        w[i] = (phy[i + 1] - phy[i]) / dt
        if w[i] > car_args.vehicle_w_max:
            w[i] = car_args.vehicle_w_max
        elif w[i] < - car_args.vehicle_w_max:
            w[i] = -car_args.vehicle_phy_max

    return v, a, phy, w


def generate_init_guess(args, xs, ys, thetas, path_length):
    # 过滤一些重复的点
    filter_x = [xs[0]]
    filter_y = [ys[0]]
    filter_theta = [thetas[0]]
    for i in range(len(xs) - 1):
        if not (xs[i + 1] == xs[i] and ys[i + 1] == ys[i]):
            filter_x.append(xs[i + 1])
            filter_y.append(ys[i + 1])
            filter_theta.append(thetas[i + 1])
    max_t = round(path_length * 2)
    Nfe = args.Nfe
    num_nodes_s = args.num_nodes_s
    num_nodes_t = Nfe * 3 + 1
    re_path, re_length = resample_path([filter_x, filter_y, filter_theta], num_nodes_s)
    max_s = re_length
    resolution_s = max_s / num_nodes_s
    resolution_t = max_t / num_nodes_t
    range_t = list(range(0, num_nodes_t))
    s = []
    for i in range_t:
        if i <= num_nodes_s:
            s.append(i)
        else:
            i = num_nodes_s
            s.append(i)
    s = [i * resolution_s for i in s]
    #  上面一段主要是为了使用ST图求解速度时使用👆， 不进行速度规划时，x_new和path[0]应该是一样的
    x, y, theta = resample_path2(re_path[0], re_path[1], re_path[2])
    ss = np.zeros(len(x))
    for i in range(1, len(x)):
        ss[i] = ss[i - 1] + math.sqrt((x[i] - x[i - 1]) ** 2 + (y[i] - y[i - 1]) ** 2)
    x_new = []
    y_new = []
    theta_new = []
    for i in range(len(s)):
        err = [abs(j - s[i]) for j in ss]
        ind = err.index(min(err))
        x_new.append(x[ind])
        y_new.append(y[ind])
        theta_new.append(theta[ind])
    v, a, phy, w = fulfill_profiles(x_new, y_new, theta_new, num_nodes_t, max_t, args)
    return x_new, y_new, theta_new, v, phy, a, w


def main():
    # 获取算法的基本参数
    args = args_setting()
    xs = [29.95677654, 29.83241658, 29.63083257, 29.35539011, 29.00874549, 28.59276364, 28.10852062, 27.55639472,
          26.93624914, 26.24770624, 25.49050734, 24.6649414, 23.77230915, 22.81536755, 21.79867717, 20.72876236,
          19.61400741, 18.46426836, 17.29026726, 16.10281391, 14.91217871, 13.72799634, 12.55934067, 11.41476268,
          10.3022736, 9.22922135, 8.20216103, 7.22679699, 6.3080204, 5.45003101, 4.65651546, 3.93085012, 3.27630097,
          2.69620006, 2.19408565, 1.77379901, 1.43953584, 1.19585266, 1.04763069, 1.0]
    ys = [5.04441631, 5.17298658, 5.38088474, 5.66274892, 6.01249988, 6.42320658, 6.8869918, 7.3949757, 7.93726289,
          8.50298809, 9.08044706, 9.65734988, 10.22124092, 10.76012776, 11.26334473, 11.72264047, 12.13342491,
          12.49605051, 12.81689446, 13.10811393, 13.38508233, 13.66310022, 13.95435647, 14.2664816, 14.60278072,
          14.96288036, 15.34347459, 15.73908834, 16.14279006, 16.54681013, 16.94304494, 17.32344541, 17.68029999,
          18.00642732, 18.29529374, 18.54106863, 18.73862622, 18.88349901, 18.97178392, 19.0]
    thetas = [5.47866497, 5.47973507, 5.48265455, 5.48823283, 5.49720098, 5.51020675, 5.52780592, 5.55044808,
              5.57845495, 5.61198834, 5.65100552, 5.69520002, 5.74392783, 5.79612101, 5.85019491, 5.9039589, 5.95454443,
              5.99836354, 6.03137134, 6.05061802, 6.05534728, 6.04700014, 6.02866122, 6.00355256, 5.974401, 5.94343951,
              5.91242853, 5.88269718, 5.8551964, 5.83055674, 5.80914482, 5.79111414, 5.77644854, 5.76499729, 5.75650254,
              5.75061984, 5.74693304, 5.74496453, 5.74418176, 5.74400056]

    obs = [np.array([[17.25, 24.5], [22.75, 24.5], [22.75, 35.5], [17.25, 35.5]], dtype=np.float32),
           np.array([[14.5, 14.5], [19.5, 14.5], [19.5, 29.5], [14.5, 29.5]], dtype=np.float32),
           np.array([[27.0, 21.5], [33.0, 21.5], [33.0, 24.5], [27.0, 24.5]], dtype=np.float32),
           np.array([[26.23, 7.0], [18.41, 11.47], [20.02, 5.31]], dtype=np.float32),
           np.array([[14.86, 7.0], [13.0, 10.50], [7.76, 7.0], [13.0, 4.07]], dtype=np.float32),
           np.array([[32.80, 26.0], [28.91, 27.87], [27.85, 22.28]], dtype=np.float32),
           np.array([[42.93, 32.0], [36.60, 36.14], [36.03, 26.87]], dtype=np.float32),
           np.array([[16.02, 6.65], [17.971, 6.65], [17.971, 11.34], [16.029, 11.34]], dtype=np.float32),
           np.array([[38.02, 22.65], [39.91, 22.65], [39.71, 27.45], [38.29, 27.45]], dtype=np.float32)]
    start = [29.95, 5.04, 5.47]
    goal = [1.0, 19.0, 5.74]

    path_length = cla_path_length(xs, ys)

    initial_guess = generate_init_guess(args, xs, ys, thetas, path_length)
    true_op_solve = py_ampl(initial_guess, obs, start, goal, args)
    plot_op(obs, initial_guess, true_op_solve, start, goal)


if __name__ == "__main__":
    main()
