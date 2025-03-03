import numpy as np
from shapely.geometry import Polygon
from amplpy import AMPL
import math
import pickle


def py_ampl(initial_guess, obs_list, start, goal, args):
    # xs, ys, thetas, raw_obs, expand_obs_1, expand_obs_2, path_length, random_sg
    # initial_guess  -->  x_new, y_new, theta_new, v, phy, a, w
    init_x = convert_profile(initial_guess[0])
    init_y = convert_profile(initial_guess[1])
    init_theta = convert_profile(initial_guess[2])

    init_v = convert_profile(initial_guess[3])
    init_phy = convert_profile(initial_guess[4])
    init_a = convert_profile(initial_guess[5])
    init_w = convert_profile(initial_guess[6])
    # obs_list = []
    # obs = solve_pkl[3]
    obs_area = []
    obs_centre = []
    for o in obs_list:
        polygon = Polygon(o)
        obs_area.append(polygon.area)
        centroid = polygon.centroid
        obs_centre.append((centroid.x, centroid.y))

    ampl = AMPL()
    ampl.read('py_NLP.mod')

    ampl.param["Nfe"] = args.Nfe
    ampl.param["K_radau"] = args.K_radau
    ampl.param["Nobs"] = len(obs_list)  # 障碍物的个数
    ampl.param["amax"] = args.vehicle_a_max
    ampl.param["vmax"] = args.vehicle_v_max
    ampl.param["wmax"] = args.vehicle_w_max
    ampl.param["phymax"] = args.vehicle_phy_max
    ampl.param["L_wheelbase"] = args.vehicle_wheelbase
    ampl.param["tf"] = args.tf
    ampl.param["AreaVehicle"] = args.AreaVehicle

    f1 = args.vehicle_front_hang + args.vehicle_wheelbase
    r1 = args.vehicle_rear_hang
    hw = args.vehicle_width / 2

    random_start = np.array(start)
    random_goal = np.array(goal)
    # 需要将起点终点的theta值换掉，因为在生成initial guess的过程中，可能产生变化
    # random_start[2] = initial_guess[2][0]
    # random_goal[2] = initial_guess[2][-1]
    start_goal = np.concatenate((np.array(random_start), np.array(random_goal)))
    ampl.param["BV"] = {i + 1: x_y_theta for i, x_y_theta in enumerate(start_goal)}
    dljtauk = [-9.0000, -4.1394, 1.7394, -3, 10.0488, 3.2247, -3.5678, 5.5320, -1.3821, 1.1678, 0.7753, -7.5320, 0.3333,
               -0.2532, 1.0532, 5.0000]
    tau = [0, 0.1550510257216822, 0.6449489742783178, 1.0]
    ampl_K = ampl.get_set('K')
    len_k = len(ampl_K.to_pandas())
    ampl.param["dljtauk"] = {
        (i, j): dljtauk[i * len_k + j]
        for i in ampl_K
        for j in ampl_K
    }
    ampl.param['tau'] = {i: t for i, t in enumerate(tau)}
    num_obs = ampl.get_parameter('Nobs').value()
    max_obs_edge = 7
    ampl_I = ampl.get_set('I')
    Nov = [4] * num_obs
    ampl.param['Nov'] = {i+1: n for i, n in enumerate(Nov)}
    OV_dict = {}
    OC_dict = {}
    for i in range(num_obs):  # +1
        for j in ampl_I:
            for k in ampl_K:
                for m in range(max_obs_edge):  # +1
                    for n in [1, 2]:
                        try:
                            temp = obs_list[i][m][n - 1]
                            OV_dict[(i + 1, j, k, m + 1, n)] = temp
                        except IndexError:
                            OV_dict[(i + 1, j, k, m + 1, n)] = 0
                        temp_c = obs_centre[i][n-1]
                        OC_dict[(i+1, j, k, n)] = temp_c

    ampl.param['OV'] = OV_dict
    ampl.param['OC'] = OC_dict
    ampl.param['Area'] = {i+1: t for i, t in enumerate(obs_area)}
    ampl_Nfe = ampl.get_parameter('Nfe').value()
    init_x_dict = {}
    init_y_dict = {}
    init_theta_dict = {}
    init_v_dict = {}
    init_phy_dict = {}
    egoV_dict = {}
    init_a_dict = {}
    init_w_dict = {}
    for i in range(1, ampl_Nfe+1):
        for j in range(0, 4):
            init_x_dict[(i, j)] = init_x[i-1][j]
            init_y_dict[(i, j)] = init_y[i-1][j]
            init_theta_dict[(i, j)] = init_theta[i-1][j]
            init_v_dict[(i, j)] = init_v[i-1][j]
            init_phy_dict[(i, j)] = init_phy[i-1][j]
            egoV_dict[(i, j, 1, 1)] = init_x[i-1][j] + f1 * math.cos(init_theta[i-1][j]) - hw * math.sin(init_theta[i-1][j])
            egoV_dict[(i, j, 2, 1)] = init_x[i-1][j] + f1 * math.cos(init_theta[i-1][j]) + hw * math.sin(init_theta[i-1][j])
            egoV_dict[(i, j, 3, 1)] = init_x[i-1][j] - r1 * math.cos(init_theta[i-1][j]) + hw * math.sin(init_theta[i-1][j])
            egoV_dict[(i, j, 4, 1)] = init_x[i-1][j] - r1 * math.cos(init_theta[i-1][j]) - hw * math.sin(init_theta[i-1][j])

            egoV_dict[(i, j, 1, 2)] = init_y[i-1][j] + f1 * math.sin(init_theta[i-1][j]) + hw * math.cos(init_theta[i-1][j])
            egoV_dict[(i, j, 2, 2)] = init_y[i-1][j] + f1 * math.sin(init_theta[i-1][j]) - hw * math.cos(init_theta[i-1][j])
            egoV_dict[(i, j, 3, 2)] = init_y[i-1][j] - r1 * math.sin(init_theta[i-1][j]) - hw * math.cos(init_theta[i-1][j])
            egoV_dict[(i, j, 4, 2)] = init_y[i-1][j] - r1 * math.sin(init_theta[i-1][j]) + hw * math.cos(init_theta[i-1][j])
        for j in range(1, 4):
            init_a_dict[(i, j)] = init_a[i-1][j]
            init_w_dict[(i, j)] = init_w[i-1][j]
    ampl.var['x'] = init_x_dict
    ampl.var['y'] = init_y_dict
    ampl.var['theta'] = init_theta_dict
    ampl.var['v'] = init_v_dict
    ampl.var['phy'] = init_phy_dict
    ampl.var['a'] = init_a_dict
    ampl.var['w'] = init_w_dict
    ampl.var['egoV'] = egoV_dict
    ampl.option["solver"] = "ipopt"
    ampl.solve()
    solve_x = ampl.get_variable('x').to_pandas()
    solve_y = ampl.get_variable('y').to_pandas()
    solve_theta = ampl.get_variable('theta').to_pandas()
    solve_v = ampl.get_variable('v').to_pandas()
    solve_phy = ampl.get_variable('phy').to_pandas()
    solve_a = ampl.get_variable('a').to_pandas()
    solve_w = ampl.get_variable('w').to_pandas()
    # with open(f'data/solve_all.pkl', 'wb') as file:
    #     pickle.dump([solve_x, solve_y, solve_theta, solve_v, solve_phy, solve_a, solve_w, random_sg], file)

    print('求解完成')
    # 将优化解恢复成列表格式
    xs, ys, thetas, vs, phys, a_s, w_s = reshape_solve([solve_x, solve_y, solve_theta, solve_v, solve_phy, solve_a,
                                                        solve_w])
    return [xs, ys, thetas, vs, phys, a_s, w_s]


def convert_profile(data):
    data = np.array(data)
    Nfe = int((len(data) - 1) / 3)
    data_new = np.zeros((Nfe, 4))
    data_new[0, 0: 4] = data[0: 4]
    data = data[4:]
    for i in range(1, Nfe):
        # if i == 79:
        #     print(1)
        data_new[i, 0] = data_new[i-1, 3]
        data_new[i, 1:4] = data[0:3]
        data = data[3:]
    return data_new


def reshape_solve(op_solve):
    solve_x = np.array(op_solve[0].iloc[:, 0].tolist()).reshape(-1, 4)
    solve_y = np.array(op_solve[1].iloc[:, 0].tolist()).reshape(-1, 4)
    solve_theta = np.array(op_solve[2].iloc[:, 0].tolist()).reshape(-1, 4)
    solve_v = np.array(op_solve[3].iloc[:, 0].tolist()).reshape(-1, 4)
    solve_phy = np.array(op_solve[4].iloc[:, 0].tolist()).reshape(-1, 4)

    xs = np.append(solve_x[:, :3].flatten(), solve_x[-1][-1])
    ys = np.append(solve_y[:, :3].flatten(), solve_y[-1][-1])
    thetas = np.append(solve_theta[:, :3].flatten(), solve_theta[-1][-1])
    vs = np.append(solve_v[:, :3].flatten(), solve_v[-1][-1])
    phys = np.append(solve_phy[:, :3].flatten(), solve_phy[-1][-1])
    a_s = np.append(0, op_solve[5].iloc[:, 0].tolist())
    w_s = np.append(0, op_solve[6].iloc[:, 0].tolist())
    return xs, ys, thetas, vs, phys, a_s, w_s