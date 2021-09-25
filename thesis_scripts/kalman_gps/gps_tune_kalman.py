import numpy as np
from numpy.lib.function_base import angle
from open_csv import open_csv
import math
from random import uniform, seed, randrange
import scipy.optimize as opt
from kalman import Kalman
from locals import geodetic_to_enu

np.set_printoptions(suppress=True)
upper_scaler = 1

def rand_sign():
    return [-1, 0, 1][randrange(3)]


def simulate(params, start_lat, start_lon, diff_lat, diff_lon, diff_acc_north, diff_acc_east, angle_change, num_samples, origin_lat, origin_lon):
    seed(1337)

    # Optymalizator może zignorować ograniczenia.
    params = np.clip(params, 0, upper_scaler)

    # Pierwszy pomiar.
    init_x, init_y, _ = geodetic_to_enu(
        start_lat, start_lon, origin_lat, origin_lon)

    # Inicjalizacja filtru.
    Q = np.array([[params[0], params[1]], [params[2], params[3]]])
    R = np.array([[params[4], params[5]], [params[6], params[7]]])
    filter = Kalman((init_x, init_y), Q, R)

    true_lat = start_lat
    true_lon = start_lon

    # Symulacja filtru.
    err = 0
    for _ in range(num_samples):
        true_lat += angle_change
        true_lon += rand_sign() * angle_change

        lat_sample = true_lat + uniform(-diff_lat, diff_lat)
        lon_sample = true_lon + uniform(-diff_lon, diff_lon)

        #print(angle_change)
        #print("%f, %f" % (true_lon, lon_sample))

        true_enu_x, true_enu_y, _ = geodetic_to_enu(
            true_lat, true_lon, origin_lat, origin_lon)

        enu_x, enu_y, _ = geodetic_to_enu(
            lat_sample, lon_sample, origin_lat, origin_lon)

        enu_x, enu_y = filter.update(
            (enu_x, enu_y), (0.5 * uniform(-diff_acc_north, diff_acc_north), 0.5 * uniform(-diff_acc_east, diff_acc_east)))

        err += math.hypot(true_enu_x - enu_x, true_enu_y - enu_y)

    #print("TRUE: %f, ESTI: %f" % (true_enu_x, enu_x))
    return err


def get_cost(params, *args):
    scenarios = args[0]
    origin_lat = args[1]
    origin_lon = args[2]

    cost = 0

    for scenario in scenarios:
        start_lat = scenarios[scenario][0]
        start_lon = scenarios[scenario][1]
        diff_lat = scenarios[scenario][2]
        diff_lon = scenarios[scenario][3]
        diff_acc_north = scenarios[scenario][4]
        diff_acc_east = scenarios[scenario][5]
        angle_change = scenarios[scenario][6]
        num_samples = scenarios[scenario][7]
        cost += simulate(params, start_lat, start_lon, diff_lat, diff_lon, diff_acc_north,
                         diff_acc_east, angle_change, num_samples, origin_lat, origin_lon)
    #print(cost)
    return cost


origin_lat, origin_lon = 54.386279, 18.590767

park = open_csv("./gps_data_park.csv")
park_start_lat, park_start_lon = np.mean(park["lat"]), np.mean(park["lon"])
park_max_diff_lat, park_max_diff_lon = np.max(
    np.abs(park["lat"] - park_start_lat)), np.max(np.abs(park["lon"] - park_start_lon))
acc_max_diff_north = 0.3
acc_max_diff_east = 0.3
park_angle_change = 0.00001  # Około 1.11m ma równiku.
park_num_samples = 100


lower = np.array([0, 0, 0, 0, 0, 0, 0, 0])
upper = upper_scaler * np.array([1, 1, 1, 1, 1, 1, 1, 1])
initial = upper * 0.05 / upper_scaler
solution = opt.minimize(get_cost, initial, bounds=opt.Bounds(lower, upper), method="Nelder-Mead", options={"maxiter": 10000}, args=(
    {"park": (park_start_lat, park_start_lon, park_max_diff_lat, park_max_diff_lon,
              acc_max_diff_north, acc_max_diff_east, park_angle_change, park_num_samples)}, origin_lat, origin_lon))

solution.x = np.clip(solution.x, 0, upper_scaler)
print(solution)
