from normalize import normalize
import matplotlib.pyplot as plt
from open_csv import open_csv
from kalman import Kalman
from locals import geodetic_to_enu, enu_to_geodetic
import numpy as np
from random import uniform

origin_lat, origin_lon = 54.386279, 18.590767

park = open_csv("./gps_data_park.csv")

init_x, init_y, _ = geodetic_to_enu(park["lat"][0], park["lon"][0], origin_lat, origin_lon)
Q = np.array([[0.11973523, 0], [0, 0.1939319]])
R = np.array([[1.0, 0], [0, .16446802]])
filter = Kalman((init_x, init_y), Q, R)

filtered = {"lat": [], "lon": []}

for lat, lon in zip(park["lat"], park["lon"]):
    x, y, _ = geodetic_to_enu(lat, lon, origin_lat, origin_lon)
    x, y = filter.update((x, y), (uniform(-0.3, 0.3), uniform(-0.3, 0.3)))
    lat_filtered, lon_filtered = enu_to_geodetic(x, y, _, origin_lat, origin_lon)
    print("%f, %f" % (lat, lat_filtered))
    filtered["lat"].append(lat_filtered)
    filtered["lon"].append(lon_filtered)

park = normalize(park)
filtered = normalize(filtered)

plt.scatter(park["lat"], park["lon"], marker='s', label="Pomiar")
plt.scatter(filtered["lat"], filtered["lon"], marker='s', label="Kalman")
plt.xlabel("Szerokość geograficzna [°]")
plt.ylabel("Długość geograficzna [°]")
plt.title("Porównanie pomiaru bezpośredniego z przepuszczonym przez filtr Kalmana")
plt.legend()
plt.show()