from normalize import normalize
import matplotlib.pyplot as plt
from open_csv import open_csv

room = normalize(open_csv("./gps_data_room.csv"))
park = normalize(open_csv("./gps_data_park.csv"))
pub = normalize(open_csv("./gps_data_pub.csv"))
park_filtered = normalize(open_csv("./filtered.csv"))

plt.scatter(room["lat"], room["lon"], marker='o', label="pomieszczenie")
plt.scatter(park["lat"], park["lon"], marker='s', label="park")
plt.scatter(pub["lat"], pub["lon"], marker='X', label="pusty plac")
plt.xlabel("Szerokość geograficzna [°]")
plt.ylabel("Długość geograficzna [°]")
plt.title("Rozrzut koordynatów GPS w środowiskach z różną widocznością satelit")
plt.legend()
plt.show()