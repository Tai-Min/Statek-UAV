import csv
import numpy as np

def open_csv(file_path):
    header = True

    lat = []
    lon = []

    with open(file_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if not header:
                lat.append(float(row[2]))
                lon.append(float(row[3]))

            header = False

    lat = np.array(lat)
    lon = np.array(lon)

    return {"lat": lat, "lon": lon}