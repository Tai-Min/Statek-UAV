import numpy as np
def normalize(gps_struct):
    lat_mean = np.mean(gps_struct["lat"])
    lon_mean = np.mean(gps_struct["lon"])

    lat = gps_struct["lat"] - lat_mean
    lon = gps_struct["lon"] - lon_mean
    
    return {"lat": lat, "lon": lon}