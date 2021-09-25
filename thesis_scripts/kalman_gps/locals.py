import math

a = 6378137.0
b = 6356752.3142
eSq = 0.00669437999


def geo_to_ecef(lat, lon):
    phi_val = lat * math.pi / 180.0
    lambda_val = lon * math.pi / 180.0
    N = a / math.sqrt(1.0 - eSq * math.sin(phi_val)**2)

    ecef_x = N * math.sin(phi_val) * math.cos(lambda_val)
    ecef_y = N * math.cos(phi_val) * math.sin(lambda_val)
    ecef_z = (b**2) / (a**2) * N * math.sin(phi_val)

    return ecef_x, ecef_y, ecef_z


def ecef_to_enu(ecef_x, ecef_y, ecef_z, origin_lat, origin_lon):
    ecef_x_origin, ecef_y_origin, ecef_z_origin = geo_to_ecef(
        origin_lat, origin_lon)

    d_ecef_x = ecef_x - ecef_x_origin
    d_ecef_y = ecef_y - ecef_y_origin
    d_ecef_z = ecef_z - ecef_z_origin

    phi_val = origin_lat * math.pi / 180.0
    lambda_val = origin_lon * math.pi / 180.0

    enu_x = -math.sin(lambda_val) * d_ecef_x + math.cos(lambda_val) * d_ecef_y
    enu_y = -math.sin(phi_val) * math.cos(lambda_val) * d_ecef_x - math.sin(
        phi_val) * math.sin(lambda_val) * d_ecef_y + math.cos(phi_val) * d_ecef_z
    enu_z = math.cos(phi_val) * math.cos(lambda_val) * d_ecef_x + math.cos(phi_val) * \
        math.sin(lambda_val) * d_ecef_y + math.sin(phi_val) * d_ecef_z

    return enu_x, enu_y, enu_z


def geodetic_to_enu(lat, lon, origin_lat, origin_lon):
    ecef_x, ecef_y, ecef_z = geo_to_ecef(lat, lon)
    return ecef_to_enu(ecef_x, ecef_y, ecef_z, origin_lat, origin_lon)


def ecef_to_geo(ecef_x, ecef_y, ecef_z):
    w = math.hypot(ecef_x, ecef_y)
    l = eSq / 2.0
    lSq = l**2
    m = (w / a)**2
    n = (((1.0 - eSq) * ecef_z / b))**2
    i = -(2.0 * lSq + m + n) / 2.0
    k = lSq * (lSq - m - n)
    q = ((m + n - 4.0 * lSq))**3 / 216.0 + m * n * lSq
    D = math.sqrt(abs((2.0 * q - m * n * lSq) * m * n * lSq))
    beta = i / 3.0 - (q + D)**(1./3.) - (q - D)**(1./3.)
    t = math.sqrt(math.sqrt(beta**2 - k) - (beta + i) / 2.0) - \
        math.copysign(math.sqrt(abs((beta - i) / 2.0)), m - n)
    w1 = w / (t + l)
    z1 = (1.0 - eSq) * ecef_z / (t - l)

    if w != 0:
        phi = math.atan2(z1, (1.0 - eSq) * w1)
    else:
        phi = math.copysign(math.pi / 2.0, ecef_z)
    lam = 2.0 * math.atan2(w - ecef_x, ecef_y)

    lat = phi * 180.0 / math.pi
    lon = lam * 180.0 / math.pi

    return lat, lon


def enu_to_ecef(enu_x, enu_y, enu_z, origin_lat, origin_lon):
    ecef_x_origin, ecef_y_origin, ecef_z_origin = geo_to_ecef(
        origin_lat, origin_lon)

    phi_val = origin_lat * math.pi / 180.0
    lambda_val = origin_lon * math.pi / 180.0

    ecef_x = (-math.sin(lambda_val) * enu_x - math.sin(phi_val) * math.cos(lambda_val)
              * enu_y + math.cos(phi_val) * math.cos(lambda_val) * enu_z) + ecef_x_origin
    ecef_y = (math.cos(lambda_val) * enu_x - math.sin(phi_val) * math.sin(lambda_val)
              * enu_y + math.cos(phi_val) * math.sin(lambda_val) * enu_z) + ecef_y_origin
    ecef_z = (math.cos(phi_val) * enu_y +
              math.sin(phi_val) * enu_z) + ecef_z_origin

    return ecef_x, ecef_y, ecef_z


def enu_to_geodetic(enu_x, enu_y, enu_z, origin_lat, origin_lon):
    ecef_x, ecef_y, ecef_z = enu_to_ecef(
        enu_x, enu_y, enu_z, origin_lat, origin_lon)
    return ecef_to_geo(ecef_x, ecef_y, ecef_z)
