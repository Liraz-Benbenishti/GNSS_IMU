import argparse
import pandas as pd
import numpy as np
from datetime import datetime, timedelta

# =========================
# Constants
# =========================
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3
GPS_UTC_LEAP_SECONDS = 18  # valid for 2026

# =========================
# Time conversion
# =========================
def gpst_string_to_utc_millis(gpst_str):
    """
    GPST string: YYYY/MM/DD HH:MM:SS.sss
    Converts GPS Time -> UTC milliseconds
    """
    gpst = datetime.strptime(gpst_str, "%Y/%m/%d %H:%M:%S.%f")
    utc = gpst - timedelta(seconds=GPS_UTC_LEAP_SECONDS)
    return int(utc.timestamp() * 1000)

# =========================
# Coordinate conversions
# =========================
def llh_to_ecef(lat_deg, lon_deg, h):
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)

    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)

    N = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat**2)

    x = (N + h) * cos_lat * cos_lon
    y = (N + h) * cos_lat * sin_lon
    z = (N * (1 - WGS84_E2) + h) * sin_lat

    return x, y, z


def enu_to_ecef_velocity(vn, ve, vu, lat_deg, lon_deg):
    """
    Exact ENU -> ECEF velocity rotation
    """
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)

    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)

    R = np.array([
        [-sin_lon, -sin_lat * cos_lon,  cos_lat * cos_lon],
        [ cos_lon, -sin_lat * sin_lon,  cos_lat * sin_lon],
        [     0.0,           cos_lat,           sin_lat]
    ])

    v_enu = np.vstack((ve, vn, vu))
    v_ecef = R @ v_enu

    return v_ecef[0], v_ecef[1], v_ecef[2]

# =========================
# POS reader
# =========================
def read_pos_file(path):
    with open(path, "r") as f:
        lines = f.readlines()

    start_idx = None
    for i, line in enumerate(lines):
        if line.strip() and line.strip()[0].isdigit():
            start_idx = i
            break

    if start_idx is None:
        raise RuntimeError("No data rows found")

    return pd.read_csv(
        path,
        delim_whitespace=True,
        skiprows=start_idx
    )



# Simple outlier detection and interpolation
def exclude_interpolate_outlier(df):
    x_wls = df[[
        "x_ecef",
        "y_ecef",
        "z_ecef"
    ]].to_numpy()
    v_wls = df[[
        "vx_ecef",
        "vy_ecef",
        "vz_ecef"
    ]].to_numpy()
    # Up velocity threshold
    v_up_th = 2.0 # m/s

    # Coordinate conversion
    x_llh = np.array(pm.ecef2geodetic(x_wls[:, 0], x_wls[:, 1], x_wls[:, 2])).T
    v_enu = np.array(pm.ecef2enuv(
        v_wls[:, 0], v_wls[:, 1], v_wls[:, 2], x_llh[0, 0], x_llh[0, 1])).T

    # Up velocity jump detection
    # Cars don't jump suddenly!
    idx_v_out = np.abs(v_enu[:, 2]) > v_up_th
    v_wls[idx_v_out, :] = np.nan
    
    # Interpolate all NaN data
    v_df = pd.DataFrame({'x': v_wls[:, 0], 'y': v_wls[:, 1], 'z': v_wls[:, 2]})
    v_df = v_df.interpolate(limit_area='outside', limit_direction='both')
    v_df = v_df.interpolate('spline', order=3)

    df[["vx_ecef", "vy_ecef", "vz_ecef"]] = v_df.values
    return df


# =========================
# Kalman placeholder
# =========================
# Kalman filter
def Kalman_filter(zs, us, phone):
    # Parameters
    # I don't know why only XiaomiMi8 seems to be inaccurate ... 
    sigma_v = 0.6 if phone == 'XiaomiMi8' else 0.1 # velocity SD m/s
    sigma_x = 5.0  # position SD m
    sigma_mahalanobis = 30.0 # Mahalanobis distance for rejecting innovation
    
    n, dim_x = zs.shape
    F = np.eye(3)  # Transition matrix
    Q = sigma_v**2 * np.eye(3)  # Process noise

    H = np.eye(3)  # Measurement function
    R = sigma_x**2 * np.eye(3)  # Measurement noise

    # Initial state and covariance
    x = zs[0, :3].T  # State
    P = sigma_x**2 * np.eye(3)  # State covariance
    I = np.eye(dim_x)

    x_kf = np.zeros([n, dim_x])
    P_kf = np.zeros([n, dim_x, dim_x])
    is_outlier_measurement = np.zeros(n, dtype=np.uint8)

    # Kalman filtering
    for i, (u, z) in enumerate(zip(us, zs)):
        # First step
        if i == 0:
            x_kf[i] = x.T
            P_kf[i] = P
            continue

        # Prediction step
        x = F @ x + u.T
        P = (F @ P) @ F.T + Q

        # Check outliers for observation
        d = distance.mahalanobis(z, H @ x, np.linalg.pinv(P))

        # Update step
        if d < sigma_mahalanobis:
            y = z.T - H @ x
            S = (H @ P) @ H.T + R
            K = (P @ H.T) @ np.linalg.inv(S)
            x = x + K @ y
            P = (I - (K @ H)) @ P
        else:
            # If no observation update is available, increase covariance
            P += 10**2*Q
            is_outlier_measurement[i] = 1

        x_kf[i] = x.T
        P_kf[i] = P

    return x_kf, P_kf, is_outlier_measurement


# Forward + backward Kalman filter and smoothing
def kalman_smoothing(df):
    phone = "BULLSHIT"  # Placeholder phone model
    x_wls = df[["x_ecef", "y_ecef", "z_ecef"]].to_numpy()
    v_wls = df[["vx_ecef", "vy_ecef", "vz_ecef"]].to_numpy()

    n, dim_x = x_wls.shape

    # Forward
    v = np.vstack([np.zeros([1, 3]), (v_wls[:-1, :] + v_wls[1:, :])/2])
    x_f, P_f, is_outlier_f = Kalman_filter(x_wls, v, phone)

    # Backward
    v = -np.flipud(v_wls)
    v = np.vstack([np.zeros([1, 3]), (v[:-1, :] + v[1:, :])/2])
    x_b, P_b, is_outlier_b = Kalman_filter(np.flipud(x_wls), v, phone)

    # Smoothing
    x_fb = np.zeros_like(x_f)
    P_fb = np.zeros_like(P_f)
    for (f, b) in zip(range(n), range(n-1, -1, -1)):
        P_fi = np.linalg.inv(P_f[f])
        P_bi = np.linalg.inv(P_b[b])

        P_fb[f] = np.linalg.inv(P_fi + P_bi)
        x_fb[f] = P_fb[f] @ (P_fi @ x_f[f] + P_bi @ x_b[b])

    summarize_outliers = is_outlier_f + np.flipud(is_outlier_b)
    return x_fb, x_f, np.flipud(x_b), summarize_outliers

# =========================
# ECEF velocity → ENU velocity
# =========================
def ecef_vel_to_enu(vx, vy, vz, lat0, lon0):
    lat0 = np.deg2rad(lat0)
    lon0 = np.deg2rad(lon0)

    sin_lat = np.sin(lat0)
    cos_lat = np.cos(lat0)
    sin_lon = np.sin(lon0)
    cos_lon = np.cos(lon0)

    R = np.array([
        [-sin_lon,  cos_lon, 0],
        [-sin_lat*cos_lon, -sin_lat*sin_lon, cos_lat],
        [ cos_lat*cos_lon,  cos_lat*sin_lon, sin_lat]
    ])

    v_enu = R @ np.vstack((vx, vy, vz))
    return v_enu[0], v_enu[1]  # East, North


# =========================
# MODIFY Dash callback
# =========================
def run_dash_enu_plot_with_velocity(
    x_f, v_f,
    x_b, v_b,
    x_fb, v_fb,
    lat0, lon0, h0,
    summarize_outliers,
    vel_scale=5.0
):

    ef, nf = ecef_to_enu(x_f[:,0], x_f[:,1], x_f[:,2], lat0, lon0, h0)
    eb, nb = ecef_to_enu(x_b[:,0], x_b[:,1], x_b[:,2], lat0, lon0, h0)
    es, ns = ecef_to_enu(x_fb[:,0], x_fb[:,1], x_fb[:,2], lat0, lon0, h0)

    vf_e, vf_n = ecef_vel_to_enu(v_f[:,0], v_f[:,1], v_f[:,2], lat0, lon0)
    vb_e, vb_n = ecef_vel_to_enu(v_b[:,0], v_b[:,1], v_b[:,2], lat0, lon0)
    vs_e, vs_n = ecef_vel_to_enu(v_fb[:,0], v_fb[:,1], v_fb[:,2], lat0, lon0)

    colors = np.where(
        summarize_outliers == 0, "green",
        np.where(summarize_outliers == 1, "yellow", "red")
    )

    N = len(es)

    app = Dash(__name__)
    app.layout = html.Div([dcc.Graph(id="enu-graph", style={"height": "95vh"})])

    @app.callback(
        Output("enu-graph", "figure"),
        Input("enu-graph", "hoverData")
    )
    def update_plot(hoverData):
        sizes_f = np.full(N, 4)
        sizes_b = np.full(N, 4)
        sizes_s = np.full(N, 6)

        fig = go.Figure()

        hover_idx = None
        if hoverData:
            hover_idx = hoverData["points"][0]["customdata"]
            sizes_f[hover_idx] = 14
            sizes_b[hover_idx] = 14
            sizes_s[hover_idx] = 18

        # Position points
        fig.add_trace(go.Scatter(
            x=ef, y=nf, mode="markers",
            marker=dict(size=sizes_f, color="blue"),
            name="Forward", customdata=np.arange(N)
        ))

        fig.add_trace(go.Scatter(
            x=eb, y=nb, mode="markers",
            marker=dict(size=sizes_b, color="orange"),
            name="Backward", customdata=np.arange(N)
        ))

        fig.add_trace(go.Scatter(
            x=es, y=ns, mode="markers",
            marker=dict(size=sizes_s, color=colors),
            name="Smoothed", customdata=np.arange(N)
        ))

        # Velocity vectors (only on hover)
        if hover_idx is not None:
            for (x0, y0, vx, vy, name, col) in [
                (ef[hover_idx], nf[hover_idx], vf_e[hover_idx], vf_n[hover_idx], "Vf", "blue"),
                (eb[hover_idx], nb[hover_idx], vb_e[hover_idx], vb_n[hover_idx], "Vb", "orange"),
                (es[hover_idx], ns[hover_idx], vs_e[hover_idx], vs_n[hover_idx], "Vs", "black")
            ]:
                fig.add_trace(go.Scatter(
                    x=[x0, x0 + vel_scale * vx],
                    y=[y0, y0 + vel_scale * vy],
                    mode="lines+markers",
                    marker=dict(size=6),
                    line=dict(width=4),
                    name=name,
                    showlegend=False
                ))

        fig.update_layout(
            title="ENU Kalman smoothing with linked velocity vectors",
            xaxis_title="East [m]",
            yaxis_title="North [m]",
            xaxis=dict(scaleanchor="y", scaleratio=1),
            hovermode="closest"
        )

        return fig

    app.run_server(debug=False, port=8050, open_browser=True)


# =========================
# Main
# =========================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pos_file", help="Path to POS file")
    args = parser.parse_args()

    df = read_pos_file(args.pos_file)

    df = df[[
        "GPST",
        "latitude(deg)",
        "longitude(deg)",
        "height(m)",
        "vn(m/s)",
        "ve(m/s)",
        "vu(m/s)"
    ]].copy()

    df["utcMillis"] = df["GPST"].apply(gpst_string_to_utc_millis)

    x, y, z = llh_to_ecef(
        df["latitude(deg)"].values,
        df["longitude(deg)"].values,
        df["height(m)"].values
    )

    df["x_ecef"] = x
    df["y_ecef"] = y
    df["z_ecef"] = z

    vx, vy, vz = enu_to_ecef_velocity(
        df["vn(m/s)"].values,
        df["ve(m/s)"].values,
        df["vu(m/s)"].values,
        df["latitude(deg)"].values,
        df["longitude(deg)"].values
    )

    df["vx_ecef"] = vx
    df["vy_ecef"] = vy
    df["vz_ecef"] = vz

    df = df[[
        "utcMillis",
        "x_ecef", "y_ecef", "z_ecef",
        "vx_ecef", "vy_ecef", "vz_ecef"
    ]]

    df = exclude_interpolate_outlier(df)
    df_out = kalman_smoothing(df)
    print(df_out.head())

    lat0, lon0, h0 = ecef_to_llh(x_fb[0,0], x_fb[0,1], x_fb[0,2])

    run_dash_enu_plot_with_velocity(
        x_f,  v_f,
        x_b,  v_b,
        x_fb, v_fb,
        lat0, lon0, h0,
        summarize_outliers,
        vel_scale=5.0   # meters per (m/s)
    )


if __name__ == "__main__":
    main()
