
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from scipy.interpolate import CubicHermiteSpline

# ============================================================
# INNSTILLINGER DU KAN ENDRE
# ============================================================

# Filnavn
FILE_PATH = "LOG00026.TXT"

# IMU-posisjoner langs skien [meter fra tupp]
# Endre disse hvis sensorene dine står andre steder
IMU_X = np.array([0.10, 0.25, 0.42, 0.60, 0.88, 1.34, 1.59, 1.78], dtype=float)

# Velg om x-aksen skal vises fra tupp (0 -> skilengde)
# eller sentrert rundt midten av skien
CENTER_X_AXIS = False

# Pitch-kolonnene i fila er r0-r7 i denne loggen
PITCH_COLS = [f"r{i}" for i in range(8)]

# Hvis pitch må inverteres i visualiseringen, sett til True
INVERT_PITCH = False

# Dersom du vil glatte litt over tid
SMOOTHING_WINDOW = 1   # 1 = ingen glatting

# ============================================================
# FILLESING
# ============================================================

def load_log_file(path):
    """
    Leser loggfil som inneholder litt tekst i starten og deretter tab-separerte data.
    Finner alle linjer som starter med et tall (time_ms), og leser disse som data.
    """
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    data_lines = []
    for line in lines:
        s = line.strip()
        # Ta kun med linjer som starter med heltall og inneholder tab
        if re.match(r'^\d+\t', s):
            data_lines.append(s)

    if not data_lines:
        raise ValueError("Fant ingen datalinjer i fila.")

    header = ["time_ms"] + [f"p{i}" for i in range(8)] + [f"r{i}" for i in range(8)]
    rows = [row.split("\t") for row in data_lines]

    df = pd.DataFrame(rows, columns=header)
    df = df.apply(pd.to_numeric, errors="coerce")
    df = df.dropna().reset_index(drop=True)

    if SMOOTHING_WINDOW > 1:
        value_cols = [c for c in df.columns if c != "time_ms"]
        df[value_cols] = df[value_cols].rolling(
            window=SMOOTHING_WINDOW, center=True, min_periods=1
        ).mean()

    return df


# ============================================================
# GEOMETRI / REKONSTRUKSJON
# ============================================================

def reconstruct_deflection(x, pitch_deg, flatten_endpoints=True):
    """
    Lager y-verdier i IMU-punktene ved å integrere lokal helning.

    pitch_deg -> lokal vinkel i grader
    dy/dx = tan(theta)

    Etter integrasjon kan vi fjerne rigid-body trend slik at endepunktene havner på samme nivå.
    """
    pitch_deg = np.asarray(pitch_deg, dtype=float)
    if INVERT_PITCH:
        pitch_deg = -pitch_deg

    slopes = np.tan(np.deg2rad(pitch_deg))

    y = np.zeros_like(x, dtype=float)
    for i in range(1, len(x)):
        dx = x[i] - x[i - 1]
        y[i] = y[i - 1] + 0.5 * (slopes[i - 1] + slopes[i]) * dx

    if flatten_endpoints:
        # Fjern lineær rigid-body komponent slik at første og siste punkt får samme høyde
        line = np.linspace(y[0], y[-1], len(y))
        y = y - line

    return y, slopes


def build_curve(x, y, slopes):
    """
    Lager en glatt kurve som går gjennom punktene og har gitt helning i hvert punkt.
    Dette er bedre enn en vanlig parabel når du har mange IMU-er.
    """
    spline = CubicHermiteSpline(x, y, slopes)
    xs = np.linspace(x.min(), x.max(), 500)
    ys = spline(xs)
    return xs, ys, spline


# ============================================================
# PLOTT
# ============================================================

def main():
    df = load_log_file(FILE_PATH)

    if len(IMU_X) != 8:
        raise ValueError("IMU_X må inneholde 8 posisjoner.")

    x = IMU_X.copy()
    if CENTER_X_AXIS:
        x = x - 0.5 * (x[0] + x[-1])

    t0 = df["time_ms"].iloc[0]
    time_s = (df["time_ms"].to_numpy() - t0) / 1000.0

    fig, ax = plt.subplots(figsize=(11, 6))
    plt.subplots_adjust(bottom=0.22)

    idx0 = 0
    pitch0 = df.loc[idx0, PITCH_COLS].to_numpy(dtype=float)
    y0, slopes0 = reconstruct_deflection(x, pitch0, flatten_endpoints=True)
    xs, ys, spline = build_curve(x, y0, slopes0)

    line_curve, = ax.plot(xs, ys * 1000, lw=2, label="Rekonstruert ski-profil")
    points, = ax.plot(x, y0 * 1000, "o", ms=8, label="IMU-punkter")

    # Små tangentlinjer i hvert IMU-punkt for å vise lokal pitch
    tangent_half_len = 0.04  # meter
    tangent_lines = []
    for xi, yi, mi in zip(x, y0, slopes0):
        xt = np.array([xi - tangent_half_len, xi + tangent_half_len])
        yt = yi + mi * (xt - xi)
        lt, = ax.plot(xt, yt * 1000, "--", lw=1)
        tangent_lines.append(lt)

    title = ax.set_title(f"Tid: {time_s[idx0]:.3f} s")
    ax.set_xlabel("Posisjon langs ski [m]")
    ax.set_ylabel("Deflection [mm]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    # Fast akseområde basert på hele datasettet
    all_y = []
    for i in range(len(df)):
        pitch_i = df.loc[i, PITCH_COLS].to_numpy(dtype=float)
        yi, mi = reconstruct_deflection(x, pitch_i, flatten_endpoints=True)
        _, ys_i, _ = build_curve(x, yi, mi)
        all_y.append(ys_i)
    all_y = np.concatenate(all_y) * 1000.0

    ypad = max(5.0, 0.1 * (all_y.max() - all_y.min() + 1e-9))
    ax.set_xlim(x.min() - 0.05, x.max() + 0.05)
    ax.set_ylim(all_y.min() - ypad, all_y.max() + ypad)

    # Slider
    slider_ax = plt.axes([0.12, 0.08, 0.76, 0.04])
    slider = Slider(
        ax=slider_ax,
        label="Frame",
        valmin=0,
        valmax=len(df) - 1,
        valinit=idx0,
        valstep=1
    )

    def update(val):
        idx = int(slider.val)

        pitch = df.loc[idx, PITCH_COLS].to_numpy(dtype=float)
        y, slopes = reconstruct_deflection(x, pitch, flatten_endpoints=True)
        xs, ys, spline = build_curve(x, y, slopes)

        line_curve.set_data(xs, ys * 1000)
        points.set_data(x, y * 1000)

        for lt, xi, yi, mi in zip(tangent_lines, x, y, slopes):
            xt = np.array([xi - tangent_half_len, xi + tangent_half_len])
            yt = yi + mi * (xt - xi)
            lt.set_data(xt, yt * 1000)

        title.set_text(
            f"Tid: {time_s[idx]:.3f} s | raw time_ms: {int(df.loc[idx, 'time_ms'])}"
        )
        fig.canvas.draw_idle()

    slider.on_changed(update)
    plt.show()


if __name__ == "__main__":
    main()
