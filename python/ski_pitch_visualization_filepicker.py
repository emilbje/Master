
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from scipy.interpolate import CubicHermiteSpline
from tkinter import Tk, filedialog

# ============================================================
# INNSTILLINGER DU KAN ENDRE
# ============================================================

# IMU-posisjoner langs skien [meter fra tupp]
IMU_X = np.array([0.10, 0.25, 0.42, 0.60, 0.88, 1.34, 1.59, 1.78], dtype=float)

# Vis x-akse sentrert rundt skiens midtpunkt
CENTER_X_AXIS = False

# Pitch-kolonnene i fila
PITCH_COLS = [f"r{i}" for i in range(8)]

# Inverter pitch hvis nødvendig
INVERT_PITCH = False

# 1 = ingen glatting
SMOOTHING_WINDOW = 1

# Fjern lineær rigid-body-trend slik at første og siste punkt havner på samme nivå
FLATTEN_ENDPOINTS = True


# ============================================================
# FILVALG
# ============================================================

def choose_file():
    root = Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    file_path = filedialog.askopenfilename(
        title="Velg TXT-loggfil",
        filetypes=[("Text files", "*.txt *.TXT"), ("All files", "*.*")]
    )
    root.destroy()
    return file_path


# ============================================================
# FILLESING
# ============================================================

def load_log_file(path):
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    data_lines = []
    for line in lines:
        s = line.strip()
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
    pitch_deg = np.asarray(pitch_deg, dtype=float)
    if INVERT_PITCH:
        pitch_deg = -pitch_deg

    slopes = np.tan(np.deg2rad(pitch_deg))

    y = np.zeros_like(x, dtype=float)
    for i in range(1, len(x)):
        dx = x[i] - x[i - 1]
        y[i] = y[i - 1] + 0.5 * (slopes[i - 1] + slopes[i]) * dx

    if flatten_endpoints:
        line = np.linspace(y[0], y[-1], len(y))
        y = y - line

    return y, slopes


def build_curve(x, y, slopes):
    spline = CubicHermiteSpline(x, y, slopes)
    xs = np.linspace(x.min(), x.max(), 500)
    ys = spline(xs)
    return xs, ys, spline


# ============================================================
# PLOTT
# ============================================================

def run_visualization(file_path):
    df = load_log_file(file_path)

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
    y0, slopes0 = reconstruct_deflection(x, pitch0, flatten_endpoints=FLATTEN_ENDPOINTS)
    xs, ys, _ = build_curve(x, y0, slopes0)

    line_curve, = ax.plot(xs, ys * 1000, lw=2, label="Rekonstruert ski-profil")
    points, = ax.plot(x, y0 * 1000, "o", ms=8, label="IMU-punkter")

    tangent_half_len = 0.04
    tangent_lines = []
    for xi, yi, mi in zip(x, y0, slopes0):
        xt = np.array([xi - tangent_half_len, xi + tangent_half_len])
        yt = yi + mi * (xt - xi)
        lt, = ax.plot(xt, yt * 1000, "--", lw=1)
        tangent_lines.append(lt)

    file_name = Path(file_path).name
    title = ax.set_title(f"{file_name} | Tid: {time_s[idx0]:.3f} s")
    ax.set_xlabel("Posisjon langs ski [m]")
    ax.set_ylabel("Deflection [mm]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    all_y = []
    for i in range(len(df)):
        pitch_i = df.loc[i, PITCH_COLS].to_numpy(dtype=float)
        yi, mi = reconstruct_deflection(x, pitch_i, flatten_endpoints=FLATTEN_ENDPOINTS)
        _, ys_i, _ = build_curve(x, yi, mi)
        all_y.append(ys_i)
    all_y = np.concatenate(all_y) * 1000.0

    ypad = max(5.0, 0.1 * (all_y.max() - all_y.min() + 1e-9))
    ax.set_xlim(x.min() - 0.05, x.max() + 0.05)
    ax.set_ylim(all_y.min() - ypad, all_y.max() + ypad)

    slider_ax = plt.axes([0.12, 0.08, 0.62, 0.04])
    slider = Slider(
        ax=slider_ax,
        label="Frame",
        valmin=0,
        valmax=len(df) - 1,
        valinit=idx0,
        valstep=1
    )

    button_ax = plt.axes([0.78, 0.075, 0.12, 0.05])
    save_btn = Button(button_ax, "Velg ny fil")

    def update_plot(idx):
        pitch = df.loc[idx, PITCH_COLS].to_numpy(dtype=float)
        y, slopes = reconstruct_deflection(x, pitch, flatten_endpoints=FLATTEN_ENDPOINTS)
        xs, ys, _ = build_curve(x, y, slopes)

        line_curve.set_data(xs, ys * 1000)
        points.set_data(x, y * 1000)

        for lt, xi, yi, mi in zip(tangent_lines, x, y, slopes):
            xt = np.array([xi - tangent_half_len, xi + tangent_half_len])
            yt = yi + mi * (xt - xi)
            lt.set_data(xt, yt * 1000)

        title.set_text(f"{file_name} | Tid: {time_s[idx]:.3f} s | raw time_ms: {int(df.loc[idx, 'time_ms'])}")
        fig.canvas.draw_idle()

    def on_slider_change(val):
        idx = int(slider.val)
        update_plot(idx)

    def on_new_file(event):
        plt.close(fig)
        new_path = choose_file()
        if new_path:
            run_visualization(new_path)

    slider.on_changed(on_slider_change)
    save_btn.on_clicked(on_new_file)

    plt.show()


def main():
    file_path = choose_file()
    if not file_path:
        print("Ingen fil valgt.")
        return

    run_visualization(file_path)


if __name__ == "__main__":
    main()
