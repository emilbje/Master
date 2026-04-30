from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import pandas as pd


BASELINE_SAMPLES = 1
OUTPUT_PATH = Path(r"C:\Users\Emil\Documents\Python_scripts\ski_press_test_comparison.png")


@dataclass(frozen=True)
class TestSpec:
    path: Path
    speed_label: str
    repeat_label: str
    color: str
    linestyle: str


TEST_SPECS = [
    TestSpec(
        path=Path(r"C:\Users\Emil\Downloads\test_100__1.csv"),
        speed_label="100",
        repeat_label="Test 1",
        color="#1f77b4",
        linestyle="-",
    ),
    TestSpec(
        path=Path(r"C:\Users\Emil\Downloads\test_100_2.csv"),
        speed_label="100",
        repeat_label="Test 2",
        color="#1f77b4",
        linestyle="--",
    ),
    TestSpec(
        path=Path(r"C:\Users\Emil\Downloads\test_500_1.csv"),
        speed_label="500",
        repeat_label="Test 1",
        color="#ff7f0e",
        linestyle="-",
    ),
    TestSpec(
        path=Path(r"C:\Users\Emil\Downloads\test_500_2.csv"),
        speed_label="500",
        repeat_label="Test 2",
        color="#ff7f0e",
        linestyle="--",
    ),
    TestSpec(
        path=Path(r"C:\Users\Emil\Downloads\test_1000_1.csv"),
        speed_label="1000",
        repeat_label="Test 1",
        color="#2ca02c",
        linestyle="-",
    ),
    TestSpec(
        path=Path(r"C:\Users\Emil\Downloads\test_1000_2.csv"),
        speed_label="1000",
        repeat_label="Test 2",
        color="#2ca02c",
        linestyle="--",
    ),
]


def load_test_data(spec: TestSpec) -> pd.DataFrame:
    df = pd.read_csv(spec.path, sep=";")
    if df.empty:
        raise ValueError("CSV-filen inneholder ingen datarader.")

    numeric_cols = ["force_kg", "rel_defl_mm", *[f"imu{i}" for i in range(8)]]
    for col in numeric_cols:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    df = df.dropna(subset=["rel_defl_mm"]).copy()
    if df.empty:
        raise ValueError("Fant ingen gyldige nedpressverdier i filen.")

    df["downpress_mm"] = df["rel_defl_mm"].abs()

    for idx in range(8):
        col = f"imu{idx}"
        baseline = df[col].iloc[:BASELINE_SAMPLES].mean()
        df[f"{col}_delta"] = df[col] - baseline

    return df.sort_values("downpress_mm")


def plot_comparison() -> tuple[Path, list[str], list[tuple[str, float, float, int]]]:
    fig, axes = plt.subplots(3, 3, figsize=(18, 12), sharex=True, constrained_layout=True)
    flat_axes = axes.flatten()
    force_ax = flat_axes[0]
    imu_axes = flat_axes[1:]

    skipped: list[str] = []
    summaries: list[tuple[str, float, float, int]] = []

    for spec in TEST_SPECS:
        try:
            df = load_test_data(spec)
        except Exception as exc:
            skipped.append(f"{spec.path.name}: {exc}")
            continue

        label = f"{spec.speed_label} - {spec.repeat_label}"
        force_ax.plot(
            df["downpress_mm"],
            df["force_kg"],
            color=spec.color,
            linestyle=spec.linestyle,
            linewidth=2.2,
            label=label,
        )

        for idx, ax in enumerate(imu_axes):
            ax.plot(
                df["downpress_mm"],
                df[f"imu{idx}_delta"],
                color=spec.color,
                linestyle=spec.linestyle,
                linewidth=1.9,
            )

        summaries.append(
            (
                label,
                float(df["downpress_mm"].max()),
                float(df["force_kg"].max()),
                int(len(df)),
            )
        )

    force_ax.set_title("Lastcelle")
    force_ax.set_ylabel("Kraft [kg]")
    force_ax.legend(loc="upper left", fontsize=9, frameon=True)

    for idx, ax in enumerate(imu_axes):
        ax.set_title(f"IMU {idx}")
        ax.set_ylabel("Delta pitch [deg]")

    for ax in flat_axes:
        ax.grid(True, alpha=0.25)
        ax.set_xlim(left=0)

    for ax in axes[-1]:
        ax.set_xlabel("Nedpress [mm]")

    fig.suptitle(
        "Sammenligning av nedpresstester\nFarge = hastighet, linjestil = repetisjon",
        fontsize=16,
        fontweight="bold",
    )

    fig.savefig(OUTPUT_PATH, dpi=180, bbox_inches="tight")
    plt.close(fig)
    return OUTPUT_PATH, skipped, summaries


if __name__ == "__main__":
    output_path, skipped_files, summaries = plot_comparison()
    print(f"Laget figur: {output_path}")
    if skipped_files:
        print("Hoppet over:")
        for item in skipped_files:
            print(f"  - {item}")
    if summaries:
        print("Oppsummering:")
        for label, max_downpress, max_force, points in summaries:
            print(
                f"  - {label}: {points} punkter, maks nedpress {max_downpress:.2f} mm, "
                f"maks kraft {max_force:.2f} kg"
            )
