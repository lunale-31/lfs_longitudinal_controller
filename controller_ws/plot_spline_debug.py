#!/usr/bin/env python3

from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


CSV_PATH = Path("spline_debug.csv")


def main() -> None:
    if not CSV_PATH.exists():
        raise FileNotFoundError(
            f"CSV file was not found: {CSV_PATH}"
        )

    data = pd.read_csv(CSV_PATH)

    control_points = data[
        data["PointType"] == "control"
    ]

    spline_points = data[
        data["PointType"] == "spline"
    ]

    if control_points.empty:
        raise ValueError(
            "No control points were found in the CSV."
        )

    if spline_points.empty:
        raise ValueError(
            "No sampled spline points were found in the CSV."
        )

    plt.figure(figsize=(10, 8))

    plt.plot(
        spline_points["X"],
        spline_points["Y"],
        label="Cubic B-spline"
    )

    plt.scatter(
        control_points["X"],
        control_points["Y"],
        s=18,
        label="Original 98 center points"
    )

    # Highlight the start of the sampled spline
    plt.scatter(
        spline_points["X"].iloc[0],
        spline_points["Y"].iloc[0],
        s=80,
        marker="x",
        label="Spline t = 0"
    )

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Looping Cubic B-spline and Track Center Points")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.show()


if __name__ == "__main__":
    main()