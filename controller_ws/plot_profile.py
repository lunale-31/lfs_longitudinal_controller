import matplotlib.pyplot as plt
import pandas as pd

# Load profile analysis data
df = pd.read_csv("Bspline_velocity_profile_analysis.csv")

# Remove rows without segment calculations.
# The final appended row only closes the velocity plot.
segment_df = df.dropna(subset=["Segment_Acceleration"]).copy()

# ============================================================
# 1. Velocity profile against cumulative track distance
# ============================================================

plt.figure(figsize=(14, 7))

plt.plot(
    df["Cumulative_S"],
    df["Velocity_Corner"],
    color="orange",
    linestyle="--",
    alpha=0.6,
    label="Cornering Limit"
)

plt.plot(
    df["Cumulative_S"],
    df["Velocity_Accel"],
    color="blue",
    linestyle=":",
    alpha=0.7,
    label="Forward Acceleration Pass"
)

plt.plot(
    df["Cumulative_S"],
    df["Velocity_Brake"],
    color="purple",
    linestyle=":",
    alpha=0.7,
    label="Backward Braking Pass"
)

plt.plot(
    df["Cumulative_S"],
    df["Velocity_Profile"],
    color="red",
    linewidth=2.5,
    label="Final Velocity Profile"
)

plt.title("Velocity Profile Decomposition")
plt.xlabel("Cumulative Track Distance [m]")
plt.ylabel("Velocity [m/s]")
plt.grid(True, linestyle="--", alpha=0.5)
plt.legend(loc="upper right")
plt.tight_layout()

plt.savefig(
    "Bspline_velocity32_profile_vs_distance.png",
    dpi=300
)

plt.show()


# ============================================================
# 2. Required segment acceleration and available limits
# ============================================================

plt.figure(figsize=(14, 7))

# Signed acceleration required by the final velocity profile
plt.plot(
    segment_df["Cumulative_S"],
    segment_df["Segment_Acceleration"],
    color="black",
    linewidth=2.0,
    label="Required Segment Acceleration"
)

# Positive acceleration limit
plt.plot(
    segment_df["Cumulative_S"],
    segment_df["Accel_Limit"],
    color="blue",
    linestyle="--",
    alpha=0.8,
    label="Available Acceleration Limit"
)

# Plot braking limit as negative because braking acceleration is negative
plt.plot(
    segment_df["Cumulative_S"],
    -segment_df["Brake_Limit"],
    color="purple",
    linestyle="--",
    alpha=0.8,
    label="Available Braking Limit"
)

# Mark infeasible segments
infeasible_df = segment_df[
    segment_df["Segment_Feasible"] == 0
]

if not infeasible_df.empty:
    plt.scatter(
        infeasible_df["Cumulative_S"],
        infeasible_df["Segment_Acceleration"],
        color="red",
        marker="x",
        s=80,
        linewidths=2,
        label="Infeasible Segment"
    )

plt.axhline(
    y=0.0,
    color="gray",
    linewidth=1.0
)

plt.title("Segment Acceleration Feasibility")
plt.xlabel("Cumulative Track Distance [m]")
plt.ylabel("Longitudinal Acceleration [m/s²]")
plt.grid(True, linestyle="--", alpha=0.5)
plt.legend(loc="upper right")
plt.tight_layout()

plt.savefig(
    "Bspline_velocity32_profile_acceleration_feasibility.png",
    dpi=300
)

plt.show()


# ============================================================
# 3. Print numerical validation summary
# ============================================================

number_of_segments = len(segment_df)

infeasible_segments = segment_df[
    segment_df["Segment_Feasible"] == 0
]

corner_violations = segment_df[
    segment_df["Corner_Feasible"] == 0
]

print("\n--- Velocity Profile Validation ---")
print(f"Number of segments: {number_of_segments}")
print(f"Infeasible acceleration segments: {len(infeasible_segments)}")
print(f"Corner-limit violations: {len(corner_violations)}")

print(
    "Maximum required acceleration: "
    f"{segment_df['Segment_Acceleration'].max():.3f} m/s²"
)

print(
    "Maximum required braking magnitude: "
    f"{abs(segment_df['Segment_Acceleration'].min()):.3f} m/s²"
)

if infeasible_segments.empty:
    print("All segments satisfy the calculated acceleration and braking limits.")
else:
    print("\nInfeasible segments:")
    print(
        infeasible_segments[
            [
                "Index",
                "Cumulative_S",
                "Delta_S_To_Next",
                "Segment_Acceleration",
                "Accel_Limit",
                "Brake_Limit"
            ]
        ].to_string(index=False)
    )

if not corner_violations.empty:
    print("\nCorner-limit violations:")
    print(
        corner_violations[
            [
                "Index",
                "Cumulative_S",
                "Velocity_Profile",
                "Velocity_Corner"
            ]
        ].to_string(index=False)
    )