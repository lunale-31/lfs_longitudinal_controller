import matplotlib.pyplot as plt
import pandas as pd

# Load data
df = pd.read_csv("v_profile.csv")

plt.figure(figsize=(14, 7))

# Plot baseline constraints as light/dashed lines
plt.plot(df["Index"], df["Velocity_Corner"], color="orange", linestyle="--", alpha=0.6, label="Cornering Limit (V_curve)")
plt.plot(df["Index"], df["Velocity_Accel"], color="blue", linestyle=":", alpha=0.7, label="Forward Accel Pass")
plt.plot(df["Index"], df["Velocity_Brake"], color="purple", linestyle=":", alpha=0.7, label="Backward Brake Pass")

# Plot final chosen profile as a thick solid line
plt.plot(df["Index"], df["Velocity_Profile"], color="red", linewidth=2.5, label="Final Velocity Profile (Min Envelope)")

plt.title("Vehicle Profile Trajectory Decomposition Analysis")
plt.xlabel("Track Point Index")
plt.ylabel("Velocity [m/s]")
plt.grid(True, linestyle="--", alpha=0.5)
plt.legend(loc="upper right")

plt.savefig("v_profile_decomposed.png", dpi=300)
plt.show()