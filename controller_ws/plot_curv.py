import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("Bspline_velocity_profile_analysis.csv")

s = data["Cumulative_S"]
curvature = data["Curvature"]
v_corner = data["Velocity_Corner"]

plt.figure(figsize=(14, 5))
plt.plot(s, curvature.abs())
plt.xlabel("Cumulative track distance [m]")
plt.ylabel("Absolute curvature [1/m]")
plt.title("Absolute spline curvature")
plt.grid(True)
plt.tight_layout()
plt.show()

plt.figure(figsize=(14, 5))
plt.plot(s, v_corner)
plt.xlabel("Cumulative track distance [m]")
plt.ylabel("Cornering velocity [m/s]")
plt.title("Cornering velocity limit")
plt.grid(True)
plt.tight_layout()
plt.show()