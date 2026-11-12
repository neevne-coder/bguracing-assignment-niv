import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("data/BrandsHatchLayout.csv")
print("Raw shape:", df.shape)
print(df.head())

df = df.dropna()
print("Shape after dropna:", df.shape)

left = df[df["side"] == "left"]
right = df[df["side"] == "right"]
print("Left points:", len(left), "| Right points:", len(right))


def polyline_length(side_df):
    dx = np.diff(side_df["x"].to_numpy())
    dy = np.diff(side_df["y"].to_numpy())
    return np.hypot(dx, dy).sum()


for name, side_df in [("left", left), ("right", right)]:
    print(f"{name}: length = {polyline_length(side_df):.1f} m, "
          f"x in [{side_df['x'].min():.1f}, {side_df['x'].max():.1f}], "
          f"y in [{side_df['y'].min():.1f}, {side_df['y'].max():.1f}]")

plt.figure(figsize=(8, 8))
plt.plot(left["x"], left["y"], label="left boundary")
plt.plot(right["x"], right["y"], label="right boundary")
plt.axis("equal")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Brands Hatch layout - raw data")
plt.legend()
plt.savefig("output/track_raw.png", dpi=150)