import yaml
import numpy as np

# Mock parameters
resolution = 0.1
dilate = 0.26
x_min, x_max = -2.0, 12.0
y_min, y_max = -2.0, 12.0

w = int((x_max - x_min) / resolution)
h = int((y_max - y_min) / resolution)
grid = np.zeros((h, w), dtype=int)

walls = [
    (4.3, 1.25, 5.6, 0.1),
    (5.0, -1.25, 6.2, 0.1),
    (6.0, 3.75, 5.3, 0.1),
    (11.25, 1.25, 0.1, 2.5),
    (11.25, 9.0, 0.1, 3.0),
    (6.25, 5.8, 0.1, 2.2),
    (0.75, 7.25, 0.1, 3.5),
    (-1.25, 6.75, 0.1, 5.5),
    (5.0, 12.0, 6.3, 0.1),
    (5.7, 8.0, 3.7, 0.1)
]

for wx, wy, wsx, wsy in walls:
    l, r = wx - wsx - dilate, wx + wsx + dilate
    b, t = wy - wsy - dilate, wy + wsy + dilate
    
    idx_l = max(0, int((l - x_min) / resolution))
    idx_r = min(w, int((r - x_min) / resolution))
    idx_b = max(0, int((b - y_min) / resolution))
    idx_t = min(h, int((t - y_min) / resolution))
    
    grid[idx_b:idx_t, idx_l:idx_r] = 1

print("Map:")
for y in range(h-1, -1, -2):  # Compress Y slightly for console output
    row = ""
    for x in range(0, w, 2):
        if x == 100 and y <= 50 and y >= 40: # roughly goal
            pass
        row += "X" if grid[y, x] else "."
    print(f"{y*resolution+y_min:5.1f} {row}")
