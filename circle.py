import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle

# Generujeme náhodné body v okolí (2D shluk)
np.random.seed(0)
points = np.random.randn(100, 2)

# Výpočet středu (např. průměrný bod)
center = points.mean(axis=0)

# Poloměr – např. maximální vzdálenost od středu
radii = np.linalg.norm(points - center, axis=1)
radius = np.max(radii)

# Vykreslení bodů a kružnice
fig, ax = plt.subplots()
ax.scatter(points[:, 0], points[:, 1])
circle = Circle(center, radius, fill=False, edgecolor='r', linestyle='--')
ax.add_patch(circle)
ax.set_aspect('equal')
plt.title('Kružnice kolem shluku bodů')
plt.show()
