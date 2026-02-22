import numpy as np

a = 1
b = 0.1
rounds = 2
split_angle = 30

rays = [0]
i = 1

# Determine where the segment end-point rays are
while rays[-1] < rounds * 360:
    rays.append(i * split_angle)
    i += 1

# Walk along the initial spiral and put down a point every time we hit a ray
main_spiral_points = []
secondary_spiral_points = []


for angle in rays:
    rad = np.radians(angle)
    main_spiral_points.append(a * np.exp(b * rad))
    secondary_spiral_points.append((a * np.exp(b * rad) + a * np.exp(b * (rad + 2 * np.pi))) / 2)

print(main_spiral_points)
print(secondary_spiral_points)
