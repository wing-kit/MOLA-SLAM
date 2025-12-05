import numpy as np
import matplotlib.pyplot as plt
import yaml

# Load TUM trajectory
data = np.loadtxt('iamcarver.tum')
x = data[:, 1]
y = data[:, 2]

# Desired number of YAML points
TARGET = 200

# Compute uniformly spaced indices
# Ensures index 0 and last index are included
idx = np.linspace(0, len(data) - 1, TARGET, dtype=int)
data_yaml = data[idx]

# Quaternion → yaw
def quat_to_yaw(qx, qy, qz, qw):
    return np.arctan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy**2 + qz**2)
    )

# Prepare YAML data
yaml_data = []
for row in data_yaml:
    qx, qy, qz, qw = row[4], row[5], row[6], row[7]
    yaw = quat_to_yaw(qx, qy, qz, qw)
    yaml_data.append({
        'x': float(row[1]),
        'y': float(row[2]),
        'yaw': float(yaw)
    })

# Save YAML
with open('trajectory.yaml', 'w') as f:
    yaml.dump(yaml_data, f, default_flow_style=False)

# Plot full trajectory
plt.figure(figsize=(12, 10))
plt.plot(x, y, 'b-', linewidth=1.5, label='Trajectory')
plt.scatter(data_yaml[:, 1], data_yaml[:, 2], c='r', s=12, label='Subsampled (200 pts)')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Trajectory with 200 Uniform Samples')
plt.legend()
plt.grid(True, alpha=0.3)
plt.axis('equal')
plt.tight_layout()
plt.savefig('trajectory_xy.png', dpi=300, bbox_inches='tight')
plt.show()
