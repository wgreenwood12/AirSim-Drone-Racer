import airsimneurips
import time
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def setup_drone_environment(client):
    client.confirmConnection()
    print('Connection confirmed')
    print('Loading level...')
    client.simLoadLevel('Soccer_Field_Easy')
    print('Level loaded')
    time.sleep(10)

def initialize_gate_locations(client):
    scene_objects = client.simListSceneObjects()
    gates = [obj for obj in scene_objects if 'Gate' in obj]
    x, y, z = [], [], []
    for gate in gates:
        pose = client.simGetObjectPose(gate)
        x.append(pose.position.x_val)
        y.append(pose.position.y_val)
        z.append(pose.position.z_val)
    return x, y, z

def spline_interpolation(x, y, z, num_points=75):
    t = np.linspace(0, 1, len(x))
    spline_x = CubicSpline(t, x)
    spline_y = CubicSpline(t, y)
    spline_z = CubicSpline(t, z)
    t_new = np.linspace(0, 1, num_points)
    x_smooth = spline_x(t_new)
    y_smooth = spline_y(t_new)
    z_smooth = spline_z(t_new)
    return x_smooth, y_smooth, z_smooth

def within_completion_cylinder(drone_pos, target_pos, radius):
    distance = np.linalg.norm(np.array([drone_pos[0], drone_pos[1]]) - np.array([target_pos[0], target_pos[1]]))
    return distance <= radius


def move_drone_along_spline(client, x_smooth, y_smooth, z_smooth, speed=8, completion_radius=4.0):
    for i in range(len(x_smooth)):
        target_pos = (x_smooth[i], y_smooth[i], z_smooth[i])
        drone_pos = client.getMultirotorState(vehicle_name="drone_1").kinematics_estimated.position
        drone_pos = (drone_pos.x_val, drone_pos.y_val, drone_pos.z_val)

        if within_completion_cylinder(drone_pos, target_pos, completion_radius):
            print(f"Skipping point {i} within completion radius")
            continue
        
        print(f"Moving to point {i}: {target_pos}")
        client.moveToPositionAsync(x_smooth[i], y_smooth[i], z_smooth[i], speed, vehicle_name="drone_1").join()

def start_race_and_control_drone(client, x, y, z):
    client.enableApiControl(vehicle_name="drone_1")
    client.arm(vehicle_name="drone_1")
    time.sleep(2)
    client.simStartRace()
    client.takeoffAsync(vehicle_name="drone_1", timeout_sec=2).join()
    time.sleep(2)
    x_smooth, y_smooth, z_smooth = spline_interpolation(x, y, z)
    move_drone_along_spline(client, x_smooth, y_smooth, z_smooth)

def plot_gate_spline_cylinders(x, y, z, x_smooth, y_smooth, z_smooth, radius=3.0, height=10.0):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, color='r', label='Gate Locations', s=50)
    ax.plot(x_smooth, y_smooth, z_smooth, color='b', label='Spline Path', linewidth=2)
    
    for i in range(len(x)):
        u = np.linspace(0, 2 * np.pi, 100)
        x_cylinder = x[i] + radius * np.cos(u)
        y_cylinder = y[i] + radius * np.sin(u)
        z_cylinder_top = np.full_like(u, z[i] + height / 2)
        z_cylinder_bottom = np.full_like(u, z[i] - height / 2)  
        
        for j in range(len(u)):
            ax.plot([x_cylinder[j], x_cylinder[j]], [y_cylinder[j], y_cylinder[j]], [z[i] - height / 2, z[i] + height / 2], color='g', alpha=0.3)
        ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    ax.set_title('Gate Locations, Spline Path, and Completion Cylinders')
    ax.legend()

    plt.show()


def main():
    client = airsimneurips.MultirotorClient()
    setup_drone_environment(client)
    x, y, z = initialize_gate_locations(client)

    # Plot Gates with Cylinders:
    # x_smooth, y_smooth, z_smooth = spline_interpolation(x, y, z)
    # plot_gate_spline_cylinders(x, y, z, x_smooth, y_smooth, z_smooth)
    
    start_race_and_control_drone(client, x, y, z)

if __name__ == "__main__":
    main()

