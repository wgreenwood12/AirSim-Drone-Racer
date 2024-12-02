import airsimneurips
import time
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PIL import Image  # Ensure you import Image from PIL
import io
# PID Controller Class
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        # Proportional term
        P = self.Kp * error

        # Integral term
        self.integral += error * dt
        I = self.Ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        D = self.Kd * derivative

        # Store current error for next calculation
        self.prev_error = error

        # Return the control output (speed adjustment)
        return P + I + D

def setup_drone_environment(client):
    client.confirmConnection()
    print('Connection confirmed')
    print('Loading level...')
    client.simLoadLevel('Soccer_Field_Easy')
    print('Level loaded')
    time.sleep(5)

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

def spline_interpolation(x, y, z, num_points):
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

def move_drone_with_pid(client, x_smooth, y_smooth, z_smooth, pid_controller, base_speed, completion_radius, dt):
    num_points = len(x_smooth)
    for i in range(num_points):
        target_pos = (x_smooth[i], y_smooth[i], z_smooth[i])
        drone_pos = client.getMultirotorState(vehicle_name="drone_1").kinematics_estimated.position
        drone_pos = (drone_pos.x_val, drone_pos.y_val, drone_pos.z_val)

        # Calculate the error as the distance to the target position
        error = np.linalg.norm(np.array(target_pos) - np.array(drone_pos))

        # Calculate the adjusted speed using the PID controller
        speed_adjustment = pid_controller.compute(error, dt)
        adjusted_speed = max(0, base_speed + speed_adjustment)  # Ensure speed is non-negative

        # Skip the completion cylinder check for the last point
        if i < num_points - 1 and within_completion_cylinder(drone_pos, target_pos, completion_radius):
            print(f"Skipping point {i} within completion radius")
            continue

        print(f"Moving to point {i}: {target_pos} with speed {adjusted_speed:.2f}")
        client.moveToPositionAsync(x_smooth[i], y_smooth[i], z_smooth[i], adjusted_speed, vehicle_name="drone_1").join()

def start_race_with_pid_control(client, x, y, z, base_speed, sphere_radius, num_points, Kp, Ki, Kd, dt):
    pid_controller = PIDController(Kp, Ki, Kd)
    client.enableApiControl(vehicle_name="drone_1")
    client.arm(vehicle_name="drone_1")
    time.sleep(2)
    client.simStartRace()
    client.takeoffAsync(vehicle_name="drone_1", timeout_sec=2).join()
    time.sleep(2)

    ##PICTURE SECTION
    camera_name = "fpv_cam"  
    image_type = airsimneurips.ImageType.Scene  
    vehicle_name = "drone_1"  
    response = client.simGetImage(camera_name, image_type, vehicle_name)
    if response:
        print("Image captured successfully")
    else:
        print("Failed to capture image")
    if response:
        image = Image.open(io.BytesIO(response))  
        img_arr = np.array(image)
        plt.imshow(img_arr)
        plt.show()

    x_smooth, y_smooth, z_smooth = spline_interpolation(x, y, z, num_points)
    move_drone_with_pid(client, x_smooth, y_smooth, z_smooth, pid_controller, base_speed, sphere_radius, dt)

def plot_gate_spline_spheres(x, y, z, x_smooth, y_smooth, z_smooth, sphere_radius):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, color='r', label='Gate Locations', s=50)
    ax.plot(x_smooth, y_smooth, z_smooth, color='b', label='Spline Path', linewidth=2)

    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 50)
    for i in range(len(x)):
        x_sphere = sphere_radius * np.outer(np.cos(u), np.sin(v)) + x[i]
        y_sphere = sphere_radius * np.outer(np.sin(u), np.sin(v)) + y[i]
        z_sphere = sphere_radius * np.outer(np.ones(np.size(u)), np.cos(v)) + z[i]
        ax.plot_surface(x_sphere, y_sphere, z_sphere, color='g', alpha=0.3, edgecolor='none')

    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    ax.set_title('Gate Locations, Spline Path, and Completion Spheres')
    ax.legend()
    plt.show()

def main():
    SPEED = 5
    SPHERE_RADIUS = 3.6
    NUM_SPLINE_POINTS = 165
    KP = 0.654
    KI = 0.009
    KD = 0.012
    DT = 0.1
 
    client = airsimneurips.MultirotorClient()
    setup_drone_environment(client)
    x, y, z = initialize_gate_locations(client)

    # Optional: Plot the gates and spline path
    # x_smooth, y_smooth, z_smooth = spline_interpolation(x, y, z, NUM_SPLINE_POINTS)
    # plot_gate_spline_spheres(x, y, z, x_smooth, y_smooth, z_smooth, SPHERE_RADIUS)

    start_race_with_pid_control(client, x, y, z, SPEED, SPHERE_RADIUS, NUM_SPLINE_POINTS, KP, KI, KD, DT)

if __name__ == "__main__":
    main()
