import rclpy
import numpy as np
import pandas as pd
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from scipy.optimize import nnls

def _compute_cluster_sizes(n_samples, dt, tau_min, tau_max, n_clusters):
    if tau_min is None:
        min_size = 1
    else:
        min_size = int(tau_min / dt)

    if tau_max is None:
        max_size = n_samples // 10
    else:
        max_size = int(tau_max / dt)

    result = np.logspace(np.log2(min_size), np.log2(max_size), num=n_clusters, base=2)

    return np.unique(np.round(result)).astype(int)


def compute_avar(x, dt=1, tau_min=None, tau_max=None, n_clusters=100, input_type='mean'):
    ALLOWED_INPUT_TYPES = ['mean', 'increment', 'integral']

    if input_type not in ALLOWED_INPUT_TYPES:
        raise ValueError("`input_type` must be one of {}.".format(ALLOWED_INPUT_TYPES))

    x = np.asarray(x, dtype=float)
    if input_type == 'integral':
        X = x
    else:
        X = np.cumsum(x, axis=0)

    cluster_sizes = _compute_cluster_sizes(len(x), dt, tau_min, tau_max, n_clusters)

    avar = np.empty(cluster_sizes.shape + X.shape[1:])
    for i, k in enumerate(cluster_sizes):
        c = X[2*k:] - 2 * X[k:-k] + X[:-2*k]
        avar[i] = np.mean(c**2, axis=0) / k / k

    if input_type == 'mean':
        avar *= 0.5
    else:
        avar *= 0.5 / dt**2

    return cluster_sizes * dt, avar


def estimate_parameters(tau, avar, effects=None, sensor_names=None):
    ALLOWED_EFFECTS = ['quantization', 'white', 'flicker', 'walk', 'ramp']

    avar = np.asarray(avar)
    single_series = avar.ndim == 1
    if single_series:
        avar = avar[:, None]

    if effects is None:
        effects = ALLOWED_EFFECTS
    elif not set(effects) <= set(ALLOWED_EFFECTS):
        raise ValueError("Unknown effects are passed.")

    n = len(tau)

    A = np.empty((n, 5))
    A[:, 0] = 3 / tau**2
    A[:, 1] = 1 / tau
    A[:, 2] = 2 * np.log(2) / np.pi
    A[:, 3] = tau / 3
    A[:, 4] = tau**2 / 2
    mask = ['quantization' in effects,
            'white' in effects,
            'flicker' in effects,
            'walk' in effects,
            'ramp' in effects]

    A = A[:, mask]
    effects = np.asarray(ALLOWED_EFFECTS)[mask]

    params = []
    prediction = []

    for column in range(avar.shape[1]):
        avar_single = avar[:, column]
        A_scaled = A / avar_single[:, None]
        x = nnls(A_scaled, np.ones(n))[0]
        prediction.append(A_scaled.dot(x) * avar_single)
        params.append(np.sqrt(x))

    params = np.asarray(params)
    prediction = np.asarray(prediction).T

    params = pd.DataFrame(params, index=sensor_names, columns=effects)

    if single_series:
        params = params.iloc[0]
        prediction = prediction[:, 0]

    return params, prediction

class AllanVarianceNode(Node):
    def __init__(self):
        super().__init__('allan_variance_node')
        self.sample_size = 650_000

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        self.subscription = self.create_subscription(
            Imu,
            '/mavros/imu/data_raw',
            self.listener_callback,
            qos_profile
        )

        self.gyro_data = {'x': [], 'y': [], 'z': []}
        self.acc_data = {'x': [], 'y': [], 'z': []}
        self.timestamps = []

    def listener_callback(self, msg):
        progress = len(self.timestamps) / self.sample_size
        print(progress * 100, "%")

        self.timestamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

        self.gyro_data['x'].append(msg.angular_velocity.x)
        self.gyro_data['y'].append(msg.angular_velocity.y)
        self.gyro_data['z'].append(msg.angular_velocity.z)

        self.acc_data['x'].append(msg.linear_acceleration.x)
        self.acc_data['y'].append(msg.linear_acceleration.y)
        self.acc_data['z'].append(msg.linear_acceleration.z)

        if len(self.timestamps) > self.sample_size:
            self.get_logger().info("Collected enough data, computing Allan deviation...")
            self.compute_allan_all_axes()
            rclpy.shutdown()

    def compute_allan_all_axes(self):
        timestamps = np.array(self.timestamps)
        fs = 1.0 / np.mean(np.diff(timestamps))  # sample frequency
        dt = 1.0 / fs

        # Compute Allan variance for gyro z axis
        gyro_tau, gyro_avar = compute_avar(self.gyro_data['z'], dt=dt)
        # Compute Allan variance for acc z axis
        acc_tau, acc_avar = compute_avar(self.acc_data['z'], dt=dt)

        # Estimate noise parameters
        gyro_params, _ = estimate_parameters(gyro_tau, gyro_avar, effects=['white', 'walk'])
        acc_params, _ = estimate_parameters(acc_tau, acc_avar, effects=['white', 'walk'])

        gyro_noise = gyro_params['white']
        gyro_rw = gyro_params['walk']
        acc_noise = acc_params['white']
        acc_rw = acc_params['walk']

        print("\n=== Estimated IMU Noise Parameters ===")

        # Section 1: ORB-SLAM3 format
        print("\n--- ORB_SLAM3 IMU params ---")
        print(f"IMU.NoiseGyro: {gyro_noise:.1e}")
        print(f"IMU.NoiseAcc: {acc_noise:.1e}")
        print(f"IMU.GyroWalk: {gyro_rw:.1e}")
        print(f"IMU.AccWalk: {acc_rw:.1e}")
        print(f"IMU.Frequency: {int(round(fs))}.0")

        # Section 2: Kalibr format
        print("\n--- kalibr_imu.yaml file params ---")
        print("rostopic: /mavros/imu/data_raw")
        print(f"update_rate: {int(round(fs))}.0")
        print()
        print(f"accelerometer_noise_density: {acc_noise:.1e}")
        print(f"accelerometer_random_walk: {acc_rw:.1e}")
        print(f"gyroscope_noise_density: {gyro_noise:.1e}")
        print(f"gyroscope_random_walk: {gyro_rw:.1e}")

        # Plot Allan deviation
        import matplotlib.pyplot as plt
        plt.figure()
        plt.loglog(gyro_tau, np.sqrt(gyro_avar), label="Gyro Z")
        plt.loglog(acc_tau, np.sqrt(acc_avar), label="Acc Z")
        plt.xlabel("Tau [s]")
        plt.ylabel("Allan Deviation")
        plt.legend()
        plt.grid(True, which='both')
        plt.title("Allan Deviation - Z Axes")
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = AllanVarianceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
