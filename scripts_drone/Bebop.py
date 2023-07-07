import numpy as np

class Bebop:
    def __init__(self, pose, desired, gains, max_variables):
        self.set_pose(pose)
        self.set_desired(desired)
        self.set_gains(gains)
        self.set_max_variables(max_variables)

    def set_pose(self, pose):
        self.pose = self._unpack_array(pose, 18, "Pose")

    def set_desired(self, desired):
        self.desired = self._unpack_array(desired, 18, "Desired")

    def set_gains(self, gains):
        self.gains = self._unpack_array(gains, 4, "Gains")

    def set_max_variables(self, max_variables):
        self.max_variables = self._unpack_array(max_variables, 4, "Max Variables")

    def _unpack_array(self, arr, expected_length, name):
        # Check if array has expected length
        if len(arr) != expected_length:
            raise ValueError(f"{name} size is incorrect. Expected size: {expected_length}. Got: {len(arr)}")
        return arr

    def compute_controller(self):
        # Prepare parameters
        kp = np.array([[self.gains[0],     0     ],
                       [   0    ,   self.gains[1]]])

        kd = np.array([[self.gains[2],     0     ],
                       [   0    ,   self.gains[3]]])

        theta_max, phi_max, z_dot_max, psi_dot_max = self.max_variables

        # Compute theta and phi controller commands
        u_theta, u_phi = self._compute_u_theta_phi(kp, kd, theta_max, phi_max)

        # Compute z_dot and psi_dot controller commands
        u_z_dot, u_psi_dot = self._compute_u_z_dot_psi_dot(z_dot_max, psi_dot_max)

        return u_theta, u_phi, u_z_dot, u_psi_dot

    def _compute_u_theta_phi(self, kp, kd, theta_max, phi_max):
        g = 9.8 # gravity acceleration constant

        # Compute tilde values
        X_til     = self.desired[:2] - self.pose[:2]
        X_dot_til = self.desired[6:8] - self.pose[6:8]

        # Compute reference values
        X_ddot_ref = self.desired[12:14] + np.dot(kd, X_dot_til) + np.dot(kp, X_til)

        # Compute normalization and rotation matrices
        normalization_theta_phi = np.array([[1/(g*theta_max), 0], 
                                            [0, 1/(g*phi_max)]])

        rotation_matrix_theta_phi = np.array([[np.cos(self.pose[5]) , np.sin(self.pose[5])], 
                                              [-np.sin(self.pose[5]), np.cos(self.pose[5])]])

        # Compute theta and phi controller commands
        u_theta, u_phi = np.dot(normalization_theta_phi, np.dot(rotation_matrix_theta_phi, X_ddot_ref))

        return u_theta, u_phi

    def _compute_u_z_dot_psi_dot(self, z_dot_max, psi_dot_max):
        # Compute tilde values
        Z_PSI_til = np.array([self.desired[2] - self.pose[2], self.desired[5] - self.pose[5]])

        # Compute reference values
        z_dot_ref, psi_dot_ref = self.desired[8:10] + np.dot(self.gains[:2], Z_PSI_til)

        # Compute z_dot and psi_dot controller commands
        u_z_dot = z_dot_ref / z_dot_max
        u_psi_dot = psi_dot_ref / psi_dot_max

        return u_z_dot, u_psi_dot
