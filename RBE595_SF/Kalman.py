import copy
import numpy as np

from rotation import rotrx, rotry, rotrz

class Kalman:
    def __init__(self, dim_x, dim_z, g = np.array([0,0,9.8]), gyro_bias = np.zeros(3) ):

        self.dim_x = dim_x
        self.dim_z = dim_z
        self.gravity_inertial = g
        self.gyro_bias = gyro_bias

        # State vectors
        self.x = np.zeros(dim_x) # States, column vector
        self.x_predict = np.zeros(dim_x) # Intermediate state prediction
        self.z = np.zeros(dim_z) # Measurements, column vector

        # Model
        self.A = np.zeros(shape=(dim_x,dim_x)) # State matrix, x_t = A @ x_t-1 + B @ u
        self.B = np.zeros(shape=(dim_x,6))
        self.H = np.zeros(shape=(dim_z,dim_x)) # Measurement function, z = Hx

        # Filter functions
        self.K = np.zeros(shape=(dim_x,dim_z) ) # Kalman gain
        self.P = np.identity(dim_x) # Covariance matrix
        self.Q = np.identity(dim_x) # Process (model) noise
        self.R = np.identity(dim_z) # Measurement noise

        self.q = 0.0001 # Tunable gain value to scale Q

    #==========================================================================
    def update_R( self, sigma_x, sigma_theta ):
        self.R = np.diag( [sigma_x, sigma_x, sigma_x, sigma_theta, sigma_theta, sigma_theta] )

    #==========================================================================
    def update_Q( self, dt = 0.01 ):
        self.Q = np.identity( self.dim_x )

        # Set diagonals
        self.Q[0:3,0:3] = self.Q[0:3,0:3] * dt**4/2
        self.Q[3:6,3:6] = self.Q[3:6,3:6] * dt**2
        self.Q[9:12,9:12] = self.Q[9:12,9:12] * dt**2

        # Set off-diagonals
        self.Q[0:3,3:6] = np.identity(3) * dt**3/2
        self.Q[3:6,0:3] = np.identity(3) * dt**3/2

        self.Q[0:3,6:9] = np.identity(3) * dt**2/2
        self.Q[6:9,0:3] = np.identity(3) * dt**2/2

        self.Q[3:6,6:9] = np.identity(3) * dt
        self.Q[6:9,3:6] = np.identity(3) * dt
        self.Q[9:12,12:15] = np.identity(3) * dt
        self.Q[12:15,9:12] = np.identity(3) * dt

        self.Q = self.Q * self.q # Scale the covariance matrix


    #==========================================================================
    def rotation_inertial2body( self, state ):
        """Uses individual Euler angles, calculates rotation matrix between inertial and body frames.
        From body frame: rotate yaw, pitch roll
        """
        R_yaw = rotrz( state[9])
        R_pitch = rotrx( state[10] )
        R_roll = rotry( state[11] )
        return R_roll @ R_pitch @ R_yaw # Body orientation matrix in inertial frame

    #==========================================================================
    def update_matrices( self, dt = 0.01, R_inertial2body = np.identity(3) ):
        # H maps the state vector x into the measurement vector z
        self.H = np.zeros( shape=(self.dim_z, self.dim_x) )
        self.H[0:3,6:9] = R_inertial2body.T
        self.H[3:6,12:15] = np.identity(3)

        # A maps previous state X_k-1 to current state X_k
        self.A = np.identity(self.dim_x)
        self.A[0:3,3:6] = np.identity(3) * dt * 0.5
        self.A[3:6,6:9] = np.identity(3) * dt * 0.5
        self.A[9:12,12:15] = np.identity(3) * dt * 0.5

        # self.A[0:3,6:9] = np.identity(3) * dt**2 * 0.5


        # B maps the control signal u into the current state X_k
        # self.B = np.zeros(shape=(self.dim_x,self.dim_x))
        # self.B[0:3,3:6] = R_inertial2body * dt
        # self.B[3:6,3:6] = R_inertial2body

        self.B = np.zeros(shape=(self.dim_x,6))

        self.B[0:3,0:3] = -np.identity(3) * dt
        self.B[0:3,3:6] = R_inertial2body * dt

        self.B[3:6,0:3] = -np.identity(3)
        self.B[3:6,3:6] = R_inertial2body


        self.update_Q()

    #==========================================================================
    def prediction_update( self, heading = np.zeros(3), dt = 0.1 ):

        u = np.append(self.x[3:6], heading)

        R_inertial2body = self.rotation_inertial2body( self.x )
        self.update_matrices(dt, R_inertial2body)
        self.x_predict = self.A @ self.x.T + self.B @ u.T

        # Update the covariance matrix, P
        self.P = self.A @ self.P @ self.A.T + self.Q

    #==========================================================================
    def correction_update( self, accel = np.zeros(3), gyro = np.zeros(3) ):


        # Update the Kalman gains
        self.K = self.P @ self.H.T @ np.linalg.pinv( self.H @ self.P @ self.H.T + self.R )


        self.z = np.append( accel, gyro )
        # self.z[0] += 0.2 # Trying to fix weird hardware offset
        self.z[0:3] = self.z[0:3] - self.rotation_inertial2body(self.x_predict).T @ self.gravity_inertial.T # Remove gravity bias
        self.z[3:6] = self.z[3:6] - self.gyro_bias # Remove bias inherent in the gyro hardware

        self.x = self.x_predict + self.K @ (self.z - self.H @ self.x_predict.T)

        # Update the covariance
        self.P = (np.identity(self.dim_x) - self.K @ self.H) @ self.P

