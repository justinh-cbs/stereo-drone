
import numpy as np

# possible forms for control interface:
# 1- takes motor commands
#    - could be RPM, could be current (amps)
# 2- takes attitude commands
#    - could do first order response, could implement a control loop here

# TODO
# class QuadrotorParams:
#     """ Store, load, save quadrotor parameters for the Quadrotor class
#     """
#     mass = [] # [float] quad

class Quadrotor:
    """ Class to simulate quadrotor dynamics and sensors

    Maintains its own internal, simulated time.  Interface consists of parameterization/
    initial values, control inputs, a propagator and getters for sensors.

    Axis convention is aerospace convention: [x,y,z] = [forward, right, down]

    Contains: 
    - motor models
    - sensor models
    """

    _mass = [] # [float, kg] mass
    _inertia = [] # [float 3x3, kg*m^2] moment of inertia

    def __init__(self):
        # model mass and inertia from collection of point masses
        pointmasses = [
            [0.05, 0.05, 0.0, 0.05],
            [-0.05, 0.05, 0.0, 0.05],
            [-0.05, -0.05, 0.0, 0.05],
            [0.05, -0.05, 0.0, 0.05],
            [0.0, 0.0, 0.02, 0.15],
            [0.0, 0.0, -0.02, 0.15],
        ]
        self._mass, self._inertia = inertia_from_pointmasses(pointmasses)
        self._x = np.array([2.0, 0.0, 0.0]) # position in local NED frame
        self._v = np.array([0.0, 0.0, 0.0]) # velocity in local NED frame
        self._R = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        self._t = 0
        self._motor_thrusts = np.array([0.4225, 0.4225, 0.4225, 0.4225])
        # initialize RPYT setpoints
        self.set_rpyt(1500, 1500, 1500, 1500)

        # print(np.matmul(self._R,self._x))

    def propagate(self, dt):

        thrust_force_setpoint = self._thrust_sp * 0.003266666 # convert PWM signal (1500) to force (in fig newtons)

        # split force evenly across four motors and model first order response
        for motor in range(0, 4):
            # disgusting pretentious math
            tau = 0.5 # first order time constant (or for the peasants like me, rate of response time)

            self._motor_thrusts[motor] += (dt / tau) * ((thrust_force_setpoint / 4) - self._motor_thrusts[motor])

        # F = m * (acc + wxv)
        # simple model assuming RPY = 0

        g = 9.8
        acc = (np.array([0, 0, -self._motor_thrusts.sum()]) + np.array([0, 0, (self._mass * g)])) / self._mass
        self._x += self._v * dt + 0.5 * acc * dt * dt
        self._v += acc * dt

        # M = I*alpha + wxIw

        print(self._motor_thrusts)
        self._t += dt

        print(self._t)

        print(f"position: {self._x[0]}, {self._x[1]}, {self._x[2]}")
        print(f"velocity: {self._v[0]}, {self._v[1]}, {self._v[2]}")
        print(f"setpoint: {self._roll_sp}, {self._pitch_sp}, {self._yaw_sp}, {self._thrust_sp}")

    def set_rpyt(self, roll, pitch, yaw, thrust):
        """
        set roll, pitch, yaw, thrust
        :params ^ those:
        """

        self._thrust_sp = thrust
        self._roll_sp = roll
        self._pitch_sp = pitch
        self._yaw_sp = yaw


# TODO
# flight controller model
# motor models
# sensor models

# TODO
# class MotorParams:
#     """ Helper Store, load, save quadrotor parameters for the Quadrotor class
#     """

# class ElectricMotor:
#     """ Model electric motor - component in Quadrotor model class
#     """

def inertia_from_pointmasses(masses):
    """ Helper function to compute mass and inertia matrix from a set of point
    masses.

    Takes as input a list of masses, each of the form:
    [x, y, z, mass] in [m, m, m, kg]
    """
    mass = 0.0
    Ixx = 0.0
    Ixy = 0.0
    Ixz = 0.0
    Iyy = 0.0
    Iyz = 0.0
    Izz = 0.0

    for point in masses:
        Ixx = Ixx + (point[1]**2 + point[2]**2)*point[3]
        Ixy = Ixy - (point[0]*point[1])*point[3]
        Ixz = Ixz - (point[0]*point[2])*point[3]
        Iyy = Iyy + (point[0]**2 + point[2]**2)*point[3]
        Iyz = Iyz - (point[1]*point[2])*point[3]
        Izz = Izz + (point[0]**2 + point[1]**2)*point[3]
        mass = mass + point[3]

    return mass, np.matrix([[Ixx, -Ixy, -Ixz],[-Ixy, Iyy, -Iyz],[-Ixz, -Iyz, Izz]])