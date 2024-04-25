import numpy as np
import scipy
from MCLocalization import MCLocalization
from DifferentialDriveSimulatedRobot import *
from Pose3D import *

class PF_3DOF_DR(MCLocalization):
    def __init__(self, *args):
        super().__init__( *args)

        self.dt = 0.1  # dt is the sampling time at which we iterate the DR
        self.wheelRadius = 0.1  # wheel radius
        self.wheelBase = 0.5  # wheel base
        self.robot.pulse_x_wheelTurns = 4096  # number of pulses per wheel turn

        # self.usk = np.zeros((3, 1))  # simulated input to the motion model

        

    def GetInput(self):
        """
        Get the input for the motion model.

        :return: * **uk, Qk**. uk: input vector (:math:`u_k={}^B[\Delta x~\Delta y]^T`), Qk: covariance of the input noise
        """

        # **To be completed by the student**.
        # TODO: to be completed by the student

        # distances traveled by the left and right wheels
        # v = d/t -> d = vt
        # usk = vd (linear) and rd (angular)
        # wheelbase = distance between left and right wheel
        # wheelbase/2 = distance from the robot's CENTER
        # because left wheel will move in the opposite direction
        # meanwhile in right wheel movement, it's already +

        rsk, Qe = self.robot.ReadEncoders()

        return rsk, Qe
    
    
    
    def MotionModel(self, particle, u, noise):
             # **To be completed by the student**.
            """
            Motion model for a 3DOF differential drive robot.

            :param particle: Pose of the particle (3x1 numpy array or similar)
            :param u: Control input (2x1 numpy array or similar) - [linear_velocity, angular_velocity]
            :param noise: Noise in the control input (2x1 numpy array or similar)

            :return: Updated particle pose after applying motion model
            """
            u_noise=u+noise
            dl = u_noise[0][0] * 2 * np.pi * self.wheelRadius / self.robot.pulse_x_wheelTurns
            dr = u_noise[1][0] * 2 * np.pi * self.wheelRadius / self.robot.pulse_x_wheelTurns

            # v's for uk (displacement)
            #  d=vt

            vl = dl/self.dt
            vr = dr/self.dt

            vx = (vr + vl) / 2
            w = (vr - vl)/ self.wheelBase

            v = np.array([[vx],[0], [w]])

            uk  = Pose3D(v * self.dt)           
            return particle.oplus(uk)
            
        #pass
    
if __name__ == '__main__':

    M = [np.array([[-40, 5]]).T,
           np.array([[-5, 40]]).T,
           np.array([[-5, 25]]).T,
           np.array([[-3, 50]]).T,
           np.array([[-20, 3]]).T,
           np.array([[40,-40]]).T]  # feature map. Position of 2 point features in the world frame.

    #Simulation:
    xs0 = np.zeros((6, 1))
    kSteps = 5000
    index = [IndexStruct("x", 0, None), IndexStruct("y", 1, None), IndexStruct("yaw", 2, 0)]
    robot = DifferentialDriveSimulatedRobot(xs0, M)  # instantiate the simulated robot object
    
    # Particle Filter
    x0 = Pose3D(np.zeros((3,1)))  # initial guess
    P0 = np.diag([2**2, 2**2, np.deg2rad(20)**2]) # Initial uncertainty
    n_particles = 50

    #create array of n_particles particles distributed randomly around x0 with covariance P

    #
    # **To be completed by the student**.
    #

    P = np.diag(np.array([0.5,0.5,0.5]))     #covariance
    particles = []
    for i in range(n_particles):
        pos_noised =  Pose3D(np.random.multivariate_normal([x0[0,0],x0[1,0],x0[2,0]],P0).reshape(3,1))
        particles.append(pos_noised)
    
    usk=np.array([[0.5, 0.03]]).T
    pf = PF_3DOF_DR(index, kSteps, robot, particles)
    pf.LocalizationLoop(x0, usk)

    exit(0)
