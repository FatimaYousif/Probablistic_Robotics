import numpy as np
import scipy
from PFMBLocalization import PFMBL
from DifferentialDriveSimulatedRobot import *
from Pose3D import *
import time

class PF_3DOF_MBL(PFMBL):
    def __init__(self, *args):

        zf_dim = 2 # dimensionality of a Cartesian feature observation
        super().__init__(zf_dim, particles, *args)
        
        self.dt = 0.1  # dt is the sampling time at which we iterate the DR
        self.wheelRadius = 0.1  # wheel radius
        self.wheelBase = 0.5  # wheel base
        self.robot.pulse_x_wheelTurns = 4096  # number of pulses per wheel turn

        self.Rr = np.array([[None],[None],[None]])
        self.Qrr = np.array([[0.3 ** 2], [0.3 ** 2] ,[0.3 ** 2]])
        # self.previous_time = time.time()

    def GetInput(self):
        """
        Get the input for the motion model.

        :return: * **uk, Qk**. uk: input vector (:math:`u_k={}^B[\Delta x~\Delta y]^T`), Qk: covariance of the input noise
        """

        # **To be completed by the student**.
        rsk, Qe = self.robot.ReadEncoders()

        return rsk, Qe
        # pass
    
    def GetMeasurements(self):
        """
        Read the measurements from the robot. Returns a vector of range distances to the map features.
        Only those features that are within the :attr:`SimulatedRobot.SimulatedRobot.Distance_max_range` of the sensor are returned.
        The measurements arribe at a frequency defined in the :attr:`SimulatedRobot.SimulatedRobot.Distance_feature_reading_frequency` attribute.

        :return: vector of distances to the map features, covariance of the measurement noise
        """
        
        # **To be completed by the student**.
        
        self.Rr, Qs = self.robot.ReadRanges()
        return self.Rr, Qs
    
        # self.Qrr =  np.full((len(self.robot.M)),1.2**2)
        # if (time.time() -self.previous_time) > (1/self.robot.xy_feature_reading_frequency):     # To be implemented
        #     self.Rr = self.robot.ReadRanges()
        #     self.Rr[self.Rr > self.robot.xy_max_range] =  0 
        #     self.previous_time = time.time()
        #     return self.Rr, self.Qrr
        # else:
        #     return np.array([]) , self.Qrr

        
    def MotionModel(self, particle, u, noise):
        # **To be completed by the student**.
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
        # pass
    

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
    particles = []
    for i in range(n_particles):
        pos_noised =  Pose3D(np.random.multivariate_normal([x0[0,0],x0[1,0],x0[2,0]],P0).reshape(3,1))
        particles.append(pos_noised)
    
    usk=np.array([[0.5, 0.03]]).T
    pf = PF_3DOF_MBL(index, kSteps, robot, particles)
    pf.LocalizationLoop(x0, usk)
    exit(0)