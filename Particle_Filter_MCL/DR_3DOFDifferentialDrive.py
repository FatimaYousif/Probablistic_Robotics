from Localization import *
import numpy as np

class DR_3DOFDifferentialDrive(Localization):
    """
    Dead Reckoning Localization for a Differential Drive Mobile Robot.
    """
    def __init__(self, index, kSteps, robot, x0, *args):
        """
        Constructor of the :class:`prlab.DR_3DOFDifferentialDrive` class.

        :param args: Rest of arguments to be passed to the parent constructor
        """

        super().__init__(index, kSteps, robot, x0, *args)  # call parent constructor

        self.dt = 0.1  # dt is the sampling time at which we iterate the DR
        self.t_1 = 0.0  # t_1 is the previous time at which we iterated the DR
        self.wheelRadius = 0.1  # wheel radius
        self.wheelBase = 0.5  # wheel base
        self.robot.pulse_x_wheelTurns = 4096  # number of pulses per wheel turn

    def Localize(self, xk_1, uk):  # motion model
        """
        Motion model for the 3DOF (:math:`x_k=[x_{k}~y_{k}~\psi_{k}]^T`) Differential Drive Mobile robot using as input the readings of the wheel encoders (:math:`u_k=[n_L~n_R]^T`).

        :parameter xk_1: previous robot pose estimate (:math:`x_{k-1}=[x_{k-1}~y_{k-1}~\psi_{k-1}]^T`)
        :parameter uk: input vector (:math:`u_k=[u_{k}~v_{k}~w_{k}~r_{k}]^T`)
        :return xk: current robot pose estimate (:math:`x_k=[x_{k}~y_{k}~\psi_{k}]^T`)
        """

        # Store previous state and input for Logging purposes
        self.etak_1 = xk_1  # store previous state
        self.uk = uk  # store input

        # TODO: to be completed by the student

            # distance in left and right wheel
        #uk=[linear and angular] =nL or nR
        # d=linear_vel*2πr/pulses

        # reference=ppt
        dl = uk[0] * 2 * np.pi * self.wheelRadius / self.robot.pulse_x_wheelTurns
        dr = uk[1] * 2 * np.pi * self.wheelRadius / self.robot.pulse_x_wheelTurns

        #avg

        # ppt
        d = (dr + dl) / 2

       
        # xk_1= prev yaw (orientation)
        # theta=new angle after motion
        
        theta = xk_1[2, 0] + np.arctan2((dr - dl), self.wheelBase) 
        
        #new pose(xk) -> prev pose(xk_1)
        # ppt
        xk = (np.array([
            [xk_1[0, 0] + d * np.cos(theta)],  #x = prev + dcosθ
            [xk_1[1, 0] + d * np.sin(theta)],  #y = prev+ dsinθ
            [theta] #θ
            ]))
        return xk
        # pass

    def GetInput(self):
        """
        Get the input for the motion model. In this case, the input is the readings from both wheel encoders.

        :return: uk:  input vector (:math:`u_k=[n_L~n_R]^T`)
        """

        # TODO: to be completed by the student
        #above return explanation
        # output comes from ReadEncoders from DDSR class
        
        return self.robot.ReadEncoders()
        pass
