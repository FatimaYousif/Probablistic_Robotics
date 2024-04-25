from DifferentialDriveSimulatedRobot import *
from DR_3DOFDifferentialDrive import *

if __name__ == "__main__":

    # feature map. Position of 2 point features in the world frame.
    M2D = [np.array([[-40, 5]]).T,
           np.array([[-5, 40]]).T,
           np.array([[-5, 25]]).T,
           np.array([[-3, 50]]).T,
           np.array([[-20, 3]]).T,
           np.array([[40,-40]]).T]
    xs0 = np.zeros((6,1))   # initial simulated robot pose

    robot = DifferentialDriveSimulatedRobot(xs0, M2D) # instantiate the simulated robot object

# ----------------------------------------------------
# a) Program the robot simulation to perform a circular shaped trajectory 

def circularTrajectory():
    num_steps = 5000  
    dt = 0.1  

    for step in range(num_steps):
        u_d = 1.0  
        r_d = np.deg2rad(8.0)

        robot_state = robot.fs(robot.xsk, np.array([[u_d], [r_d]]))

# ----------------------------------------------------
# b) Program the robot simulation to perform a trajectory shaped as an "8".

def eightShapedTrajectory():
    num_steps = 5000
    dt = 0.1
    r_d=np.deg2rad(8.0)

    for step in range(num_steps):
        u_d = 1.0  

        # Xsk * Xsk-1 <0  and Xsk < Xsk-1
        # If both are true --> change angular vel
        if robot.xsk[0] * robot.xsk_1[0]<0 and robot.xsk[0] < robot.xsk_1[0] :
            r_d = -r_d  
        
        robot_state = robot.fs(robot.xsk, np.array([[u_d], [r_d]]))

# simulation results

# circularTrajectory()
eightShapedTrajectory()

exit(0)


