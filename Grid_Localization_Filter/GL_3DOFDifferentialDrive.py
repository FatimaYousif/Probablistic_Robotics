from GL import GL
from DifferentialDriveSimulatedRobot import *
from DR_3DOFDifferentialDrive import *
from math import exp, sqrt, pi
from scipy import stats
from Pose3D import *
from Histogram import *

# probability density function
# https://www.probabilitycourse.com/chapter4/4_2_3_normal.php

def pdf(mean, std, x):
    return 1 / (std * sqrt(2 * pi)) * exp(- (x-mean)**2 / (2 * std**2))


class GL_3DOFDifferentialDrive(GL, DR_3DOFDifferentialDrive):
    """
    Grid Reckoning Localization for a 3 DOF Differential Drive Mobile Robot.
    """

    def __init__(self, dx_max, dy_max, range_dx, range_dy, p0, index, kSteps, robot, x0, *args):
        """
        Constructor of the :class:`GL_4DOFAUV` class. Initializes the Dead reckoning localization algorithm as well as the histogram filter algorithm.

        :param dx_max: maximum x displacement in meters
        :param dy_max: maximum y displacement in meters
        :param range_dx: range of x displacements in meters
        :param range_dy: range of y displacements in meters
        :param p0: initial probability histogram
        :param index: index struture containing plotting information
        :param kSteps: number of time steps to simulate the robot motion
        :param robot: robot object
        :param x0: initial robot pose
        :param args: additional arguments
        """

        super().__init__(p0, index, kSteps, robot, x0, *args)

        self.sigma_d = 1

        self.range_dx = range_dx
        self.range_dy = range_dy

        self.Deltax = 0  # x displacement since last x cell motion
        self.Deltay = 0  # y displacement since last y cell motion
        self.Delta_etak = Pose3D(np.zeros((3, 1)))

        self.cell_size = self.pk_1.cell_size_x  # cell size is the same for x and y

        self.uk_1 = np.zeros((2, 1), dtype=np.float32)

  
    def GetMeasurements(self):
        """
        Read the measurements from the robot. Returns a vector of range distances to the map features.
        Only those features that are within the :attr:`SimulatedRobot.SimulatedRobot.Distance_max_range` of the sensor are returned.
        The measurements arribe at a frequency defined in the :attr:`SimulatedRobot.SimulatedRobot.Distance_feature_reading_frequency` attribute.

        :return: vector of distances to the map features
        """
        # TODO: To be implemented by the student

        return self.robot.ReadRanges()
        
        # pass

    def StateTransitionProbability_4_uk(self,uk):
        return self.Pk[uk[0, 0], uk[1, 0]]

    def StateTransitionProbability_4_xk_1_uk(self, etak_1, uk):
        """
        Computes the state transition probability histogram given the previous robot pose :math:`\eta_{k-1}` and the input :math:`u_k`:

        .. math::

            p(\eta_k | \eta_{k-1}, u_k)

        :param etak_1: previous robot pose in cells
        :param uk: input displacement in number of cells
        :return: state transition probability :math:`p_k=p(\eta_k | \eta_{k-1}, u_k)`

        """

        # TODO: To be implemented by the student

        # target_position = NEXT MOVE
        # rest = same as measurement probab

        next_position = etak_1 + uk

        p_k = Histogram2D(self.p0.num_bins_x, self.p0.num_bins_y, self.p0.x_range, self.p0.y_range)

        for x_bin in p_k.x_range:
            for y_bin in p_k.y_range:
                curr_position = np.array([[x_bin, y_bin]]).T
                distance = np.linalg.norm(curr_position - next_position)

                p_k.element[x_bin, y_bin] = pdf(0, self.sigma_d, distance)

        return p_k

        pass

    def StateTransitionProbability(self):
        """
        Computes the complete state transition probability matrix. The matrix is a :math:`n_u \times m_u \times n^2` matrix,
        where :math:`n_u` and :math:`m_u` are the number of possible displacements in the x and y axis, respectively, and
        :math:`n` is the number of cells in the map. For each possible displacement :math:`u_k`, each previous robot pose
        :math:`{x_{k-1}}` and each current robot pose :math:`{x_k}`, the probability :math:`p(x_k|x_{k-1},u_k)` is computed.


        :return: state transition probability matrix :math:`P_k=p{x_k|x_{k-1},uk}`
        """

        # TODO: To be implemented by the student


        # Pk= ?

        # next = initialize
        # 3 steps = MOVE
        # uk=[∆x -1, ∆y-1]
       
        # p_uk =initialize
        # each cell = x y
        # etak_1 = prev = for above STP function 
        # store P(x)
        # assign P(x) -> transition


        transition_matrix = np.zeros((3, 3, self.p0.nCells, self.p0.nCells))

        for delta_x in range(3):
            for delta_y in range(3):
                uk = np.array([[delta_x - 1, delta_y - 1]]).T
                print("Computing transition matrix for Uk:", uk)

                # n**2 rows, n**2 columns
                p_uk = np.zeros((self.p0.nCells, self.p0.nCells))

                #  column = cell = values of p(etak|uk, etak_1)
                for column in range(self.p0.nCells):
                    index_y = column // self.p0.num_bins_x
                    index_x = column % self.p0.num_bins_x

                    etak_1 = np.array([[self.p0.x_range[index_x], self.p0.y_range[index_y]]]).T
                    
                    transition = self.StateTransitionProbability_4_xk_1_uk(etak_1, uk)

                    # probablities for each col/cell
                    p_uk[:, column] = transition.histogram_1d

                transition_matrix[delta_x, delta_y] = p_uk

        self.Pk = transition_matrix
        return self.Pk

        # pass

    def uk2cell(self, uk):
        """
        Converts the number of cells the robot has displaced along its DOFs in the world N-Frame to an index that can be
        used to acces the state transition probability matrix.

        :param uk: vector containing the number of cells the robot has displaced in all the axis of the world N-Frame
        :returns: index: index that can be used to access the state transition probability matrix
        """

        # TODO: To be implemented by the student

        # pass
        
        # index:
        # round -> int (discrete)
        # ones_like (uk) = initialize

        uk = np.round(uk).astype(np.int32)
        index=uk + np.ones_like(uk)
        return index
        # pass
   

    def MeasurementProbability(self, zk):
        """
        Computes the measurement probability histogram given the robot pose :math:`\eta_k` and the measurement :math:`z_k`.
        In this case the the measurement is the vector of the distances to the landmarks in the map.

        *** To be implemented by the student ***

        :param zk: :math:`z_k=[r_0~r_1~..r_k]` where :math:`r_i` is the distance to the i-th landmark in the map.
        :return: Measurement probability histogram :math:`p_z=p(z_k | \eta_k)`
        """

        total_p_z = Histogram2D(self.p0.num_bins_x, self.p0.num_bins_y, self.p0.x_range, self.p0.y_range)

      
        for landmark, landmark_dist in zk:

            p_z = Histogram2D(self.p0.num_bins_x, self.p0.num_bins_y, self.p0.x_range, self.p0.y_range)

            # for each bin
            for y_bin, y_centre in zip(p_z.y_range, p_z.y_center_range):
                for x_bin, x_centre in zip(p_z.x_range, p_z.x_center_range):

                    # center = (x y) = 2D = transpose
                    cell_centre = np.array([[x_centre, y_centre]]).T

                    #  dist = center - landmark
                    distance = np.linalg.norm(cell_centre - landmark)

                    # P
                    # distance = mean
                    # sigma_d from this class
                    # landmark_dist = x
                    # store in bin

                    probability = pdf(distance, self.sigma_d, landmark_dist)
                    p_z.element[x_bin, y_bin] = probability

            # normalize
            p_z.histogram_1d /= np.sum(p_z.histogram_1d)

            # total = sum
            total_p_z.histogram_1d += p_z.histogram_1d
        
        # for valid PDF
        total_p_z.histogram_1d /= len(zk)

        return total_p_z
    

    def GetInput(self,usk):
        """
        Provides an implementation for the virtual method :meth:`GL.GetInput`.
        Gets the number of cells the robot has displaced in the x and y directions in the world N-Frame. 
        To do it, it
        calls several times the parent method :meth:`super().GetInput`, 
        corresponding to the Dead Reckoning Localization
        of the robot, until it has displaced at least one cell in any direction.

        Note that an iteration of the robot simulation :meth:`SimulatedRobot.fs` is normally done in the :meth:`GL.LocalizationLoop`
        method of the :class:`GL.Localization` class, but in this case it is done here to simulate the robot motion
        between the consecutive calls to :meth:`super().GetInput`.

        :param usk: control input of the robot simulation
        :return: uk: vector containing the number of cells the robot has displaced in the x and y directions in the world N-Frame
        """
        # TODO: To be implemented by the student
        # ------------------------------------------
        
        def absolute_displacement(delta):
            """ Compute the absolute cell displacement based on a metric delta. """
            return abs(delta) // self.cell_size
        
        def true_displacement(delta):
            """ Given a metric delta, compute the corresponding cell displacement. """
            return absolute_displacement(delta) if delta > 0 else -absolute_displacement(delta)


        while absolute_displacement(self.Deltax) + absolute_displacement(self.Deltay) == 0:
            # Run a simulation step
            self.robot.fs(self.robot.xsk, usk)

            # Get encoder readings
            uk = DR_3DOFDifferentialDrive.GetInput(self)
            print(uk)
           # Convert encoder readings to raw displacement
            delta_reading = uk
            wheel_distance = delta_reading * 2 * pi * self.wheelRadius / self.robot.pulse_x_wheelTurns
            wheel_velocity = wheel_distance / self.dt
            forward_velocity = np.mean(wheel_velocity)
            angular_velocity = ((wheel_velocity - forward_velocity) / (self.wheelBase / 2))[1][0]

            expanded_uk = np.array([[forward_velocity, 0, angular_velocity]]).T
            # Compute displacement
            self.xk = self.xk_1.oplus(expanded_uk * self.dt)
            self.Delta_etak = self.xk - self.xk_1
            # Accumulate displacement
            self.Deltax += self.Delta_etak[0][0]
            self.Deltay += self.Delta_etak[1][0]

            self.uk_1 = uk
            self.xk_1 = self.xk
            

        displacement_x, displacement_y = true_displacement(self.Deltax), true_displacement(self.Deltay)

        # Partially reset deltas (will bring them to [-cell size, cell size])
        self.Deltax -= displacement_x
        self.Deltay -= displacement_y

        return np.array([[displacement_x, displacement_y]]).T


        # pass




