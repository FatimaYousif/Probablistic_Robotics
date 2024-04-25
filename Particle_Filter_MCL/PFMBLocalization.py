import numpy as np
import math
from MCLocalization import MCLocalization
import random


class PFMBL(MCLocalization):
    """
    Particle Filter Map Based Localization class.

    This class defines a Map Based Localization using a Particle Filter. It inherits from :class:`MCLocalization`, 
    so the Prediction step is already implemented.
    It needs to implement the Update function, and consecuently the Weight and Resample functions.
    """
    def __init__(self, zf_dim, M, *args) -> None:
        
        self.zf_dim = zf_dim  # dimensionality of a feature observation
        
        self.M = M          #landmarks
        self.nf = len(M)
        super().__init__(*args)
    
    def pdf(self,x, mean, sigma):
        return (1 / (sigma * math.sqrt(2 * math.pi))) * math.exp(-((x - mean) ** 2) / (2 * sigma ** 2))



    def Weight(self, z, R): 
        """
        Weight each particle by the liklihood of the particle being correct.
        The probability the particle is correct is given by the probability that it is correct given the measurements (z). 

        
        :param z: measurement vector
        :param R: measurement noise covariance
        :return: None
        """
        # To be completed by the student
        
        # landmark
        lm = self.M
        
        # weights
        # particle_weights = np.zeros_like(self.particle_weights)
        particle_weights=np.copy(self.particle_weights)
        # like measurementProbablity

        # landmark (l), measured_l_dist(zl) = z
        for l, zl in enumerate(z):
            pos=zl[0]
            dist_observed=zl[1]
            # getMeasurements like
            for idx, particle in enumerate(self.particles):
                
                # dist(particle, landmark)
                particle_dist = np.linalg.norm(particle[0:2]-pos)

                # guassian
                pdf = self.pdf(x=particle_dist ,mean=dist_observed, sigma=R)
                
                # weights * guassian
                particle_weights[idx] = particle_weights[idx] * pdf
                # particle_weights[idx] = particle_weights[idx] + pdf
        particle_weights/=len(z)
        self.particle_weights=particle_weights
        
        return None 


        
    
    def Resample(self):
        """
        Resample the particles based on their weights to ensure diversity and prevent particle degeneracy.

        This function implements the resampling step of a particle filter algorithm. It uses the weights
        assigned to each particle to determine their likelihood of being selected. Particles with higher weights
        are more likely to be selected, while those with lower weights have a lower chance.

        The resampling process helps to maintain a diverse set of particles that better represents the underlying
        probability distribution of the system state. 

        After resampling, the attributes 'particles' and 'weights' of the ParticleFilter instance are updated
        to reflect the new set of particles and their corresponding weights.

        :return: None
        """
        # To be completed by the student
        # print("Resampling",self.particle_weights)
        random_particles = []

        # this M for iterations
        M = len(self.particles)        #iterations
        W = np.sum(self.particle_weights)    #weighted

        # Book page 90= USR= line 4
        r = np.random.uniform(0, W/M)      #rand 
        i = 0 
        c = self.particle_weights[i]

        # slides 90 = universal stochastic resampling
        for m in range(M):
            u = r + m*(W/M)
            while u > c:
                i = i + 1
                c = c + self.particle_weights[i] 
            random_particles.append(self.particles[i])
        self.particles = random_particles

        # noise
        # Q  = np.diag(np.array([0.2 ** 2, 0 ,0.05 ** 2]))
        # sampling_noise =  np.random.multivariate_normal([0,0,0],Q).reshape(3,1) 
        # for i in range(len(self.particles)):
        #     self.particles[i] =   self.particles[i].oplus(sampling_noise)   
        self.particle_weights = np.ones(len(self.particles))/len(self.particle_weights)    #add noise here
        return None
    
    def Update(self, z, R):
        """
        Update the particle weights based on sensor measurements and perform resampling.

        This function adjusts the weights of particles based on how well they match the sensor measurements.
       
        The updated weights reflect the likelihood of each particle being the true state of the system given
        the sensor measurements.

        After updating the weights, the function may perform resampling to ensure that particles with higher
        weights are more likely to be selected, maintaining diversity and preventing particle degeneracy.
        
        :param z: measurement vector
        :param R: the covariance matrix associated with the measurement vector

        """
        # To be completed by the student

        # book slide 89
        self.Weight(z,R)
        self.particle_weights/=np.sum(self.particle_weights)
        self.neff = 1/ np.sum(self.particle_weights**2) 
        if self.neff < len(self.particle_weights)/2:
            self.Resample()
            
        # return
    

    def Localize(self):
        uk, Qk = self.GetInput()

        if uk.size > 0:
            self.Prediction(uk, Qk)
        
        zf, Rf = self.GetMeasurements()
        if len(zf) > 0:
            self.Update(zf, Rf)

        self.PlotParticles()
        return self.get_mean_particle()