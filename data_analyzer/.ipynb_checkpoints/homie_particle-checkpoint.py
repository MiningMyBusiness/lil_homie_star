import numpy as np
import math
from scipy.spatial.distance import mahalanobis

class homie_particle:
    
    def __init__(self, xpos, ypos, orientation):
        self._x = xpos
        self._y = ypos
        self._theta = orientation # in radians wrt positive x-axis
        self._land_mu = [] # list of landmark mean positions, each element 1D-array with 2 elements
        self._land_cov = [] # list of landmark covariance (estimate of landmark position), 2D-array with 4 elements 2X2 vector
        
        self._dist_bet_wheels =  16 # cm distance between the right and left wheels
        self._id = None
    
    
    
    ### move the robot by rotation of left and right wheels
    def move(self, l_dist, r_dist, del_t):
        if l_dist != 0 or r_dist != 0:
            omega_delt = (r_dist - l_dist)/self._dist_bet_wheels
            v_r = r_dist/del_t
            v_l = l_dist/del_t
            R = (self._dist_bet_wheels/2)*((v_l + v_r)/(v_r - v_l))
            term1 = R*(math.sin(self._theta)*math.cos(omega_delt) + math.cos(self._theta)*math.sin(omega_delt))
            term2 = R*(math.sin(self._theta)*math.sin(omega_delt) - math.cos(self._theta)*math.cos(omega_delt))
            term3 = self._theta
            ICC_x = self._x - R*math.sin(self._theta)
            ICC_y = self._y + R*math.cos(self._theta)
            # update values
            self._x = ICC_x + term1
            self._y = ICC_y + term2
            self._theta = omega_delt + term3
            # ensure theta is between 0 and 2*pi
    #         self.constrain_theta()
    #         print(self._x, self._y, self._theta*(180/3.14159))
        
    def constrain_theta(self):
        if self._theta < 0:
            self._theta += (2*3.14159)
        if self._theta > (2*3.14159):
            self._theta -= (2*3.14159)
        
        
        
    
    
    ### assign weight to this robot based on recent sensor value
    def assign_weight(self, r, bearing, exclude_idxs=[]): # exclude idx for mutual exclusion
        """
            r being distance and bearing is angle to landmark
        """
        self.clean_landmarks()
        bearing = np.deg2rad(bearing)
        sense_xy = self.polar_to_xy(r, bearing)
#         print(r, bearing)
#         print('wall location:'+str(sense_xy))
        landmark_idx = self.associate_sense(sense_xy, exclude_idxs=exclude_idxs)
        landmark_idx = self.update_landmark(sense_xy, landmark_idx, r) # will assign a landmark idx if none exists
        imp_weight = self.get_imp_weight(sense_xy, landmark_idx)
        return imp_weight, landmark_idx
        
        
        
        
    def polar_to_xy(self, r, bearing):
        """
            polar is (r, theta) pair
        """
        head_vec_x = math.cos(self._theta)
        head_vec_y = math.sin(self._theta)
        sense_vec_x = head_vec_x*math.cos(bearing) - head_vec_y*math.sin(bearing)
        sense_vec_y = head_vec_x*math.sin(bearing) + head_vec_y*math.cos(bearing)
#         print(self._theta*180/3.14159)
#         print(sense_vec_x, sense_vec_y)
        mark_x = self._x + r*sense_vec_x
        mark_y = self._y + r*sense_vec_y
        return np.array([mark_x, mark_y])
    
    
    
    def associate_sense(self, sense_xy, exclude_idxs=[]):
        if len(self._land_mu) == 0: # if the particle has no landmarks
            return None
        else:
            min_dist = np.inf
            min_idx = None
            for qq in range(len(self._land_mu)):
                if qq not in exclude_idxs:
                    this_cov = self._land_cov[qq]
                    this_mu = self._land_mu[qq]
                    cov_inv = np.linalg.inv(this_cov)
                    this_dist = mahalanobis(sense_xy, this_mu, cov_inv)
                    if this_dist < min_dist:
                        min_dist = this_dist
                        min_idx = qq
            if min_dist < 2: # found an acceptable match to an existing landmark
                return min_idx
            else: # did not find any matching landmarks
                return None
            
            
            
    def update_landmark(self, sense_xy, landmark_idx, r):
        obs_cov = np.array([np.array([r,0]),np.array([0,r])])
        if landmark_idx is None:
            self._land_mu.append(sense_xy)
            self._land_cov.append(obs_cov)
            return (len(self._land_mu) - 1)
        else:
            ## kalman update for belief of landmark location and covariance
            this_cov = self._land_cov[landmark_idx]
            this_mu = self._land_mu[landmark_idx]
            y_k = sense_xy - this_mu
            s_k = np.matmul(np.matmul(np.identity(2), this_cov), 
                           np.identity(2).transpose())+ obs_cov
            k_k = np.matmul(np.matmul(this_cov,np.identity(2).transpose()), 
                            np.linalg.inv(s_k))
            k_k_y_k = np.array([k_k[0,0]*y_k[0] + k_k[0,1]*y_k[1], k_k[1,0]*y_k[0] + k_k[1,1]*y_k[1]])
            next_mu = this_mu + k_k_y_k
            next_cov = np.matmul((np.identity(2) - np.matmul(k_k, np.identity(2))),
                                 this_cov)
            self._land_mu[landmark_idx] = next_mu
            self._land_cov[landmark_idx] = next_cov
            return landmark_idx
        
        
        
        
    def get_imp_weight(self, sense_xy, landmark_idx):
        this_mu = self._land_mu[landmark_idx]
        this_cov = self._land_cov[landmark_idx]
        if np.sum(np.abs(this_cov)) > 4000:
            print(this_cov)
        this_num = np.exp(-1*0.5*(mahalanobis(sense_xy, this_mu, np.linalg.inv(this_cov))**2))
        this_den = np.sqrt(2*3.14159*np.linalg.det(this_cov))
        this_weight = this_num/this_den
        return this_weight
    
    
    def clean_landmarks(self):
        idx = 0
        while idx < len(self._land_cov):
            if np.sum(np.abs(self._land_cov)) > 40000:
                self._land_mu.pop(idx)
                self._land_cov.pop(idx)
            else:
                idx += 1