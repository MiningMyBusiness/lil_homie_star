import numpy as np
import math
from homie_particle import homie_particle
import copy

class homie_filter:
    
    def __init__(self, num_particles, start_x, start_y, start_theta):
        # create a list of homie particles
        self._num_homies = num_particles
        self._homies = np.array([homie_particle(start_x, start_y, start_theta) for i in range(num_particles)])
        self.give_particle_id()
        self._xs = []
        self._ys = []
        self._thetas = []
        
        # physical properties of all homie robots 
        self._wheel_circ = 21.2 # cm circumference of each wheel 
        self._clics_per_rev = 20 # number of "clicks" per revolution from rotary encode
        self._wheel_err_mult = 0.1 # multiplier to guesstimate wheel movement error
        
        self._homie_weights = None
        
        
        
    def move_particles(self, l_cnt, r_cnt, del_t):
        # compute distance moved by each wheel
        l_dist_mu = self._wheel_circ*l_cnt/self._clics_per_rev
        r_dist_mu = self._wheel_circ*r_cnt/self._clics_per_rev
        
        # compute standard deviation of movement for each wheel
        l_dist_sig = self._wheel_err_mult*np.sqrt(np.abs(l_dist_mu))
        r_dist_sig = self._wheel_err_mult*np.sqrt(np.abs(r_dist_mu))
        
        # randomly sample left and right wheel distances and move particles
        for i in range(len(self._homies)):
#             print('Moving particle: '+str(self._homies[i]._id))
            this_l_dist = np.random.normal(l_dist_mu, l_dist_sig)
            this_r_dist = np.random.normal(r_dist_mu, r_dist_sig)
            self._homies[i].move(this_l_dist, this_r_dist, del_t)
    
    
    
    def update_particle_weights(self, front, left, back, right): # sensed ranges from each direction
        # modify detection to represent the position of the distance sensors 
        # front
        front_angle = np.rad2deg(math.atan(2.5/(front + 3)))
        front = np.sqrt(np.power(front + 3, 2) + (2.5**2))
        # left
        left_angle = 90 + np.rad2deg(math.atan(5/(left + 6.5)))
        left = np.sqrt(np.power(left + 6.5, 2) + (5**2))
        # back
        back_angle = 180 - np.rad2deg(math.atan(3/(back + 14.5)))
        back = np.sqrt(np.power(back + 14.5, 2) + (3**2))
        # right
        right_angle = 270 - np.rad2deg(math.atan(5/(right + 6.5)))
        right = np.sqrt(np.power(right + 6.5, 2) + (5**2))
        homie_weights = []
        for i in range(len(self._homies)):
#             print('Updating particle: '+str(self._homies[i]._id))
            this_idxs = []
            front_weight = 1
            left_weight = 1
            back_weight = 1
            right_weight = 1
            if not np.isnan(front):
                front_weight, front_land_idx = self._homies[i].assign_weight(front, front_angle)
                this_idxs.append(front_land_idx)
            if not np.isnan(left):
                left_weight, left_land_idx = self._homies[i].assign_weight(left , left_angle, exclude_idxs=this_idxs)
                this_idxs.append(left_land_idx)
            if not np.isnan(back):
                back_weight, back_land_idx = self._homies[i].assign_weight(back, back_angle, exclude_idxs=this_idxs)
                this_idxs.append(back_land_idx)
            if not np.isnan(right):
                right_weight, right_land_idx = self._homies[i].assign_weight(right, right_angle, exclude_idxs=this_idxs)
            homie_weights.append(front_weight*left_weight*back_weight*right_weight)
#             print('')
        self._homie_weights = homie_weights
        
        
        
    
    def resample_homies(self):
        if self._homie_weights is None:
            print(' No sensing has happended, Please provide sensory data')
            return None
        else:
            normalize_weights = np.array(self._homie_weights)/np.sum(self._homie_weights)
            cum_sum = []
            for i in range(len(normalize_weights)+1):
                cum_sum.append(np.sum(normalize_weights[0:i]))
            cum_sum = np.array(cum_sum)
            new_homies = []
            xs = []
            ys = []
            thetas = []
            for i in range(self._num_homies):
                xs.append(self._homies[i]._x)
                ys.append(self._homies[i]._y)
                thetas.append(self._homies[i]._theta)
                rand_pick = np.random.uniform(0,1)
                homie_pick_idx = np.sum(cum_sum < rand_pick) - 1
                new_homies.append(copy.deepcopy(self._homies[homie_pick_idx]))
            self._homies = np.array(new_homies)
            self.give_particle_id()
            self._xs.append(xs)
            self._ys.append(ys)
            self._thetas.append(thetas)
            
            
    def give_particle_id(self):
        for i,homie in enumerate(self._homies):
            homie._id = i
    
    
    
    def get_most_landmarks(self):
        max_len = 0
        for homie in self._homies:
            this_len = len(homie._land_mu)
            if this_len > max_len:
                max_len = this_len
        return max_len
        