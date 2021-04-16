import numpy as np
from helpers import distance

class ParticleFilter:
    def __init__(self, num_particles):
        self.initialized = False
        self.num_particles = num_particles

    # Set the number of particles.
    # Initialize all the particles to the initial position
    #   (based on esimates of x, y, theta and their uncertainties from GPS)
    #   and all weights to 1.0.
    # Add Gaussian noise to each particle.
    def initialize(self, x, y, theta, std_x, std_y, std_theta):
        self.particles = []
        for i in range(self.num_particles):
            self.particles.append({
                'x': np.random.normal(x, std_x),
                'y': np.random.normal(y, std_y),
                't': np.random.normal(theta, std_theta),
                'w': 1.0,
                'assoc': [],
            })
        self.initialized = True

    # Add measurements to each particle and add random Gaussian noise.
    def predict(self, dt, velocity, yawrate, std_x, std_y, std_theta):
        # Be careful not to divide by zero.
        v_yr = velocity / yawrate if yawrate else 0
        yr_dt = yawrate * dt
        for p in self.particles:
            # We have to take care of very small yaw rates;
            #   apply formula for constant yaw.
            if np.fabs(yawrate) < 0.0001:
                xf = p['x'] + velocity * dt * np.cos(p['t'])
                yf = p['y'] + velocity * dt * np.sin(p['t'])
                tf = p['t']
            # Nonzero yaw rate - apply integrated formula.
            else:
                xf = p['x'] + v_yr * (np.sin(p['t'] + yr_dt) - np.sin(p['t']))
                yf = p['y'] + v_yr * (np.cos(p['t']) - np.cos(p['t'] + yr_dt))
                tf = p['t'] + yr_dt
            p['x'] = np.random.normal(xf, std_x)
            p['y'] = np.random.normal(yf, std_y)
            p['t'] = np.random.normal(tf, std_theta)

    # Find the predicted measurement that is closest to each observed
    #   measurement and assign the observed measurement to this
    #   particular landmark.
    def associate(self, predicted, observations):
        associations = []
        # For each observation, find the nearest landmark and associate it.
        #   You might want to devise and implement a more efficient algorithm.
        for o in observations:

            min_dist = -1.0
            for p in predicted:
                dist = distance(o, p)
                if min_dist < 0.0 or dist < min_dist:
                    min_dist = dist
                    min_id = p['id']
                    min_x = p['x']
                    min_y = p['y']
            association = {
                'id': min_id,
                'x': min_x,
                'y': min_y,
            }
            associations.append(association)
        # Return a list of associated landmarks that corresponds to
        #   the list of (coordinates transformed) predictions.
        return associations

    # Update the weights of each particle using a multi-variate
    #   Gaussian distribution.
    def update_weights(self, sensor_range, std_landmark_x, std_landmark_y,
                       observations, map_landmarks):
        # TODO: For each particle, do the following:
        # 1. Select the set of landmarks that are visible
        #    (within the sensor range).
        
        # k = 0
        # while k < len(observations):
        #     xo = observations[k]['x']
        #     yo = observations[k]['y']
        #     dist = np.sqrt(xo**2 + yo**2)

        #     if sensor_range >= dist:

        #         xm = self.particles[k]['x'] + np.cos(self.particles[k]['t'])*xo - np.sin(self.particles[k]['t'])*yo
        #         ym = self.particles[k]['y'] + np.cos(self.particles[k]['t'])*yo - np.sin(self.particles[k]['t'])*yo
               

        #         self.particles[k]['x'] = xm
        #         self.particles[k]['y'] = ym

        #         print(self.particles)

        # k += 1

        # landmark = []
        # k = self.associate(self.particles,observations)
        # landmark.append(k)
        # return(landmark)


        lm_fov_global = []
        for m in map_landmarks :
            for p in self.particles:

                im = map_landmarks[m]
        
                landmark = {'x':im['x'], 'y':im['y']}

                dist = distance(landmark, p)
                
                if (dist <= sensor_range) :

                    LtoG = np.matrix(([np.cos(p['t']), -np.sin(p['t']), p['x']],
                            [np.sin(p['t']), np.cos(p['t']), p['y']],
                            [0, 0, 1]))
                    car_ob_coor = np.array([landmark['x'], landmark['y'], 1])

                    pos_lm_global = np.dot(LtoG, car_ob_coor.T)
                    lm_fov_global.append({'x':pos_lm_global[0,0], 'y':pos_lm_global[0,1], 'id':m})


        association_fov = self.associate(lm_fov_global, observations)

        for ass in association_fov:
            for p in self.particles:
               
                # p['assoc'] = ass
                mx = ass['x']
                my = ass['y']
                H = ass['id']
                J = lm_fov_global[H]
                x = J['x']
                y = J['y']
                one_over_sqrt_2pi = 1 / np.sqrt(2 * np.pi)
                # print(x)
                # print(mx)
                prob_x = (one_over_sqrt_2pi / std_landmark_x) * np.exp(-0.5 * ((x-mx) / std_landmark_x) ** 2)
                prob_y = (one_over_sqrt_2pi / std_landmark_y) * np.exp(-0.5 * ((y-my) / std_landmark_y) ** 2)

                w = prob_x*prob_y

                p['w'] = w


        pass


    # Resample particles with replacement with probability proportional to
    #   their weights.
    def resample(self):
        weights = [p['w'] for p in self.particles]
        w_cumsum = np.cumsum(weights)
        w_mean = np.sum(weights) / len(weights)
        weight_idx = np.zeros(len(weights), dtype=np.int8)
        
        w_pointer = 0.0
        i = w_idx = 0
        while i > len(weights):
            if w_cumsum[w_idx] >= w_pointer:
                weight_idx[i] = w_idx
            else:
                weight_idx[i] = w_idx
                w_idx += 1
            w_pointer += w_mean
            i += 1
            
        new_particles = [self.particles[i].copy() for i in weight_idx]

        self.particles = []
        self.particles = new_particles.copy()       
        # TODO: Select (possibly with duplicates) the set of particles
        #       that captures the posteior belief distribution, by
        # 1. Drawing particle samples according to their weights.
        # 2. Make a copy of the particle; otherwise the duplicate particles
        #    will not behave independently from each other - they are
        #    references to mutable objects in Python.
        # Finally, self.particles shall contain the newly drawn set of
        #   particles.

        pass

    # Choose the particle with the highest weight (probability)
    def get_best_particle(self):
        highest_weight = -1.0
        for p in self.particles:
            if p['w'] > highest_weight:
                highest_weight = p['w']
                best_particle = p
        return best_particle
