# Week 4 - Motion Model & Particle Filters
- land mark의 map coordinate값과 particle의 map coordinate값을 Euclidean식을 이용하여 두 오브젝트의 거리를 구한다.
- 이후 sensor range 값에 들어오는 landmark 값을 set 시켜준다.
- landmark와의 상대거리(car coordinate)값을 map coordinate 값으로 변환시켜주기 위해 particle의 map coordinate상에서의 state를 이용한다.
- 이렇게 예측된 landmark의 map coordinate 상의 location을 predictation 해준다. Cancel changes
#
    def update_weights(self, sensor_range, std_landmark_x, std_landmark_y,
                       observations, map_landmarks):
            
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
#
- 위에서 predict한 land mark의 location을 기존에 observation된 값을 비교하여 두 값 사이에 최소하되는 landmark을 associate 해준다.
- association value와 위에서 predictation value를 normalization해주어 landmark의 위치에 대한 probability를 계산해준다.
- map coordinate 상의 x값과 y값의 probability를 통해 particle에 대한 weighting을 부여해준다.

#
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
 #

- 파티클 weight 에 따라 resampling 되며 가중치가 크면 여러 번, 가중치가 작으면 클 때보다는 작게 sampling 된다.
- weight idx 함수를 이용하여 다음 주기의 Particle 에 weighting sampling 하게 된다.
- resampling 된 파티클을 복사하여 재입력한다.

#
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
#
[//]: # (Image References)
[empty-update]: ./empty-update.gif
[example]: ./example.gif

## Assignment

You will complete the implementation of a simple particle filter by writing the following two methods of `class ParticleFilter` defined in `particle_filter.py`:

* `update_weights()`: For each particle in the sample set, calculate the probability of the set of observations based on a multi-variate Gaussian distribution.
* `resample()`: Reconstruct the set of particles that capture the posterior belief distribution by drawing samples according to the weights.

To run the program (which generates a 2D plot), execute the following command:

```
$ python run.py
```

Without any modification to the code, you will see a resulting plot like the one below:

![Particle Filter without Proper Update & Resample][empty-update]

while a reasonable implementation of the above mentioned methods (assignments) will give you something like

![Particle Filter Example][example]

Carefully read comments in the two method bodies and write Python code that does the job.
