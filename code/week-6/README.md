# Week 6 - Prediction & Behaviour Planning
## -Gaussian Naive Bayed Classifier-
---
- 과제: Gaussian Naive Bayed Classifier를 통해 차량의 주행경로를 예측하는 algorithm을 구현한다.
#
	def train(self, X, Y):


		self.x = np.array(X)
		self.y = np.array(Y)

		groupby=list(set(Y))
		result=dict()

		for ip in groupby:
		    result[ip]=Y.count(ip)

		cnt = list(result.values())

		self.prior_prob = [cnt[0]/len(Y), cnt[1]/len(Y), cnt[2]/len(Y)]

		self.mu = np.array([self.x[np.where(self.y==i)].mean(axis=0) for i in self.classes])
		self.st = np.array([self.x[np.where(self.y==i)].std(axis=0) for i in self.classes])

		return self
	def predict(self, observation):


		prob = np.zeros(len(self.classes))

		for i in range(len(self.classes)):
		    a = 1
		    for j in range(self.x.shape[1]):
			a *= gaussian_prob(observation[j], self.mu[i][j], self.st[i][j])
			prob[i] = a

			res = prob / prob.sum()

		return self.classes[res.argmax(axis=0)]
#
- GNB Algorithm는 차량의 주행경로(left,keep,right)에 대한 data 수집 및 이를 통해 각 class에 대한  평균과 표준편차를 계산해준다.
- 계산된 평균과 표준편차는 Prediction과정에서 사용된다.
- 먼저, def.train에선 X,Y를 통해 사전 확률을 계산하고 Y_index를 통해 그에 matching 되는 X_index의 평균과 표준편차를 계산한다.
- 후에 def.predict에선 관측 데이터에 대한 확률을 Gaussian을 통해 계산해주고 prior를 곱해준다.
- 이후, total probability로 나눠주어 normalization해준다.
---
# RESULT
![image](https://user-images.githubusercontent.com/80083729/117477190-a995f900-af98-11eb-916d-1ab076d7b288.png)
- 위 그림과 같이 84%의 확률을 결과값으로 볼 수 있음을 확인할 수 있다.
---
## -Behaviour Planning-
### 1st 
- 아래 코드의 경우 predicted trajectory 값들을 고려하여 cost function을 통해 optimal path를 생성한다.
- FSM 기반으로 한 함수를 사용하여 다음 sample의 vehicle state를 구한다.
- min_cost와 best_next_state를 initialization해준다.
- state와 prediction을 고려하여 trajectory를 생성한다.
- 위의 과정을 반복하여 여러개의 trajectory를 생성하고 cost를 저장한다.
- 그리고 생성된 trajectory의 cost가 이전 sample의 cost보다 작으면(cost < min_cost) 그 값을 다시 min_cost에 저장한다.
- 이후 가장 적은 cost의 state를 반영하여 최적의 trajectory를 생성한다.
#
	state = self.successor_states()
		min_cost = 9999999
		best_next_state = None
		cost_array = []

		for st in state:

		    t = self.generate_trajectory(st, predictions)
		    cost = calculate_cost(self, t, predictions)
		    cost_array.append({'cost': cost, 'state': st, 'trajectory': t})  

		for k in cost_array:
		    if k['cost'] < min_cost:
			min_cost = k['cost']
			best_next_state = k['state']

		best_trajectory = self.generate_trajectory(best_next_state, predictions)

		return best_trajectory 
#
---
### 2nd
- goal_distance_cost 를 통해 goal과 현재의 위치에 따라 cost를 부여한다.
- final lane과 intended lane 사이의 cost에 대한 정의를 1-exp(-t)로 정의하였다.
- inefficiency_cost를 통해서는 차량의 속도를 반영하여 intended lane, final lane의 속도를 비교하여 느린 lane에 cost를 더 부여하게 된다.
#
	def goal_distance_cost(vehicle, trajectory, predictions, data):

	    goal_distance = data[2]

	    if goal_distance / vehicle.goal_s > 0.4:
		    cost = 0.0
	    elif goal_distance > 0:
		    cost = 1 - exp((vehicle.goal_lane - data[0] + vehicle.goal_lane - data[1]) / (data[2]))
	    else:
		    cost = 1
	    print("goal_distance_cost:", cost)
	    return cost

	def inefficiency_cost(vehicle, trajectory, predictions, data):

	    int_lane = data[0]
	    final_lane = data[1]
	    goal_dist = data[2]

	    if goal_dist / vehicle.goal_s > 0.4:
		    cost = exp(-(int_lane + final_lane))
	    else:
		    cost = 1 - exp(-(int_lane + final_lane))
	    # print(cost)
    return cost
#
---
# RESULT
![image](https://user-images.githubusercontent.com/80083729/117494047-c2110e00-afae-11eb-81fa-90fb891ea494.png)
- 다음과 같은 결과 값을 돌출할 수 있었다.
## Assignment #1

Under the directory [./GNB](./GNB), you are given two Python modules:

* `prediction.py`: the main module you run. The `main()` function does two things: (1) read an input file ([`train.json`](./GNB/train.json)) and train the GNB (Gaussian Naive Bayes) classifier using the data stored in it, and (2) read another input file ([`test.json`](./GNB/test.json)) and make predictions for a number of data points. The accuracy measure is taken and displayed.
* `classifier.py`: main implementation of the GNB classifier. You shall implement two methods (`train()` and `precict()`), which are used to train the classifier and make predictions, respectively.

Both input files ([`train.json`](./GNB/train.json) and [`test.json`](./GNB/test.json)) have the same format, which is a JSON-encoded representation of training data set and test data set, respectively. The format is shown below:

```
{
	"states": [[s_1, d_1, s_dot_1, d_dot_1],
	           [s_2, d_2, s_dot_2, d_dot_2],
	           ...
	           [s_n, d_n, s_dot_n, d_dot_n]
	          ],
	"labels": [L_1, L_2, ..., L_n]
}
```

The array `"states"` have a total of `n` items, each of which gives a (hypothetically) measured state of a vehicle, where `s_i` and `d_i` denote its position in the Frenet coordinate system. In addition, `s_dot_i` and `d_dot_i` give their first derivates, respectively. For each measured state, a label is associated (given in the `"labels"` array) that represents the vehicle's behaviour. The label is one of `"keep"`, `"left"`, and `"right"`, which denote keeping the current lane, making a left turn, and making a right turn, respectively.

The training set has a total of 750 data points, whereas the test set contains 250 data points with the ground truth contained in `"labels"`.

The GNB classifier is trained by computing the mean and variance of each component in the state variable for each observed behaviour. Later it is used to predict the behaviour by computing the Gaussian probability of an observed state for each behaviour and taking the maximum. You are going to implement that functionality. For convcenience, a separate function `gaussian_prob()` is already given in the module `classifier.py`.


---

## Assignment #2

Under the directory [./BP](./BP), you are given four Python modules:

* `simulate_behavior.py`: the main module you run. It instantiates a simple text-based simulation environment and runs it using the configuration specified in the same module.
* `road.py`: `class Road` is implemented here. It captures the state of the simulated road with a number of vehicles (including the ego) running on it, and visualizes it using terminal output.
* `vehicle.py`: `class Vehicle` implements the states of a vehicle and its transition, along with the vehicle's dynamics based on a simple kinematic assumption. Note that a vehicle's trajectory is represented by two instances of object of this class, where the first one gives the current state and the second one predicts the state that the vehicle is going to be in after one timestep.
* `cost_functions.py`: implementation of cost functions governing the state transition of the ego vehicle. The main job required for your assignment is to provide an adequate combination of cost functions by implementing them in this module.

### Task 1

Implement the method `choose_next_state()` in `vehicle.py`. It should

* determine which state transitions are possible from the current state (`successor_states()` function in the same module will be helpful),
* calculate cost for each state transition using the trajectory generated for each behaviour, and
* select the minimum cost trajectory and return it.

Note that you must return a planned trajectory (as described above) instead of the state that the vehicle is going to be in.

### Task 2

In `cost_functions.py`, templates for two different cost functions (`goal_distance_cost()` and `inefficiency_cost()`) are given. They are intended to capture the cost of the trajectory in terms of

* the lateral distance of the vehicle's lane selection from the goal position, and
* the time expected to be taken to reach the goal (because of different lane speeds),

respectively.

Note that the range of cost functions should be carefully defined so that they can be combined by a weighted sum, which is done in the function `calculate_cost()` (to be used in `choose_next_state()` as described above). In computing the weighted sum, a set of weights are used. For example, `REACH_GOAL` and `EFFICIENCY` are already defined (but initialized to zero values). You are going to find out a good combination of weights by an empirical manner.

You are highly encouraged to experiment with your own additional cost functions. In implementing cost functions, a trajectory's summary (defined in `TrajectoryData` and given by `get_helper_data()`) can be useful.

You are also invited to experiment with a number of different simulation settings, especially in terms of

* number of lanes
* lane speed settings (all non-ego vehicles follow these)
* traffic density (governing the number of non-ego vehicles)

and so on.

Remember that our state machine should be geared towards reaching the goal in an *efficient* manner. Try to compare a behaviour that switches to the goal lane as soon as possible (note that the goal position is in the slowest lane in the given setting) and one that follows a faster lane and move to the goal lane as the remaining distance decreases. Observe different behaviour taken by the ego vehicle when different weights are given to different cost functions, and also when other cost metrics (additional cost functions) are used.
