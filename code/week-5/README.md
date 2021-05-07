# Week 5 - Path Planning & the A* Algorithm

---
- 차량의 헤딩 각도를 고려하여 각 action마다 다른 cost를 부여하여 DP(Dynamic Programming)을 기반으로 최적 경로를 탐색하는 과제이다. 
- DP란? 가장 짧은 경로를 통해 목적지까지 도달할 수 있는 action을 각 cell에서 결정짓는 알고리즘이다. 
---
#
	for y, x, t in p:
		if (y, x) == goal and value[(t, y, x)] > 0:
			value[(t, y, x)] = 0
                	policy[(t, y, x)] = -999
                	change = True

#
- 반복문을 통해 목표 지점을 찾고 Vehicle이 목적지에 도착했을 때 그 값이 유효하다면 Value를 '0'으로 설정하고 목적지의 cost를 '0'으로 한다.
---
#

	elif grid[(y, x)] == 0:
              
		for i in range(0,3):             
			orient = (action[i]+ t) % 4
			y2 = y + forward[orient][0]
			x2 = x + forward[orient][1]

			if 0 <= x2 < grid.shape[1] and 0 <= y2 < grid.shape[0] and grid[(y2, x2)] == 0:        
				v2 = value[(orient, y2, x2)] + cost[i]

				if v2 < value[(t, y, x)]:
					value[(t, y, x)] = v2
					policy[(t, y, x)] = action[i]
					change = True
#
- Vehicle이 나아가야 할 방향은 이전 sample에서의 방향 ('t')에서 현재의 action을 더하여 4로 나눈 나머지로 결정된다.
- forward 행렬을 이용한 연산을 통해 (y2,x2)를 구하고, vehicle이 map상에서 유효한 위치에 있는지 확인한다.
- 만일 위 조건을 만족한다면, cost와 value의 연산을 통해 새로운 v2를 구한다.
- 현재 v2의 value가 최적인지 확인한다.
---
#
 	y, x, o = init

	    policy_i = policy[(o, y, x)]
	    for i in range(0,3):
		if policy_i == action[i]:
		    policy_name_start = action_name[i]

	    policy2D[(y, x)] = policy_name_start
	    while policy[(o, y, x)] != -999:
		if policy[(o, y, x)] == action[0]:
		    o2 = (o - 1) % 4  # turn left
		elif policy[(o, y, x)] == action[1]:
		    o2 = o  # go straight
		elif policy[(o, y, x)] == action[2]:
		    o2 = (o + 1) % 4  # turn right

		y, x = y + forward[o2][0], x + forward[o2][1]
		o = o2

		policy_temp = policy[(o,y,x)]
		if policy_temp == -999:
		    policy_name = "*"
		else:
		    for i in range(0,3):
			if policy_temp == action[i]:
			    policy_name = action_name[i]

		policy2D[(y,x)] = policy_name 

	    return policy2D

#
- 위 코드는 gobal path serching 이후 goal에 도달하기 위해 vehicle이 각 cell에서 한 action을 나타낸다.
- goal에 도달하기 전 까지 최적 경로 값에 대한 다음 policy을 생성하고 작업 이름(R, #, L)을 할당한다.
## Examples

We have four small working examples for demonstration of basic path planning algorithms:

* `search.py`: simple shortest path search algorithm based on BFS (breadth first search) - only calculating the cost.
* `path.py`: built on top of the above, generating an optimum (in terms of action steps) path plan.
* `astar.py`: basic implementation of the A* algorithm that employs a heuristic function in expanding the search.
* `policy.py`: computation of the shortest path based on a dynamic programming technique.

These sample source can be run as they are. Explanation and test results are given in the lecture notes.

## Assignment

You will complete the implementation of a simple path planning algorithm based on the dynamic programming technique demonstrated in `policy.py`. A template code is given by `assignment.py`.

The assignmemt extends `policy.py` in two aspects:

* State space: since we now consider not only the position of the vehicle but also its orientation, the state is now represented in 3D instead of 2D.
* Cost function: we define different cost for different actions. They are:
	- Turn right and move forward
	- Move forward
	- Turn left and move forward

This example is intended to illustrate the algorithm's capability of generating an alternative (detouring) path when left turns are penalized by a higher cost than the other two possible actions. When run with the settings defined in `assignment.py` without modification, a successful implementation shall generate the following output,

```
[[' ', ' ', ' ', 'R', '#', 'R'],
 [' ', ' ', ' ', '#', ' ', '#'],
 ['*', '#', '#', '#', '#', 'R'],
 [' ', ' ', ' ', '#', ' ', ' '],
 [' ', ' ', ' ', '#', ' ', ' ']]
```

because of the prohibitively high cost associated with a left turn.

You are highly encouraged to experiment with different (more complex) maps and different cost settings for each type of action.
