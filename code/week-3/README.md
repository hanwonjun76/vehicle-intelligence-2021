# Week 3 - Kalman Filters, EKF and Sensor Fusion
-자코비안 함수를 통해 obsesrvation model을 linearliztion 해주었다.
H_j = Jacobian(self.x)
-위에서 linearlization된 H_j matrix와 공분산 matrix P, measurment noise matrix를 이용하여 S matrix를 구하고 이를 통해 kalman gain을 얻을 수 있다.
S = np.dot(np.dot(H_j, self.P), H_j.T) + self.R
K = np.dot(np.dot(self.P, H_j.T), np.linalg.inv(S)) 

- measurement model과 non-linear model식을 이용하여 두 model사이의 오차값을 계산 해 준다. 
h = np.array([
            [sqrt(self.x[0]*self.x[0] + self.x[1]*self.x[1])],
            [atan2(self.x[1],self.x[0])],
            [(self.x[0]*self.x[2]+self.x[1]*self.x[3])/(sqrt(self.x[0]*self.x[0] + self.x[1]*self.x[1]))]
        ], dtype=np.float32) #shape (3,3)
        
        h = np.ravel(h)
        # z1 = np.array([[z[0],],
        # [z[1],],
        # [z[2],]
        # ])
    
        y = z-h #shpae (3,1)
        
 - 차량의 위치와 map coordinate가 이루는 각을 phi라고 정하고 이 phi의 범위를 -phi<x<phi로 normalization해준다. 
    if y[1]> np.pi:
            y[1] = y[1]- 2*np.pi

        elif y[1]<-np.pi:
            y[1] = y[1] + 2*np.pi
         


- 마지막으로 위에서 구한 kalman gain을 통해 state model과 observation model사이의 weight를 주어 자차의 위치를 estimation하게 된다.
        self.x = self.x + np.dot(K, y)
        #    P = (I - K * H_j) * P
        self.P = self.P - np.dot(np.dot(K, H_j), self.P)

[//]: # (Image References)
[kalman-result]: ./kalman_filter/graph.png
[EKF-results]: ./EKF/plot.png

## Kalman Filter Example

In directory [`./kalman_filter`](./kalman_filter), a sample program for a small-scale demonstration of a Kalman filter is provided. Run the following command to test:

```
$ python testKalman.py
```

This program consists of four modules:

* `testKalman.py` is the module you want to run; it initializes a simple Kalman filter and estimates the position and velocity of an object that is assumed to move at a constant speed (but with measurement error).
* `kalman.py` implements a basic Kalman fitler as described in class.
* `plot.py` generates a plot in the format shown below.
* `data.py` provides measurement and ground truth data used in the example.

The result of running this program with test input data is illustrated below:

![Testing of Kalman Filter Example][kalman-result]

Interpretation of the above results is given in the lecture.

In addition, you can run `inputgen.py` to generate your own sample data. It will be interesting to experiment with a number of data sets with different characteristics (mainly in terms of variance, i.e., noise, involved in control and measurement).

---

## Assignment - EFK & Sensor Fusion Example

In directory [`./EKF`](./EKF), template code is provided for a simple implementation of EKF (extended Kalman filter) with sensor fusion. Run the following command to test:

```
$ python run.py
```

The program consists of five modules:

* `run.py` is the modele you want to run. It reads the input data from a text file ([data.txt](./EKF/data.txt)) and feed them to the filter; after execution summarizes the result using a 2D plot.
* `sensor_fusion.py` processees measurements by (1) adjusting the state transition matrix according to the time elapsed since the last measuremenet, and (2) setting up the process noise covariance matrix for prediction; selectively calls updated based on the measurement type (lidar or radar).
* `kalman_filter.py` implements prediction and update algorithm for EKF. All the other parts are already written, while completing `update_ekf()` is left for assignment. See below.
* `tools.py` provides a function `Jacobian()` to calculate the Jacobian matrix needed in our filter's update algorithm.
*  `plot.py` creates a 2D plot comparing the ground truth against our estimation. The following figure illustrates an example:

![Testing of EKF with Sensor Fusion][EKF-results]

### Assignment

Complete the implementation of EKF with sensor fusion by writing the function `update_ekf()` in the module `kalman_filter`. Details are given in class and instructions are included in comments.
