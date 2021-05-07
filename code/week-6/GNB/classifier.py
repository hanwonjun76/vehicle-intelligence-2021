import numpy as np
import random
from math import sqrt, pi, exp

def gaussian_prob(obs, mu, sig):
    # Calculate Gaussian probability given
    # - observation
    # - mean
    # - standard deviation
    num = (obs - mu) ** 2
    denum = 2 * sig ** 2
    norm = 1 / sqrt(2 * pi * sig ** 2)
    return norm * exp(-num / denum)

# Gaussian Naive Bayes class
class GNB():
    # Initialize classification categories
    def __init__(self):
        self.classes = ['left', 'keep', 'right']
    
    # Given a set of variables, preprocess them for feature engineering.
    def process_vars(self, X):
        return X
    
    # Train the GNB using a combination of X and Y, where
    # X denotes the observations (here we have four variables for each) and
    # Y denotes the corresponding labels ("left", "keep", "right").
    def train(self, X, Y):
        '''
        Collect the data and calculate mean and standard variation
        for each class. Record them for later use in prediction.
        '''
        # TODO: implement code.

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

       
    # Given an observation (s, s_dot, d, d_dot), predict which behaviour
    # the vehicle is going to take using GNB.
    def predict(self, observation):
        '''
        Calculate Gaussian probability for each variable based on the
        mean and standard deviation calculated in the training process.
        Multiply all the probabilities for variables, and then
        normalize them to get conditional probabilities.
        Return the label for the highest conditional probability.
        '''
        # TODO: implement code.

        prob = np.zeros(len(self.classes))

        for i in range(len(self.classes)):
            a = 1
            for j in range(self.x.shape[1]):
                a *= gaussian_prob(observation[j], self.mu[i][j], self.st[i][j])
                prob[i] = a

                res = prob / prob.sum()

        return self.classes[res.argmax(axis=0)]
