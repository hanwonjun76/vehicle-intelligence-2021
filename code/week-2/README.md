# Week 2 - Markov Localization

---

[//]: # (Image References)
[plot]: ./markov.gif

## Assignment

- 처음 위치에 대한 probability를 '0'으로 초기화 한다.
- Motion Model은 Markov assumption으로 직전 위치의 확률과 오브젝트의 이동으로 현재 위치를 예측한다.
- 현재의 위치값 position-i, 평균 이동거리 mov, 이동에 대한 표준 편차 stev를 통해 확률을 gaussian distribution 형태로 만들어 주었다.
- postion_prob는 total probability 값을 계산해 준 변수이다. 


~~~
def motion_model(position, mov, priors, map_size, stdev):
    # Initialize the position's probability to zero.
    position_prob = 0.0


    probs_ay =[]

    for i in range(map_size):
     
        w = norm_pdf(position-i, mov, stdev)*priors[i]
        probs_ay.append(w) 
    
    position_prob = sum(probs_ay)
~~~    
    
- observation model의 경우 motion model에 대한 uncertainty를 보완해주는 계측 모델 값이다.
- landmark에 대한 관측값이 0이거나, 보다 많은 수가 관측되면 신뢰성 없음으로 간주하게 된다. 

def observation_model(landmarks, observations, pseudo_ranges, stdev):
    # Initialize the measurement's probability to one.
    distance_prob = 1.0
    

    if len(observations)==0 or len(observations)> len(pseudo_ranges): 
        distance_prob =0.0
    else:

        for i in range(len(observations)):
            d = observations[i]
            mu = pseudo_ranges[i]
            sig = stdev
            L = norm_pdf(d, mu, sig)
            distance_prob *= L



![Expected Result of Markov Localization][plot]

If you run the program (`main.py`) without any modification to the code, it will generate only the frame of the above plot because all probabilities returned by `motion_model()` are zero by default.
