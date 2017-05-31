### Model Predictive Control

The goal of the project is to develop a Model Predictive Controller to drive a car around the track in a simulator. The simulator provides vehicle position, speed and orientation in global coordinate system. In addition it also provides actuator values for steering angle and throttle, along with the waypoints of a reference trajectory that the car is supposed to follow.

#### The Model

This project uses the Global Kinematic Model presented in the lectures. The state of the vehicle is represented by a six dimensional vector [x, y, psi, v, cte, epsi] where x and y are vehicle coordinates, psi is vehicle orientation, v is vehicle speed, cte is cross track error and epsi is orientation error.

The kinematic model is described by the following equations:
```
x[t+1] = x[t] + v * cos(psi[t]) * dt
y[t+1] = y[t] + v * sin(psi[t]) * dt
psi[t+1] = psi[t] + v[t]/Lf * delta * dt
v[t+1]  = v[t] + a * dt
cte[t+1] = cte[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = epsi[t] + v[t]/Lf * delta * dt
```
where Lf is the distance between the front of the vehicle and the center of gravity and delta and a are the actuators or control inputs to our vehicle.

#### Timestep Length and Elapsed duration

The prediction horizon T, is the duration over which future predictions are made and is represented by the number of steps N times duration of each step dt. Ideally we want to make predictions into the future as far as we can but at the same time the environment should not change too much within the prediction horizon. Also we want our dt to be as small as possible so the prediction is very smooth but making dt too small will increase computation cost.

After trying different combinations the selected values for this project were N = 10 and dt = 50 ms so the prediction horizon is 0.5 seconds and it covers a distance of approximately 15 meters at 75 mph. These values were found to perform well at speeds between 30 mph and 75 mph (I did not test smaller speeds because it is boring).

Other values attempted were:
 -  N = 20  and dt = 50 ms  - the car completed the track at 45 mph but not at 70 mph
 - N = 10 and dt = 100 ms  - the car left the track at 45 mph.

#### Polynomial Fitting and MPC Preprocessing

The simulator provides waypoints in the global coordinate system. These values were first converted to car coordinate system then were fitted to a 3rd degree polynomial.

#### Model Predictive Control with Latency

This project assumes a latency of 100 ms between the MPC computation and applying the actuator values. To simulate this latency we first calculated the vehicle state after 100 ms using the kinematic model. This state was then used as input to MPC algorithm to determine the actuator values. These values were applied with 100 ms latency.

### Conclusion

This was a fun project! I learned that MPC performs better compared to PID Controller. The video below shows the car driving around the track using a reference speed of 70 mph.

[![Model Predictive Control](https://img.youtube.com/vi/4bfoB_0QJXk/0.jpg)](https://youtu.be/4bfoB_0QJXk)

Note: I increased the weight of steering angle changes between adjacent intervals by a factor of 1000 in order to avoid sudden changes in steering angle. This is important to ensure a smooth trajectory.
