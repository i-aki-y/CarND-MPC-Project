## MPC Project

This is my MPC Project write-up

### Model

In this project, I implemented "Model Predictive Control" (MPC) model. The MPC model uses state variables such as current position and yaw rate as input and provides actuator variables as outputs which control next car sates. Optimal actuator variables are selected by minimizing the cost function in some constraint conditions. Applying a kinematic model as the constraint conditions, we can obtain the optimal variables which follow the model update equations.

#### States

I use the following states as MPC input:

1. ($x$, $y$): Current car's position in map coordinate.
2. $\psi$: Current car heading angle from $x$ axis. Rounding counterclockwise means increasing $\psi$.
3. $v$: Current speed of the car.
4. $cte$: Cross track error.
5. $e\psi$: Difference between designed angle $\psi des$ (road orientation) and current car's heading angle.

The variables $x, y, \psi, v$ are obtained from measurements, while the variables $cte, e\psi$ are calculated from given waypoints which represent the road positions.

#### Actuators

Actuators are outputs of this model. they are used to control the next car states. The following variables are actuators:

1. $\delta$: steering angle.
2. $a$: the throttle of the car which normalized to [-1, 1].

#### Update equations
Update equations are used as constraint functions. In this project, the following kinematic model is used.

$$
\begin{aligned}
x_{t+1} &= x_{t} + v_{t} \cos(\psi_t) dt \\
y_{t+1} &= y_{t} + v_{t} \sin(\psi_t) dt \\
\psi_{t+1} &= \psi_{t} + \frac{v_t}{L_f} \delta_t dt\\
v_{t+1} &= v_{t} + a_{t} dt \\
cte_{t+1} &=  f(x_t) - y_t + v_t \sin(e\psi_{t}) dt\\
e\psi_{t+1} &= \psi_{t} - \psi des_{t} + \frac{v_t}{L_f} \delta_t dt,
\end{aligned}
$$
where $L_f$ is the distance between the front of the vehicle and its center of gravity, $f(x_t)$ represents the function of reference trajectory which is obtained by applying polynomial fitting to the given waypoints.

### Cost function
Cost function is used for objective function of optimization problem. I used the following cost function:
$$
J = w_{cte} cte_{t}^2 + w_{e\psi} e\psi_{t}^2 + w_{v}(v_{t} - v_{\rm{ref}})^2 + w_{\delta}\delta_{t}^2 + w_{a} a_{t}^2 + w_{\Delta \delta} (\delta_{t} - \delta_{t-1})^2 + w_{\Delta a}(a_{t} - a_{t-1})^2,
$$
Especially, the variable $v_{\rm{ref}}$ is used to control the car speed. The hyper-parameters $w_{cte}, w_{e\psi}, w_{v}, w_{\delta}, w_{a}, w_{\Delta \delta}, w_{\Delta a} (> 0)$ are introduced to control the importance of each term. The actual parameters are defined in line 27-37 of `MPC.cpp`.

## Timestep Length and Elapsed Duration (N & dt)
In this MPC, the optimizer predicts future car states up to time $T$ according to a given update equation (in discrete time) and uses these states to evaluate the best actuators.
The time $T$ is defined by two hyper-parameters $T = n * dt$, where $n$ is a number of prediction step and $dt$ is a time interval between two consecutive predictions.

If $T$ is fixed, an increase of $n$ (decrease of $dt$) will improve the accuracy. But a large $n$ requires more calculation cost. Therefore, there is a trade-off between accuracy and calculation cost. If $T$ is too small, the predicted area is too short. In that case, the model fails to take into account the advantage of future prediction. On the other hand, the large $T$ may have another disadvantage. In that case, optimizer improves accuracy of a distant area at the expense of local area accuracy. Usually, the accuracy around the local area is more important than distant area. Therefore this may make the car unstable.

I think that an appropriate combination of $n$ and $dt$ is depends on the road track shape and desired car speed. In this project, the car speed is at most $\sim 100 $ [mph] (= 44.7 [m/s]). The car displacement in $dt=0.1$ [sec] is approximately 4.47 [m]. I think $n=10$, $dt=0.1$[sec] ($T=1$ [sec]) is reasonable starting point. To obtain better values, I tried with some different combinations.

|T| n | dt |
|-| - | - |
|1.0 | 10  | 0.1 |
|1.2 | 10  | 0.12  |
|1.5 | 10  | 0.15  |
|1.0 | 20  | 0.05  |

In my case, the parameter $n=10, dt = 0.15 (T=1.0)$ gives stable result.


## Polynomial Fitting and MPC Preprocessing
To calculate the $cte_{t}, e\psi$ variables, I need to obtain the reference trajectory function $f(x)$. I applied the 3 degree polynomial function fitting to the given waypoints. An approximated function is described by using 4 coefficients ($c_0, ..., c_3$):

$$
f(x) = c_3 x^3 + c_2 x^2 + c_1 x + c_0.
$$

Approximation by using the function $f(x)$ will fail when the road close to vertical angle. To keep the approximation applicable we need to transform the coordinate from map coordinate to car coordinate. I applied a coordinate transformation before the fitting.
$$
\begin{aligned}
x_p' &=  (x_p - x_c)\{\cos(-\psi) - \sin(-\psi)\}\\
y_p' &=  (y_p - y_c)\{\sin(-\psi) + \cos(-\psi)\},
\end{aligned}
$$
where $(x_p, y_p)$ and $(x_p', y_p')$ are waypoint position in map and car coordinate, respectively, and $(x_c, y_c)$ is car position in map coordinate.

After the coordinate transformation, the origin of the coordinate is located on the car position $(x_c, y_c) = (0, 0)$, and the car heading direction is parallel to the x-axis ($\psi=0$). Therefore, the $cte_t, e\psi$ are obtained by the following equations:

$$
\begin{aligned}
cte_t   &= f(x=0) - y_c = c_0,\\
e\psi_t &= \tan^{-1}(f'(x=0)) - \psi = tan^{-1}(c_1)
\end{aligned}
$$

## Model Predictive Control with Latency

The existence of latency causes the mismatch between the car states in which the optimal actuators are derived and the states actually performed the derived actuators. This mismatch makes car unstable especially when the car's speed is increased.
In order to avoid that mismatch, I predicted the future car states after the latency time. I also use the kinematic model to obtain the predicted states.

$$
\begin{aligned}
x_{pred} &= x_{t} + v_{t} \cos(\psi_t) dt \\
y_{pred} &= y_{t} + v_{t} \sin(\psi_t) dt \\
\psi_{pred} &= \psi_{t} + \frac{v_t}{L_f} \delta_t dt \\
v_{pred} &= v_{t} + a_{t} dt \\
cte_{pred} &=  f(x_{pred}) - y_{pred} \\
e\psi_{pred} &= \psi_{pred} - \psi des_{pred}
\end{aligned}
$$

 I applied the predicted states to the MPC as inputs instead of the current sates.
