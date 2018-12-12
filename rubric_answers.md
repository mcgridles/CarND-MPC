### The Model
The model used is the Model Predictive Control (MPC) model used in the lecture.
It treats following the correct trajectory as an optimization problem and simulates different actuator values to determine the best course of action.
Then, it re-optimizes the parameters every timestep to make sure the car is always using the most up-to-date information.

The state vector consists of `{x,y,psi,v,cte,epsi}`, and the actuator vector is `{steering_angle, throttle}`.

Cost functions were modified by adding a multiplier to improve smoothness and performance which were tested with trial and error.

The update equations are exactly the same as in the lesson, but instead of using a linear model for the path and polynomial model of order 3 was used.
This allows the path to better fit most roads.

They are:

`AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);`

`AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));`
      
`fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);`

`fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);`

`fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);`

`fg[1 + v_start + t] = v1 - (v0 + a0 * dt);`

`fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));`

`fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);`

### Timestep Length and Elapsed Duration
The initial value for `N` and `dt` that were tried were 40 and 0.05 respective as in the lesson, but this caused the simulation to run very slowly.
These numbers are also unnecessary because the controller does not need to see 40 steps ahead nor does it need the granularity of `dt=0.05`.
After those were tried, a more reasonable guess of `N=10` and `dt=0.1` were tried and produced adequate results.

### Polynomial Fitting and MPC Preprocessing
The polynomial fitting is done using the `polyfit()` function as done in the lesson, however the points are first transformed from vehicle coordinates to world coordinates.
This is done using the difference between the vehicle position and the point position to know where the points are in relation to the vehicle, and the psi angle to convert to world coordinates.

Once this is done, the points can be passed to `polyfit()` as mentioned above, and were used to fit a polynomial of order 3.

### MPC with Latency
Initially, latency was dealt with by keeping the vehicle speed low enough that there was never any issue. 
In order to increase the vehicle speed however, delay needs to be built in to the state calculations. 

To do this, the initial state is first set, and then an updated state is calculated using the update equations found at `MPC.cpp:106`.
Instead of using `dt` to update, the 100ms delay is used.