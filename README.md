**Self-Driving Car Engineer Nanodegree**

**Term2 – Project5: Model Predictive Control (MPC)**

![](./media/image1.png){width="6.645449475065617in"
height="3.813953412073491in"}

**INTRODUCTION**

The purpose of the project is to implement Model Predictive Control
(MPC) for UDACITY’s car simulator. The path waypoints are given by the
simulator depending on the location of the vehicle and there is 100msec
latency between actuation commands (besides connection latency).

**Model Predictive Control (MPC)**

MPC is an optimization problem, for this project solves the optimal
trajectory. It involves simulating different actuator inputs to predict
the resulting trajectory and then selects the one with minimum cost. The
advantage of MPC over PID is that it can deal with latency much more
effectively, by considering explicitly the latency.

For example, on this project we know the current state and reference
trajectory we want to follow. We then optimize the actuators at each
step in time in order to minimize the cost of the predicted trajectory.
Once we’ve found the lowest cost trajectory, we implement the very first
set of actuation commands. The rest of the actuation set is thrown away.
Take the new state and use it to calculate a new optimal trajectory
again. The solver used is IPOPT.

**MPC implementation **

1.  The simulator gives us the current state of the vehicle:

**x, y, psi, speed**

1.  The simulator passes the reference trajectory that we need to follow
    as a series of way points. The points are transformed into the car’s
    coordinate system:

    ptsx\_car\[i\] = x \* cos(-psi) - y \* sin(-psi);

    ptsy\_car\[i\] = x \* sin(-psi) + y \* cos(-psi);

A third order polynomial fit is done:

auto coeffs = polyfit(ptsx\_car, ptsy\_car, 3);

3-We run the simulation using the vehicle model starting from the
current state for the duration of the latency. Also, the Cross Track
Error (CTE) and orientation error (EPSI) are calculated. This is the new
initial state for MPC:

const double dt = latency / 1000.0;

// Latency - predict state

// After transform psi is 0 =&gt; cos(0)=1 and sin(0)=0

// simplifies p\_px, p\_py and p\_psi predictions

double p\_px = 0.0 + 1.0 \* v \* dt;

double p\_py = 0.0 + 0.0 \* v \* dt; // will be 0.0

double p\_psi = 0.0 - v \* (delta \* deg2rad(25)) / Lf \* dt;

double p\_v = v + a \* dt;

double p\_cte = polyeval(coeffs, p\_px);

double p\_epsi = atan(2\*coeffs\[2\]\*p\_px + coeffs\[1\]);

4-The new state is passed to the MPC class which solves for the steering
and throttle actuators

auto sol = mpc.Solve(state,coeffs);

//const double Lf = 2.67;

double steer\_value = sol\[0\] / (deg2rad(25) \* 1.0) \* -1;

double throttle\_value = sol\[1\];

**FG\_eval class **

Defined in MPC.cpp, builds the vector ‘fg’ with the cost constraints. It
uses the equations of motion, the polynomial fit coefficients, and the
desired constraints:

// CONSTRAINT to mininimize goes in fg\[0\]

fg\[0\] = 0;

// Minimize cost for reference state variables

for (int t = 0; t &lt; N; t++){

fg\[0\] += 1 \* CppAD::pow(vars\[cte\_start + t\], 2);

fg\[0\] += 1 \* CppAD::pow(vars\[epsi\_start + t\], 2);

fg\[0\] += 1 \* CppAD::pow(vars\[v\_start + t\] - ref\_v, 2);

}

// Minimize cost for actuator variables

for (int t = 0; t &lt; N -1; t++){

fg\[0\] += 1 \* CppAD::pow(vars\[delta\_start + t\], 2);

fg\[0\] += 1 \* CppAD::pow(vars\[a\_start + t\], 2);

}

// Minimize the value gap between sequential actuations

for (int t = 0; t &lt; N - 2; t++){

fg\[0\] += 500 \* CppAD::pow(vars\[delta\_start + t + 1\] -
vars\[delta\_start + t\], 2);

fg\[0\] += 1 \* CppAD::pow(vars\[a\_start + t + 1\] - vars\[a\_start +
t\], 2);

}

// ALL OTHER CONSTRAINTS

// Initial constraints

fg\[1 + x\_start\] = vars\[x\_start\];

fg\[1 + y\_start\] = vars\[y\_start\];

fg\[1 + psi\_start\] = vars\[psi\_start\];

fg\[1 + v\_start\] = vars\[v\_start\];

fg\[1 + cte\_start\] = vars\[cte\_start\];

fg\[1 + epsi\_start\] = vars\[epsi\_start\];

// Kinematic Model Equations of Motion , CTE and EPSI

for (int t = 1; t &lt; N; t++) {

// f(xt)

AD&lt;double&gt; f0 = coeffs\[0\] + coeffs\[1\] \* x0;

// desired psi ( not know yet, therefore use the derivative of f(xt) )

AD&lt;double&gt; psides0 = CppAD::atan(coeffs\[1\]);

fg\[1 + x\_start + t\] = x1 - (x0 + v0 \* CppAD::cos(psi0) \* dt);

fg\[1 + y\_start + t\] = y1 - (y0 + v0 \* CppAD::sin(psi0) \* dt);

fg\[1 + psi\_start + t\] = psi1 - (psi0 + v0 \* delta0 / Lf \* dt);

fg\[1 + v\_start + t\] = v1 - (v0 + a0 \* dt);

fg\[1 + cte\_start + t\] = cte1 - ((f0 - y0) + (v0 \* CppAD::sin(epsi0)
\* dt));

fg\[1 + epsi\_start + t\] = epsi1 - ((psi0 - psides0) + v0 \* delta0 /
Lf \* dt);

}

**MPC::solve method **

Defined in MPC.cpp. Defines vars, lower/upper bounds for vars and
constrains:

-Vector ‘vars’ for state and actuator variables:

// Initial value of the independent variables.

// SHOULD BE 0 besides initial state.

Dvector vars(n\_vars);

for (int i = 0; i &lt; n\_vars; i++) {

vars\[i\] = 0.0;

}

// Set the initial variable values

vars\[x\_start\] = x;

vars\[y\_start\] = y;

vars\[psi\_start\] = psi;

vars\[v\_start\] = v;

vars\[cte\_start\] = cte;

vars\[epsi\_start\] = epsi;

-Vars lower and upper bounds:

Dvector vars\_lowerbound(n\_vars);

Dvector vars\_upperbound(n\_vars);

// Set all non-actuators upper and lowerlimits

// to the max negative and positive values.

for (int i = 0; i &lt; delta\_start; i++) {

vars\_lowerbound\[i\] = -1.0e19;

vars\_upperbound\[i\] = +1.0e19;

}

// The upper and lower limits of delta are set to -25 and 25

// degrees (values in radians).

for (int i = delta\_start; i &lt; a\_start; i++) {

vars\_lowerbound\[i\] = -0.436332;

vars\_upperbound\[i\] = +0.436332;

}

// Acceleration/decceleration upper and lower limits.

for (int i = a\_start; i &lt; n\_vars; i++) {

vars\_lowerbound\[i\] = -1.0;

vars\_upperbound\[i\] = +1.0;

}

-Constraints lower and upper bounds:

// Lower and upper limits for the constraints

// Should be 0 besides initial state.

Dvector constraints\_lowerbound(n\_constraints);

Dvector constraints\_upperbound(n\_constraints);

for (int i = 0; i &lt; n\_constraints; i++) {

constraints\_lowerbound\[i\] = 0;

constraints\_upperbound\[i\] = 0;

}

// Initial state not zero

constraints\_lowerbound\[x\_start\] = x;

constraints\_lowerbound\[y\_start\] = y;

constraints\_lowerbound\[psi\_start\] = psi;

constraints\_lowerbound\[v\_start\] = v;

constraints\_lowerbound\[cte\_start\] = cte;

constraints\_lowerbound\[epsi\_start\] = epsi;

// Initial state not zero

constraints\_upperbound\[x\_start\] = x;

constraints\_upperbound\[y\_start\] = y;

constraints\_upperbound\[psi\_start\] = psi;

constraints\_upperbound\[v\_start\] = v;

constraints\_upperbound\[cte\_start\] = cte;

constraints\_upperbound\[epsi\_start\] = epsi;

It also calls for the FG\_eval class and performs the optimization for
the new set of actuator constraints:

// creates FG object for solver

FG\_eval fg\_eval(coeffs);

// solve the problem

CppAD::ipopt::solve&lt;Dvector, FG\_eval&gt;(

options, vars, vars\_lowerbound, vars\_upperbound,
constraints\_lowerbound,

constraints\_upperbound, fg\_eval, solution);

**Conclusions**

-   The MPC was implemented for going around the track at a speed
    of 40MPH. The only constraint weight that was changed was the steer
    rate weight, which was set to 500. The car was able to go around the
    track indefinitely.

-   For other speeds, the constraint weights would have to be adjusted.

-   Also, for other speeds the latency delay could be different. For
    example, for tires, relaxation length (and the time to achieve it)
    reduces as the driving speed increases.

-   Turning the latency kinematic model made it very difficult for the
    car to stay on the track.


