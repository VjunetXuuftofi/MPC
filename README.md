# MPC Project
An MPC controller, part of the Self-Driving Car Engineer Nanodegree
Program.

---

## The Model
For this project, I chose to use the standard kinematic model covered
in the course videos. The state has an x-coordinate, a y-coordinate, a
heading, and a velocity. It also contains actuator values (acceleration
and change in heading, or `delta`) The state values are updated using standard
motion equations:
```
x = x + v * cos(psi) * delta_t
y = x + v * cos(psi) * delta_t
psi = psi + v * delta_psi / Lf * delta_t
v = v + a * delta_t
```
`Lf` is a constant based on the turning radius of the car. `delta` is
the car's steering actuation.

There are also two error components, e_psi and cte. The e_psi is the
difference between the calculated angle of the road and the heading. The
cte, is the cross-track-error, or the distance from the center of the
car to the center of the road. Both of these are calculated by comparing
the car's actual location and heading with that predicted by the fitted
polynomial of the road waypoints, in the following way:
```
cte = y_pred - y + v * sin(epsi) * delta_t
epsi = psi - atan(ypred) + v0/Lf * delta * dt
```

## Timestep Length and Elapsed Duration
The car decides where to drive by running a series of possible
actuations through an optimizer. This requires specifying how often you
want actuations to occur, as well as how long you want to predict a path
for. I chose to use 100ms as my delta_t, as it did not make sense to use
less because of the high latency and using much more made the predictions
too granular. I used 10 as my n value, which meant that the optimizer was
predicting 1 second out, which was a good value that meant the car was
able to turn well while not breaking the model each time a new waypoint
became visible.

## Polynomial Fitting and MPC Preprocessing
The first thing that I did to all measurements was to convert them from
map space into vehicle space, in the following way:
```
px_w = (px_w - px_c) * cos(psi) + (px_w - px_c) * sin(psi)
py_w = (py_w - py_c) * cos(psi) - (py_w - py_c) * sin(psi)
```
Where `(px_w, py_w)` are the coordinates of a waypoint and `(px_c, py_c)`
are the coordinates of the car. After doing this, the waypoints are in
reference to the car, which becomes the origin.

I then fit these transformed waypoints in a third degree polynomial,
which allowed for a basic continuous curve to compare with for error
calculations.

## Latency
The project has a built in 100ms latency for all actuations. I handled
this by passing the state of the vehicle as the model predicted it would
be in 100ms, namely:
```
mean_psi =  v * latency * steering_angle / Lf / 2
x = cos(mean_psi) * v * latency
y = -sin(mean_psi) * v * latency
```
The psi above is the mean psi throughout the latency, with the original
psi at 0. This allowed me to overcome the latency issues.