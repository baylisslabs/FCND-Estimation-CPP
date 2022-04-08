# Project: Building an Estimator

This is the writeup for my implementation of the *Building an Estimator* in C++. In each section below I address all the required rubric items. The animation below shows the final result: the quadrotor follows the test trajectory succesfully.

<p align="center">
<img src="writeup/todo.gif" width="500"/>
</p>


## Determining measurement noise

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

$
\begin{align}
\hat{\sigma}^2 = {1\over n} \sum_{i=1}^{n} (x_i-\hat{x})^2
\end{align}
$

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt

```cpp
gps_xs = np.genfromtxt("../config/log/Graph1.txt",delimiter=",",skip_header=True)
sigma = np.std(gps_xs[:,1])
x_bar = np.mean(gps_xs[:,1])

imu_axs = np.genfromtxt("../config/log/Graph2.txt",delimiter=",",skip_header=True)
sigma = np.std(imu_axs[:,1])
x_bar = np.mean(imu_axs[:,1])
```

## Implementing rate gyro attitude integration

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

$
\begin{align}
\bar{q_t} = dq*q_t \\
\bar{\theta_t}=Pitch(\bar{q_t}) \\
\bar{\phi_t}=Roll(\bar{q_t})
\end{align}
$

```cpp
auto attitudeEst = Quaternion<float>::FromEuler123_RPY(rollEst,pitchEst,ekfState(6));
auto attitudePred = attitudeEst.IntegrateBodyRate(gyro,dtIMU);
auto predictedPitch = attitudePred.Pitch();
auto predictedRoll = attitudePred.Roll();
ekfState(6) = attitudePred.Yaw();
```

## Implementing prediction step elements

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

$
\begin{align}
x_{t} =
 \begin{bmatrix}
    x \\
    y \\
    z \\
    \dot{x} \\
    \dot{y} \\
    \dot{z} \\
    \psi
  \end{bmatrix} \\
\end{align}
$

$
\begin{align}
\bar{x_t} = \hat{x_{t-1}} +
 \begin{bmatrix}
    \dot{x} \\
    \dot{y} \\
    \dot{z} \\
    \ddot{x} \\
    \ddot{y} \\
    \ddot{z} \\
    0
  \end{bmatrix} \varDelta T
\end{align}
$


Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.

```cpp
auto accelInI = attitude.Rotate_BtoI(accel) + V3F(0,0,-CONST_GRAVITY);
VectorXf dState(QUAD_EKF_NUM_STATES);
dState(0) = curState(3);
dState(1) = curState(4);
dState(2) = curState(5);
dState(3) = accelInI.x;
dState(4) = accelInI.y;
dState(5) = accelInI.z;
dState(6) = 0;
predictedState = curState + (dState * dt);
```

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.

TODO: Fix code

$
\begin{align}
R'_{bg} = \begin{bmatrix}
  -cos(\theta)sin(\psi) & -sin(\phi)sin(\theta)sin(\psi) - cos(\theta)sin(\psi) &  -sin(\phi)sin(\theta)sin(\psi) + sin(\theta)cos(\psi) \\
  sin(\theta)sin(\psi) & sin(\phi)sin(\theta)sin(\psi) - sin(\theta)sin(\psi) & sin(\phi)sin(\theta)sin(\psi) + sin(\theta)sin(\psi) \\
  0 & 0 & 0
  \end{bmatrix}
\end{align}
$

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.


```cpp
const auto phi = roll;
const auto theta = pitch;
const auto psi = yaw;
RbgPrime(0,0) = -cosf(theta)*sinf(psi);
RbgPrime(0,1) = -sinf(phi)*sinf(theta)*sinf(psi) - cosf(theta)*cosf(psi);
RbgPrime(0,2) = -cosf(phi)*sinf(theta)*sinf(psi) + sinf(theta)*cosf(psi);

RbgPrime(1,0) = cosf(theta)*cosf(psi);
RbgPrime(1,1) = sinf(phi)*sinf(theta)*cosf(psi) - cosf(theta)*sinf(psi);
RbgPrime(1,2) = cosf(phi)*sinf(theta)*cosf(psi) + sinf(theta)*sinf(psi);
```

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.

```cpp
auto attitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
auto gravity = V3F(0,0,-CONST_GRAVITY);
auto accelInB = accel + attitude.Rotate_ItoB(gravity);
```

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.

$
\begin{align}
G_t = g'(x_t,u_t,\varDelta t) = \begin{bmatrix}
  1 & 0 & 0 & \varDelta t & 0           & 0           & 0 \\
  0 & 1 & 0 & 0           & \varDelta t & 0           & 0 \\
  0 & 0 & 1 & 0           & 0           & \varDelta t & 0 \\
  0 & 0 & 0 & 1           & 0           & 0           & R'_{bg}[0:]u_t[0:3]\varDelta t \\
  0 & 0 & 0 & 0           & 1           & 0           & R'_{bg}[1:]u_t[0:3]\varDelta t \\
  0 & 0 & 0 & 0           & 0           & 1           & R'_{bg}[2:]u_t[0:3]\varDelta t \\
  0 & 0 & 0 & 0           & 0           & 0           & 1 \\
  \end{bmatrix}
\end{align}
$

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.

```cpp
for(int i=0;i<3;++i) {
  gPrime(i,i+3) = dt;
}
```

```cpp
VectorXf ut(3);
ut << accelInB.x, accelInB.y, accelInB.z;
for(int i=0;i<3;++i) {
  gPrime(i+3,6) = RbgPrime.row(i).dot(ut)*dt;
}
```

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.
$
\begin{align}
\bar{\Sigma_T}=G_t\Sigma_{t-1}G_t^T+Q_t
\end{align}
$

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.

```cpp
ekfCov = gPrime * ekfCov;
gPrime.transposeInPlace();
ekfCov = ekfCov * gPrime + Q;
```


## Implementing magnetometer update

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

$
\begin{align}
z_t = \begin{bmatrix}
  \psi
  \end{bmatrix}
\end{align}
$
$
\begin{align}
h(x_t) = \begin{bmatrix}
  x_{t,\psi}
  \end{bmatrix}
\end{align}
$
$
\begin{align}
h'(x_t) = \begin{bmatrix}
  0 & 0 & 0 & 0 & 0 & 0 & 1
  \end{bmatrix}
\end{align}
$
$
\begin{align}
normalise \space h(x_t) : -\pi <= z_t[0] - h(x_t)[0] < \pi
\end{align}
$


Lorem ipsum dolor sit amet, consectetur adipiscing elit

```cpp
hPrime(0,6) = 1;
zFromX(0) = z(0) - AngleNormF(magYaw - ekfState(6));
```


## Implementing GPS update

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

$
\begin{align}
z_t = \begin{bmatrix}
  x \\
  y \\
  z \\
  \dot{x} \\
  \dot{y} \\
  \dot{z} \\
  \end{bmatrix}
\end{align}
$
$
\begin{align}
h(x_t) = \begin{bmatrix}
  x_{t,x} \\
  x_{t,y} \\
  x_{t,z} \\
  x_{t,\dot{x}} \\
  x_{t,\dot{y}} \\
  x_{t,\dot{z}} \\
  \end{bmatrix}
\end{align}
$
$
\begin{align}
h'(x_t) = \begin{bmatrix}
  1 & 0 & 0 & 0 & 0 & 0 & 0 \\
  0 & 1 & 0 & 0 & 0 & 0 & 0 \\
  0 & 0 & 1 & 0 & 0 & 0 & 0 \\
  0 & 0 & 0 & 1 & 0 & 0 & 0 \\
  0 & 0 & 0 & 0 & 1 & 0 & 0 \\
  0 & 0 & 0 & 0 & 0 & 1 & 0 \\
  \end{bmatrix}
\end{align}
$

```cpp
hPrime.leftCols(6).setIdentity();
zFromX = ekfState.head(6);
```






