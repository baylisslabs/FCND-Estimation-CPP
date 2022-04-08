# Project: Building an Estimator

This is the writeup for my implementation of the *Building an Estimator* in C++. In each section below I address all the required rubric items. The animation below shows the final result: the quadrotor follows the test trajectory succesfully.

<p align="center">
<img src="writeup/todo.gif" width="500"/>
</p>


## Determining measurement noise

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.


<!-- $$
\begin{align}
F_{total} &= F_1 + F_2 + F_3 + F_4 \\
\tau_x &= (F_1 + F_4 - F_2 - F_3)l \\
\tau_y &= (F_1 + F_2 - F_3 - F_4)l \\
\tau_z &= \tau_1 + \tau_2 + \tau_3 + \tau_4
\end{align}
$$ -->

<div align="center"><img style="background: white;" src="https://render.githubusercontent.com/render/math?math=%5Cbegin%7Balign%7D%0AF_%7Btotal%7D%20%26%3D%20F_1%20%2B%20F_2%20%2B%20F_3%20%2B%20F_4%20%5C%5C%0A%5Ctau_x%20%26%3D%20(F_1%20%2B%20F_4%20-%20F_2%20-%20F_3)l%20%5C%5C%0A%5Ctau_y%20%26%3D%20(F_1%20%2B%20F_2%20-%20F_3%20-%20F_4)l%20%5C%5C%0A%5Ctau_z%20%26%3D%20%5Ctau_1%20%2B%20%5Ctau_2%20%2B%20%5Ctau_3%20%2B%20%5Ctau_4%0A%5Cend%7Balign%7D"></div>


Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

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

```cpp
auto attitudeEst = Quaternion<float>::FromEuler123_RPY(rollEst,pitchEst,ekfState(6));
auto attitudePred = attitudeEst.IntegrateBodyRate(gyro,dtIMU);
auto predictedPitch = attitudePred.Pitch();
auto predictedRoll = attitudePred.Roll();
ekfState(6) = attitudePred.Yaw();
```

## Implementing prediction step elements

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

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

```cpp
auto attitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
auto gravity = V3F(0,0,-CONST_GRAVITY);
auto accelInB = accel + attitude.Rotate_ItoB(gravity);
```

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

```cpp
ekfCov = gPrime * ekfCov;
gPrime.transposeInPlace();
ekfCov = ekfCov * gPrime + Q;
```


## Implementing magnetometer update

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

```cpp
hPrime(0,6) = 1;
zFromX(0) = z(0) - AngleNormF(magYaw - ekfState(6));
```


## Implementing GPS update

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

```cpp
hPrime.leftCols(6).setIdentity();
zFromX = ekfState.head(6);
```






