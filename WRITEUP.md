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
const float l = L / sqrtf(2.f);
const float c_bar = collThrustCmd;
const float p_bar = momentCmd.x / l;
const float q_bar = momentCmd.y / l;
const float r_bar = -momentCmd.z / kappa;
```

## Implementing rate gyro attitude integration

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

<!-- $$
\begin{align}
p_{\text{error}} &= p_c - p \\

\bar{u}_p&= k_{p-p} p_{\text{error}} \\

q_{\text{error}} &= q_c - q \\

\bar{u}_q&= k_{p-q} q_{\text{error}} \\

r_{\text{error}} &= r_c - r \\

\bar{u}_r&= k_{p-r} r_{\text{error}} \\
\end{align}
$$ -->

<div align="center"><img style="background: white;" src="https://render.githubusercontent.com/render/math?math=%5Cbegin%7Balign%7D%0Ap_%7B%5Ctext%7Berror%7D%7D%20%26%3D%20p_c%20-%20p%20%5C%5C%0A%0A%5Cbar%7Bu%7D_p%26%3D%20k_%7Bp-p%7D%20p_%7B%5Ctext%7Berror%7D%7D%20%5C%5C%0A%0Aq_%7B%5Ctext%7Berror%7D%7D%20%26%3D%20q_c%20-%20q%20%5C%5C%0A%0A%5Cbar%7Bu%7D_q%26%3D%20k_%7Bp-q%7D%20q_%7B%5Ctext%7Berror%7D%7D%20%5C%5C%0A%0Ar_%7B%5Ctext%7Berror%7D%7D%20%26%3D%20r_c%20-%20r%20%5C%5C%0A%0A%5Cbar%7Bu%7D_r%26%3D%20k_%7Bp-r%7D%20r_%7B%5Ctext%7Berror%7D%7D%20%5C%5C%0A%5Cend%7Balign%7D"></div>

Then we convert to required moments for each axis using Newton's 2nd Law for Rotation:

<!-- $$
\tau = I\alpha
$$ -->

<div align="center"><img style="background: white;" src="https://render.githubusercontent.com/render/math?math=%5Ctau%20%3D%20I%5Calpha"></div>

In code this can be implemented concisely using the V3F vector library:


```cpp
momentCmd = (pqrCmd-pqr)*kpPQR*V3F(Ixx,Iyy,Izz);
```

## Implementing prediction step elements

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

```cpp
const float b_x_actual = R(0,2);
const float b_y_actual = R(1,2);
const float r11 = R(0,0);
const float r12 = R(0,1);
const float r21 = R(1,0);
const float r22 = R(1,1);
const float inv_r33 = 1.f/R(2,2);

const float inv_c = -mass/collThrustCmd;
```

The Roll-Pitch controller is a first order controller (P-only). It calculates target rotation matrix element first derivatives as per the following equations:

<!-- <!-- $$
\begin{align}
\dot{b}^x_c  &= k_p(b^x_c - b^x_a) \\
\dot{b}^y_c  &= k_p(b^y_c - b^y_a) \\
\end{align}
$$ -->

<div align="center"><img style="background: white;" src="https://render.githubusercontent.com/render/math?math=%5Cbegin%7Balign%7D%0A%5Cdot%7Bb%7D%5Ex_c%20%20%26%3D%20k_p(b%5Ex_c%20-%20b%5Ex_a)%20%5C%5C%0A%5Cdot%7Bb%7D%5Ey_c%20%20%26%3D%20k_p(b%5Ey_c%20-%20b%5Ey_a)%20%5C%5C%0A%5Cend%7Balign%7D"></div>


```cpp
const float b_x_c_target = CONSTRAIN(accelCmd.x*inv_c,-sinf(maxTiltAngle),sinf(maxTiltAngle));
const float b_y_c_target = CONSTRAIN(accelCmd.y*inv_c,-sinf(maxTiltAngle),sinf(maxTiltAngle));
const float b_x_c_dot = kpBank*(b_x_c_target - b_x_actual);
const float b_y_c_dot = kpBank*(b_y_c_target - b_y_actual);
```

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

<!-- $$
\begin{pmatrix} p_c \\ q_c \\ \end{pmatrix}  = \frac{1}{R_{33}}\begin{pmatrix} R_{21} & -R_{11} \\ R_{22} & -R_{12} \end{pmatrix} \times \begin{pmatrix} \dot{b}^x_c \\ \dot{b}^y_c  \end{pmatrix}
$$ -->

<div align="center"><img style="background: white;" src="https://render.githubusercontent.com/render/math?math=%5Cbegin%7Bpmatrix%7D%20p_c%20%5C%5C%20q_c%20%5C%5C%20%5Cend%7Bpmatrix%7D%20%20%3D%20%5Cfrac%7B1%7D%7BR_%7B33%7D%7D%5Cbegin%7Bpmatrix%7D%20R_%7B21%7D%20%26%20-R_%7B11%7D%20%5C%5C%20R_%7B22%7D%20%26%20-R_%7B12%7D%20%5Cend%7Bpmatrix%7D%20%5Ctimes%20%5Cbegin%7Bpmatrix%7D%20%5Cdot%7Bb%7D%5Ex_c%20%5C%5C%20%5Cdot%7Bb%7D%5Ey_c%20%20%5Cend%7Bpmatrix%7D"></div>

```cpp
pqrCmd.x = inv_r33*(r21*b_x_c_dot - r11*b_y_c_dot);
pqrCmd.y = inv_r33*(r22*b_x_c_dot - r12*b_y_c_dot);
```

At this point with tuning the controller was able to meet the requirements of Scenario X:

<p align="center">
<img src="writeup/todo.gif" width="500"/>
</p>


## Implementing magnetometer update

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

```cpp
const float error = posZCmd - posZ;
const float error_dot = velZCmd - velZ;
const float u1_bar = kpPosZ*error + kpVelZ*error_dot + KiPosZ*integratedAltitudeError + accelZCmd;
```


## Implementing GPS update

Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

```cpp
if (velCmd.magXY() > maxSpeedXY) {
  velCmd = velCmd*(maxSpeedXY/velCmd.magXY());
}
```


```cpp
if (accelCmd.magXY() > maxAccelXY) {
  accelCmd = accelCmd*(maxAccelXY/accelCmd.magXY());
}
```






