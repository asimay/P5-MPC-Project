# CarND-Controls-MPC
The goals / steps of this project are the following:

* Implement Model Predictive Control to drive the car around the track, calculate CTE by yourself, and there's a 100 millisecond latency between actuations commands on top of the connection latency.

---

[//]: # (Image References)
[image1]: ./outputs/final.gif
[image2]: ./outputs/cte_plot_3500.png
[image3]: ./outputs/cte_plot_10000.png
[image4]: ./outputs/two_value_plot1.png
[image5]: ./outputs/two_value_plot2.png
[image6]: ./outputs/1-80kmh.PNG
[image7]: ./outputs/2-80kmh.PNG
[image8]: ./outputs/8.jpg
[image9]: ./outputs/9.jpg
[image10]: ./outputs/equation.png

Final result:

![alt text][image1]

---

## Dependencies

This program need:
* **Ipopt and CppAD** 
* [**matplotlib wrapper**](https://github.com/lava/matplotlib-cpp)

I've included it in head file, but you still need to setup the environment under ubuntu, installation step:

```
sudo apt-get install python-matplotlib python-numpy python2.7-dev
```

## Model Predictive Control  Process


#### 1. Read the planner data, and transform from map coordinate to odom coordinate.

The waypoints data is in global coordinate, so we must transform the data into odom coordinate, the odom coordinate's origin is vehicle's origin, and `x`, `y` is the vehicle's run distance. 

In code, I write a function `Eigen::VectorXd MapToOdom(double ptx, double pty, double px, double py, double psi)` to do the transformation.

#### 2. Fit the 3 order polynomial.

With above data, I did the 3 order polynomial for the data, this will used for fit polynomial the waypoints and MPC predictive calculation.

#### 3. Because of latency, we should add latency into state.

Do the prediction with latency:

```cpp
double predict_x = 0 + v * cos(0) * DT;
double predict_y = 0 + v * sin(0) * DT;
double predict_psi = 0 + v * (-steer_value) * DT / LF; //delta is counter-clock
double predict_v = v + throttle_value *  DT;
double predict_cte = cte + v * sin(epsi) * DT;    
double predict_epsi = epsi + v * (-steer_value) * DT / LF;
```

#### 4. Initialize the state with latency prediction state.

#### 5. Use **IPOPT** to solve the control variables's optimization.

1. MPC Model equation described as belows:

![alt text][image10]


2. In the first step, I tuned `N = 10` and `dt = 0.1`, and `ref_v = 80.0`,
and set constraints and weights for control variables & fg, and plot the data to tune the code, for example, weights of `[cte, epsi, v, delta, a, delta_change, a_change, epsi_change]` from `[3500， 2000， 1， 10， 10， 200， 10， 0.8]` to:

for steering angle weights:

  1) `[3500， 2000， 1， 10， 10， 200， 10， 0.8]`
  2) `[3500， 2000， 1， 10， 10， 500， 10， 0.8]`
  3) `[3500， 2000， 1， 10， 10， 1000， 10， 0.8]`
  4) `[3500， 2000， 1， 10， 10， 2000， 10， 0.8]`  
  5) `[3500， 2000， 1， 10， 10， 3000， 10， 0.8]`  

for cte weights:
  1) `[3500， 2000， 1， 10， 10， 3000， 10， 0.8]`  
  2) `[5500， 2000， 1， 10， 10， 3000， 10， 0.8]`    
  3) `[8500， 2000， 1， 10， 10， 3000， 10， 0.8]`     
  4) `[10000， 2000， 1， 10， 10， 3000， 10， 0.8]`   
  5) `[15000， 2000， 1， 10， 10， 3000， 10， 0.8]`   

and for every parameter tuning, I will not list at here. the data plot result examples are shown as below:  
cte from 3500 to 10000:

![alt text][image2]
![alt text][image3]

The Final tuned weights as belows:

```cpp
//weight of variables' cost
int  weight_cte_cost = 3500;
int  weight_epsi_cost = 2000;
int  weight_v_cost = 1;
int  weight_delta_cost = 10;
int  weight_a_cost = 10;
int  weight_delta_change_cost = 3000;
int  weight_a_change_cost = 10;
int  weight_epsi_change_cost = 0.8;
```

with above parameters, the car runs very well in simulation, almost always in the middle of road all the time, no ties out of road. 

The screenshot of simulation as below:

![alt text][image6]
![alt text][image7]


**Remember**, above parameter is based on `v_ref = 80, v_constraint = 100`, and `N = 10` and `dt = 0.1`.

In this situation, the vehicle is successfully run around the lake, but the vehicle's speed is equal or below **80MPH**, especially when turning, it will down to about 40MPH.

**I want to challenge myself, and hope the car can run at 100MPH.**

So I changed the base condition above, I tuned the parameter based on `v_ref = 100, v_constraint = 140`, and `N = 10` and `dt = 0.09`.

after about `more than20 groups combined parameters` tuned, I finally tuned the parameters at:



```cpp
//weight of variables' cost
int  weight_cte_cost = 30000;
int  weight_epsi_cost = 110;
int  weight_v_cost = 10;
int  weight_delta_cost = 2000;
int  weight_a_cost = 20;
int  weight_delta_change_cost = 30000;
int  weight_a_change_cost = 10;
int  weight_epsi_change_cost = 20;
```

I expand the iterater times to 200 to observe more cases, the data plot result examples are shown as below:  

![alt text][image4]
![alt text][image5]

above graph, steering values turns sharply after 100 step, it's OK, because there is a sharp turning road at that simulation time. 


#### 6. Output the MPC predicted trajectory and reference trajectory.

with new tuned parameters, the car runs good in simulation based on `v_ref = 100` condition, and the car speed can run at about **95MPH**.

The screenshot of simulation as below:

![alt text][image8]
![alt text][image9]


## Final conclusion:

MPC model is much better than PID method, and under 80MPH it also works very well. There is also another theory used for solving non-linear MPC model, I think it's much better than MPC model, will investigate it.
