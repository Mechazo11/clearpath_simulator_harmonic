# NOTES

This markdown keeps all my notes for the RoSE 25 paper which is the final project for EE 7500 MPC class.

## Paper note

* The test problem is formulated as finite-horizon optimal control problem [1]
* 

## Which vehicle model do we use?

Two are available. Either Ackermann model (from ros2_rover) or differential drive (skid-steering) controller. Excerpt from discussion with Chatgpt o1-preview are as follows

* An Ackermann vehicle model has two control inputs: steering angle $\theta$ and linear velocity $v$.
* MPPI can utilize the Ackermann vehicle model by using it as the system dynamics within the MPPI algorithm. The Ackermann model provides the kinematic equations relating steering angle and velocity to the vehicle's motion. MPPI samples control inputs (steering and velocity), predicts future states using the Ackermann model, and evaluates costs to optimize the control inputs for trajectory planning.

## Notes from Aggressive Driving with Model Predictive Path Integral Control [1]

* Algorithm 1 

<img src="figs/algorithm1_mppi_paper.png" alt="An example image" width="500" height="auto">

* Cost formulation (Look into page )
$2.5(V_{des} - V)^2 + 50.0h(p_{x}, p_{y})^2 + 40.0C + 10.0 ||\mathbf{u}||^2$


## MPPI-Generic

**TODO**


## Nav2 MPPI implementation notes

According to Chatgpt o1 preview, the cost function of Nav2's MPPI controller is as follows

* Slide showing the steps of sampling-based MPPI controller:

<img src="figs/nav2_mppi_controller.png" alt="An example image" width="500" height="auto">

* Path tracking error: $C_{\text{path}} = w_{\text{path}} \sum_{t=1}^{N} \left( \text{distance}\left( x_t, \text{path}_t \right) \right)^2$

* Obstacle Cost: $C_{\text{obstacle}} = w_{\text{obstacle}} \sum_{t=1}^{N} \text{costmap}\left( x_t \right)$

* 


---
## Paper references

* [1] Aggressive Driving with Model Predictive Path Integral Control by Grady Williams
* [2] MPOPI paper
* [3] MPPI-Generic paper

## External references

* MPPI by Autorally: https://github.com/AutoRally/autorally/wiki/Model-Predictive-Path-Integral-Controller-(MPPI)

* Easy intro section writeup: https://sites.gatech.edu/acds/mppi/