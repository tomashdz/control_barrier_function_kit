# Risk-based-Stochastic-Control-Barrier-Functions

The following functions solve a reach avoid problem by bounding the risk to a desired value in a high way scenario and in presence of other agents with stochastic velocities:

PM_stochastic: Considers a point mass model fro the ego vehicle. Reachability is enforces by Lyapunov functions.

Uni_stochastic_Uni_PF: Considers a unicycle model for the vehicle. Reachability is enforces with the help of potential fields.

traffic: Scenario with more car handled by potential fields

traffic_lyap: Scenario with more car handled by Lyapunov function

traffic_lyap_minvchange**: This is the code used in the paper "Risk-bounded Control using Stochastic Barrier Functions". It considers denser traffic. Uses Bicycle mode and Lyapunov functions for reachability. Furthermore, the quadratic cost includes a terms that aims to reduce fast changes in the control inputs from iteration to iteration.

stochastic risk CBFs (Python) includes the python version of the code in "traffic_lyap_minvchange".
