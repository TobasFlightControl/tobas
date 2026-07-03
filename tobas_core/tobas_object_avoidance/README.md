# Tobas Object Avoidance

This package receives an obstacle map and outputs repulsive acceleration for obstacle avoidance.

The repulsive potential field U<sub>r</sub>(q) centered at the obstacle position is defined as follows.
$$U_r(q) = \begin{cases} \frac{1}{2}k_r\left(\frac{1}{\rho(q, q_o)} - \frac{1}{\rho_0}\right)^2, & \rho(q, q_o) \leq \rho_0 \\ 0 & \rho(q, q_o) > \rho_0 \end{cases}$$

k<sub>r</sub>: coefficient
q: obstacle position
q<sub>0</sub>: drone position
ρ<sub>0</sub>: repulsive influence range
ρ: Euclidean distance between two points

The magnitude F<sub>r</sub> of the repulsive force acting on the drone is obtained by differentiating U<sub>r</sub>(q).
$$F_r = \begin{cases} k_r\left(\frac{1}{\rho(q, q_o)} - \frac{1}{\rho_0}\right)\frac{1}{\rho(q, q_o)^2}\nabla \rho(q, q_o), & \rho(q, q_o) \leq \rho_0 \\ 0 & \rho(q, q_o) > \rho_0 \end{cases}$$

This is based on the following paper.
https://www.researchgate.net/publication/392565826_Artificial_potential_field_method_for_path_and_trajectory_planning_of_unmanned_aerial_vehicles_A_review
