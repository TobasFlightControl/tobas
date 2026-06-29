# 概要

障害物を回避するため、障害物マップを受信し斥力加速度を出力します

障害物位置を中心とした斥力場U<sub>r</sub>(q)を以下のように定義します。
$$U_r(q) = \begin{cases} \frac{1}{2}k_r\left(\frac{1}{\rho(q, q_o)} - \frac{1}{\rho_0}\right)^2, & \rho(q, q_o) \leq \rho_0 \\ 0 & \rho(q, q_o) > \rho_0 \end{cases}$$

k<sub>r</sub>：係数
q：障害物位置
q<sub>0</sub>：ドローン位置
ρ<sub>0</sub>：斥力影響範囲
ρ：2点間のユークリッド距離

ドローンにかかる斥力の大きさF<sub>r</sub>は、U<sub>r</sub>(q)を微分した値になります。
$$F_r = \begin{cases} k_r\left(\frac{1}{\rho(q, q_o)} - \frac{1}{\rho_0}\right)\frac{1}{\rho(q, q_o)^2}\nabla \rho(q, q_o), & \rho(q, q_o) \leq \rho_0 \\ 0 & \rho(q, q_o) > \rho_0 \end{cases}$$

以下の論文を参考にしています。
https://www.researchgate.net/publication/392565826_Artificial_potential_field_method_for_path_and_trajectory_planning_of_unmanned_aerial_vehicles_A_review
