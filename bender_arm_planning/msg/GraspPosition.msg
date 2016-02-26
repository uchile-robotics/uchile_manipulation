# Posicion efectiva del grasp
geometry_msgs/PoseStamped grasp_pose

# Multiples posiciones factibles
trajectory_msgs/JointTrajectoryPoint[] pregrasp
trajectory_msgs/JointTrajectoryPoint[] grasp

# Calidad del grasp
float64 quality