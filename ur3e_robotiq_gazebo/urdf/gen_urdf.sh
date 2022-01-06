rosrun xacro xacro $(rospack find ur3e_robotiq_gazebo)/urdf/ur3e_robotiq.xacro \
joint_limit_params:=$(rospack find ur_description)/config/ur3e/joint_limits.yaml \
kinematics_params:=$(rospack find ur_description)/config/ur3e/default_kinematics.yaml \
physical_params:=$(rospack find ur_description)/config/ur3e/physical_parameters.yaml \
visual_params:=$(rospack find ur_description)/config/ur3e/visual_parameters.yaml \
transmission_hw_interface:=hardware_interface/PositionJointInterface \
safety_limits:=false \
safety_pos_margin:=0.15 \
safety_k_position:=20 \
-o $(rospack find ur3e_robotiq_gazebo)/urdf/ur3e_robotiq.urdf