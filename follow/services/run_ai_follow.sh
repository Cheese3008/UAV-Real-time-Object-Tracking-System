source /opt/ros/jazzy/setup.bash
source ~/follow/install/setup.bash
ros2 run ai_follow test1_node --ros-args -p enable_gui:=false -p publish_debug_image:=true