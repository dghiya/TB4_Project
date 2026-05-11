## For gemini perception
#Terminal 1
ros2 launch tb4_openx_sim gazebo_sim.launch.py

# Terminal 2 — Navigation (optional, for full pipeline)
ros2 launch tb4_openx_navigation navigate.launch.py

# Terminal 3 — Gemini perception
export GOOGLE_API_KEY="AIzaSyDHSN-k9BCDRpsHLpfsB3JmvY7HTxmYg7k"
ros2 launch ros2_gemini_er gemini_debug.launch.py \ publish_rate_limit_hz:=0.5

#Make gemini detect custom objects
ros2 launch ros2_gemini_er gemini_debug.launch.py \
  target_label:="object name here" \
  publish_rate_limit_hz:=0.5

# Get an API key from: https://aistudio.google.com/api-keys?project=gen-lang-client-0095763437


## For Navigation and Manipulator
1. ros2 launch tb4_openx_sim gazebo_sim.launch.py
2. ros2 launch tb4_openx_navigation navigate.launch.py
3. ros2 launch tb4_openx_manipulation move_group.launch.py
4. ros2 launch tb4_openx_manipulation manipulation_pipeline.launch.py
5. ros2 run tb4_openx_navigation mission_controller.py

# For real robot
## Terminal 1 — bring up robot (turtlebot4 should auto-launch on boot)
## If not:
1. ros2 launch turtlebot4_bringup standard.launch.py
2. ros2 launch tb4_openx_navigation real_navigate.launch.py
3. ros2 launch tb4_openx_manipulation real_manipulation_pipeline.launch.py
4. ros2 launch tb4_openx_manipulation move_group.launch.py use_sim:=false
5. ros2 run tb4_openx_navigation real_mission_controller.py
