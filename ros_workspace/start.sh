#!/bin/bash
# 创建新会话并命名窗口
tmux new-session -d -s robot_session -n "MultiPanes" \
    "cd /home/ubuntu/workspace/ZIMA/ros_workspace && source devel/setup.bash && roslaunch zima_gazebo gazebo.launch; exec bash"

# 水平分割窗口为上下两部分
tmux split-window -v -t robot_session:0.0 \
    "cd /home/ubuntu/workspace/ZIMA/ros_workspace && source devel/setup.bash && roslaunch zima_ros gazebo_demo.launch; exec bash"
    
# 将上半部分垂直分割为左右两窗格
tmux split-window -h -t robot_session:0.0 \
    "cd /home/ubuntu/workspace/ZIMA/ros_workspace && source devel/setup.bash && roslaunch zima_ros rviz.launch; exec bash"
    
# 将下半部分垂直分割为左右两窗格（保留一个备用窗格）
tmux split-window -h -t robot_session:0.1 \
    "cd /home/ubuntu/workspace/ZIMA/ros_workspace && source devel/setup.bash; exec bash"

# 调整布局为均匀排列（4窗格）
tmux select-layout -t robot_session:0 tiled

# 附加到会话
tmux attach -t robot_session