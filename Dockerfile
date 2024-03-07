FROM osrf/ros:rolling-desktop-full

RUN mkdir -p ~/plansys2_ws/src && \
    cd ~/plansys2_ws/src && \
    git clone https://github.com/IntelligentRoboticsLabs/ros2_planning_system.git && \
    git clone https://github.com/IntelligentRoboticsLabs/plansys2_tfd_plan_solver.git && \
    git clone https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples.git && \
    #git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git && \
    git clone https://github.com/fmrico/popf.git &&\
    git clone https://github.com/ros-planning/navigation2.git && \
    #cd BehaviorTree.CPP && \
    #git switch v3.8 && \
    cd ~/plansys2_ws/src/popf && \
    git switch ros2 && \
    cd ~/plansys2_ws && \
    apt-get update && apt-get install -y lsb-release && \
    apt-get install -y ros-rolling-geographic-msgs && \
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro rolling && \
    cd ~/ 
    
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/rolling/setup.bash && \
    cd ~/plansys2_ws && \
    colcon build --symlink-install


# WORKDIR ~/plansys2_ws
