# Software components

This doc is to list all the "LEGO pieces" of this project. Components of same level should be loose coupling.

## System level introduction

1. Project language
    1. C++ with [google coding template](https://zh-google-styleguide.readthedocs.io/en/latest)
    1. C
    1. CMake
    1. bash
    1. Markdown
1. Building method
    1. CMake
        1. **[Added]** Test program
        1. **[Added]** Export lib/include
    1. ROS catkin (test with ros melodic)
1. Dependency library
    1. **[Added]** Glog
    1. **[Added]** Gflags
    1. Gtest
    1. **[Compatible]** Cartographer-ros
    1. ROS-Navigation-Costmap2d
    1. **[Added]** Json
    1. **[Added]** Protobuf
1. Visualization tools
    1. **[Added]** console
    1. **[Added]** ROS-Rviz
1. Simulation tools
    1. **[Added]** Map based simple simulation
    1. **[Added]** Gazebo

## Components

1. System level:
    1. **[Added]** System bash call interface.
    1. Process communication manager.
    1. **[Added]** File system lock manager(Use flock).
    1. System infomation interface(like CPU/RAM).
    1. Parameter manager:
        1. **[Added]** Use gflags.

1. Common level:
    1. Logging interface.
        1. **[Added]** Glog.
        1. Delayed logger.
    1. Math library:
        1. **[Added]** Radian & angle manager with translation.
        1. **[Added]** Coordinate transformation.
    1. **[Added]** Time relevant:
        1. **[Added]** Interface for acquiring different time.
        1. **[Added]** StopWatch.
        1. **[Added]** Timer.
    1. Local file manager interface:
        1. **[Added]** Inherit from protobuf.
        1. **[Added]** Json.
    1. Encrypter.
    1. Compresser.
    1. **[Added]** Mutex system.

1. Application level:
    1. **[Added]** Interface for describing a point & point list.
    1. **[Added]** Interface for describing a map cell & cell list.
    1. **[Added]** Interface for multi-layered map management.
        1. **[Added]** Map marker manager.

1. Algorithm library:
    1. Jitter filter algorithm.
    1. PID algorithm.
    1. **[Added]** Trace path algorithm.
    1. **[Added]** Dijkstra algorithm.
    1. **[Added]** ZigZag path planner algorithm.
    1. Particle filter algorithm.
    1. **[Added]** BAB match algorithm.

1. Hardware interface:
    1. **[Added]** Serial port interface.
    1. SDIO port interface.
    1. GPIO port interface.
    1. MIPI port interface.
    1. **[Added]** Chassis manager:
        1. **[Added]** Bumper
        1. **[Added]** Wheel
        1. **[Added]** Wall sensor
        1. **[Added]** Gyro
        1. **[Added]** Lidar

1. Logic framework:
    1. FSM
    1. **[Added]** HFSM
    1. Behavior tree
    1. Shadow devices
    1. **[Added]** Event mechanism

1. Motion library:
    1. Broad sense motion:
        1. Data manager control.
        1. Motor control.
    1. Narrow sense motion:
        1. **[Added]** Trace path.
        1. **[Added]** Rotate.
        1. **[Added]** Retreat.
        1. **[Added]** Encircle obstacle.

1. Movement library:
    1. **[Added]** Trace path movement.
    1. **[Added]** Encircle obstacle movement.

1. SLAM:
    1. 2D lidar SLAM:
        1. **[Added]** Cartographer
        1. Karto SLAM
        1. **[Added]** Simple SLAM (Pure frontend)
    1. VSLAM:
        1. ORB SLAM
