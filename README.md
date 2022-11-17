# Development of Publisher and Subscriber using ROS 2
As part of learning the newly introduced world of ROS 2, the tutorials given in the ros2_humble forum [here](http://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html), have been thoroughly practised. In this branch the development of Publisher and Subscriber have been demonstrated.

## Overview
The exercise includes the following
1. Learning the differences in ROS and ROS2.
2. ROS 2 Humble installation and building from source which can be refered [here (installation)](http://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) and [here](http://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html).
3. Going through the tutorials of ROS 2 from [here (environment setup)](http://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html).
4. Creating a [package](http://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html).
5. Developing class files for ros2 nodes implementing publisher and subscriber which can be refered [here](http://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html).
6. Running 'cpplint' and 'cppcheck' to check for coding style and detect bugs.


## Dependencies
| Name | Version | License |
| :--- | :--- | :--- |
| Ubuntu | 20.04(LTS) | FSF Licenses |
| ROS 2 | Humble Hawksbill | Apache License 2.0 |
| C++ | 14 | Creative Commons Attribution-ShareAlike 3.0 Unported License |

## Tools
* ```ament_cmake```
* ```rclcpp```
* ```std_msgs```

## Steps to build this package
```
git clone https://github.com/roboticistjoseph/beginner_tutorials/tree/ros_pub_sub
cd beginner_tutorials
colcon build
```

## Steps to run the nodes
Subscriber Terminal:
```
. install/setup.bash
ros2 run beginner_tutorials subscriber_node 
```

Publisher Terminal:
```
. install/setup.bash
ros2 run beginner_tutorials publisher_node 
```

## Output
### Publisher Node terminal
```
joseph@jkatak:~/beginner_tutorials$ ros2 run beginner_tutorials subscriber_node 
[INFO] [1668644285.577618276] [subscriber]: Incoming messgae: 'Developer- Joseph:  0'
[INFO] [1668644286.077572455] [subscriber]: Incoming messgae: 'Developer- Joseph:  1'
[INFO] [1668644286.577284253] [subscriber]: Incoming messgae: 'Developer- Joseph:  2'
[INFO] [1668644287.077432683] [subscriber]: Incoming messgae: 'Developer- Joseph:  3'
[INFO] [1668644287.577370890] [subscriber]: Incoming messgae: 'Developer- Joseph:  4'
```

### Subscriber Node terminal
```
joseph@jkatak:~/beginner_tutorials$ ros2 run beginner_tutorials subscriber_node 
[INFO] [1668644285.577618276] [subscriber]: Incoming messgae: 'Developer- Joseph:  0'
[INFO] [1668644286.077572455] [subscriber]: Incoming messgae: 'Developer- Joseph:  1'
[INFO] [1668644286.577284253] [subscriber]: Incoming messgae: 'Developer- Joseph:  2'
[INFO] [1668644287.077432683] [subscriber]: Incoming messgae: 'Developer- Joseph:  3'
[INFO] [1668644287.577370890] [subscriber]: Incoming messgae: 'Developer- Joseph:  4'
```

## Code Analysis
Running 'cpplint' and 'cppcheck' to check for coding style and detect bugs.
### cpplint
Change to the root directory of the package, ```/beginner_tutorials```, and run:
```
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp ./include/*.hpp > ./results/cpplint.txt
```
The results of running ```cpplint``` can be found in ```/results/cpplint.txt```.

### cppcheck
Change to the root directory of the package, ```/beginner_tutorials```, and run:
```
cppcheck --enable=all --std=c++17 ./src/*.cpp ./include/*.hpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > results/cppcheck.txt
```
The results of running ```cppcheck``` can be found in ```/results/cppcheck.txt```.