# Development of ROS 2 tf2, unit testing, bag files

In this branch the development of ROS 2 tf2, unit testing, bag files have been demonstrated as part of learning the newly introduced world of ROS 2.

## Overview
The exercise includes the following
1. Learning about static and time-variant broadcaster from [here](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html).
2. Verification of TF frames using ```tf2_echo``` and having a visual representation by generating a pdf output.
3. Learning to create different levels of integrations tests and test cases using following links: [Testing](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html), [System test](https://github.com/TommyChangUMD/system_tests/tree/humble/test_rclcpp_simple), [Integration test](https://github.com/TommyChangUMD/minimal_integration_test) and [Unit Tests](https://www.youtube.com/watch?v=t2Jm1Nt49-A&list=PLK0b4e05LnzbuxWCdip-2Tf-SIiZle5NA&index=5).
4. Using 'ros bags' to record topic info and save it in database from [here](http://docs.ros.org/en/humble/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-CPP.html).
5. Creating a yaml based launch file that launches ROS 2 bag and record all topics by using command-line argument, [reference1](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html).
6. Adding doxygen-compatible comments for cpp files and methods within them.
7. Running 'cpplint' and 'cppcheck' to check for coding style and detect bugs.


## Dependencies
| Name | Version | License |
| :--- | :--- | :--- |
| Ubuntu | 20.04(LTS) | FSF Licenses |
| ROS 2 | Humble Hawksbill | Apache License 2.0 |
| C++ | 14 | Creative Commons Attribution-ShareAlike 3.0 Unported License |

## REPs
* ```ament_cmake```
* ```rclcpp```
* ```std_msgs```
* ```ros2launch```


## Setting up environment: part 1
This is the 'overlay' workspace present at the Top.
```
. <path-to-ros2_humble>/ros2_humble/install/local_setup.bash
```

## Commands to build this package
```
git clone https://github.com/roboticistjoseph/beginner_tutorials -b Week11_HW
rosdep install -i --from-path src --rosdistro humble -y
cd beginner_tutorials
colcon build --packages-select beginner_tutorials

```

## Setting up environment: part 2
- This is the 'underlay' workspace present beneath 'overlay'.
- Both the 'setting up of the environment' procedures must be done for every new terminal opened.
```
. install/setup.bash
```

## Commands to run the nodes
Publisher-Server Terminal:
```
. install/setup.bash
ros2 run beginner_tutorials publisher_node 
```

Client Terminal:
```
. install/setup.bash
ros2 service call /add_two_ints_v2 example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
```

## Output
### Publisher-Server Node terminal
```
joseph@jkatak:~/beginner_tutorials$ ros2 run beginner_tutorials publisher_node 
[INFO] [1668654272.050861640] [publisher]: Publishing: 'Developer- Joseph:  0'
[INFO] [1668654272.550778845] [publisher]: Publishing: 'Developer- Joseph:  1'
[INFO] [1668654273.050546312] [publisher]: Publishing: 'Developer- Joseph:  2'
[INFO] [1668654273.550717134] [publisher]: Publishing: 'Developer- Joseph:  3'
[INFO] [1668654274.050721202] [publisher]: Publishing: 'Developer- Joseph:  4'
[INFO] [1668654274.550975447] [publisher]: Publishing: 'Developer- Joseph:  5'
[INFO] [1668654275.050572626] [publisher]: Publishing: 'Developer- Joseph:  6'
[INFO] [1668654275.550823319] [publisher]: Publishing: 'Developer- Joseph:  7'
[INFO] [1668654276.050522875] [publisher]: Publishing: 'Developer- Joseph:  8'
[INFO] [1668654276.550567372] [publisher]: Publishing: 'Developer- Joseph:  9'
[INFO] [1668654277.050506899] [publisher]: Publishing: 'Developer- Joseph:  10'
[INFO] [1668654277.061940966] [publisher]: Incoming request
a: 1 b: 2
[INFO] [1668654277.062012241] [publisher]: sending back response: [3]
[INFO] [1668654277.550480005] [publisher]: Publishing: 'Developer- Joseph:  11'
[INFO] [1668654278.050686072] [publisher]: Publishing: 'Developer- Joseph:  12'

```

### Client Node terminal
```
joseph@jkatak:~/beginner_tutorials$ ros2 service call /add_two_ints_v2 example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=1, b=2)

response:
example_interfaces.srv.AddTwoInts_Response(sum=3)

```

## Publishing static transform using ROS2 TF2
- Converting the static transforma between world frame and child frame.

- For output
```
ros2 topic echo /tf_static
```
- Save frames
```
ros2 run tf2_tools view_frames
```

### Screenshot of rqt_console
```rqt_console``` helps monitor logged messages.
```
ros2 run rqt_console rqt_console
```
![rqt_console](/results/rqt_console.png) 

## Commands to run Launch file
```
cd beginner_tutorials && . install/setup.bash
ros2 launch nodes_launch.yaml
```

## Store published data using ros bag using ROS2
- Launch this file to start recording the data.
```
ros2 launch launch/bag_launch.yaml
```
- To stop, press ctrl+c. 
- To see the data stored:
```
ros2 bag play tutorial_bag
```


## Commands to Moniter Parameter response
```
ros2 param list
ros2 param set /param_helper_node my_parameter universe
```

## Monitering Parameter response output
```
joseph@jkatak:~/beginner_tutorials/launch$ ros2 launch node_launcher.yaml
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [minimal_param_node-1]: process started with pid [44475]
[minimal_param_node-1] [INFO] [1668651106.180145554] [minimal_param_node]: Hello world!
[minimal_param_node-1] [INFO] [1668651107.180334288] [minimal_param_node]: Hello world!
[minimal_param_node-1] [INFO] [1668651108.180327606] [minimal_param_node]: Hello universe!
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

rosbag 2 Tutorial: [here](http://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)

Description: ros2 bag is a command line tool for recording data published on topics in your system. It accumulates the data passed on any number of topics and saves it in a database. You can then replay the data to reproduce the results of your tests and experiments. Recording topics is also a great way to share your work and allow others to recreate it.

Installation of ```rosbag2```:
```
sudo apt install ros-humble-rosbag2
```

Quaterneon: tf2::Quaternion is a class for a quaternion that provides convenient functions for converting Euler angles to quaternions and vice versa.
