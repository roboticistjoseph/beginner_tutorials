# Development of ROS Services, Logging, and Launch files

As part of learning the newly introduced world of ROS 2, the tutorials given in the ros2_humble forum [here](http://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html), have been thoroughly practised. In this branch the development of ROS Services, Logging, and Launch files have been demonstrated.

## Overview
The exercise includes the following
1. Learning about simple Service and Client nodes fro [here](http://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html).
2. Integrating service with existing Publisher node.
3. Implementation of ROS 2 Logging levels which can be referred [here](https://docs.ros2.org/bouncy/api/rclcpp/logging_8hpp.html) and [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html#logger-levels).
4. Creating a launch file that launches both nodes and accepts at least one command-line argument, [reference1](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html) and [reference2](https://docs.ros.org/en/foxy/How-To-Guides/Launch-file-different-formats.html).
5. Adding doxygen-compatible comments for cpp files and methods within them.
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
git clone https://github.com/roboticistjoseph/beginner_tutorials/
cd beginner_tutorials
colcon build
```

## Steps to run the nodes
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
### Screenshot of rqt_console
![rqt_console](/results/rqt_console.png) 

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