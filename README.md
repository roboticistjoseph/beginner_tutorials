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

## Commands to build this package
```
git clone https://github.com/roboticistjoseph/beginner_tutorials/
cd beginner_tutorials
colcon build
```

## Commands to run the nodes
Publisher-Server Terminal:
```
cd beginner_tutorials && . install/setup.bash
ros2 run beginner_tutorials publisher_node 
```

Client Terminal:
```
cd beginner_tutorials && . install/setup.bash
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

### Logging Levels

<pre><font color="#4E9A06"><b>joseph@jkatak</b></font>:<font color="#3465A4"><b>~/beginner_tutorials</b></font>$ ros2 run beginner_tutorials publisher_node 
[INFO] [1668725753.965350804] [publisher]: Publishing: &apos;Developer- Joseph:  0&apos;
[INFO] [1668725753.965833352] [publisher]: Logger level: Info, Publishing:Developer- Joseph:  0
[INFO] [1668725754.465347022] [publisher]: Publishing: &apos;Developer- Joseph:  1&apos;
[INFO] [1668725754.465566642] [publisher]: Logger level: Info, Publishing:Developer- Joseph:  1
[INFO] [1668725754.965364998] [publisher]: Publishing: &apos;Developer- Joseph:  2&apos;
[INFO] [1668725754.965579624] [publisher]: Logger level: Info, Publishing:Developer- Joseph:  2
[INFO] [1668725755.465385939] [publisher]: Publishing: &apos;Developer- Joseph:  3&apos;
[INFO] [1668725755.465604558] [publisher]: Logger level: Info, Publishing:Developer- Joseph:  3
<font color="#C4A000">[WARN] [1668725755.465669556] [publisher]: Logger level: Warning, Too many Publising cycles</font>
[INFO] [1668725755.965402930] [publisher]: Publishing: &apos;Developer- Joseph:  4&apos;
[INFO] [1668725755.965618115] [publisher]: Logger level: Info, Publishing:Developer- Joseph:  4
<font color="#C4A000">[WARN] [1668725755.965681029] [publisher]: Logger level: Warning, Too many Publising cycles</font>
[INFO] [1668725756.465410277] [publisher]: Publishing: &apos;Developer- Joseph:  5&apos;
[INFO] [1668725756.465678144] [publisher]: Logger level: Info, Publishing:Developer- Joseph:  5
<font color="#C4A000">[WARN] [1668725756.465744670] [publisher]: Logger level: Warning, Too many Publising cycles</font>
[INFO] [1668725756.965438963] [publisher]: Publishing: &apos;Developer- Joseph:  6&apos;
[INFO] [1668725756.965654878] [publisher]: Logger level: Info, Publishing:Developer- Joseph:  6
<font color="#C4A000">[WARN] [1668725756.965716958] [publisher]: Logger level: Warning, Too many Publising cycles</font>
[INFO] [1668725757.465370031] [publisher]: Publishing: &apos;Developer- Joseph:  7&apos;
[INFO] [1668725757.465589177] [publisher]: Logger level: Info, Publishing:Developer- Joseph:  7
<font color="#C4A000">[WARN] [1668725757.465652714] [publisher]: Logger level: Warning, Too many Publising cycles</font>
<font color="#CC0000">[FATAL] [1668725757.465699010] [publisher]: Logger level: Fatal, Fatal error due to overuse</font>
[INFO] [1668725757.965204002] [publisher]: Publishing: &apos;Developer- Joseph:  8&apos;
[INFO] [1668725757.965326375] [publisher]: Logger level: Info, Publishing:Developer- Joseph:  8
<font color="#C4A000">[WARN] [1668725757.965352503] [publisher]: Logger level: Warning, Too many Publising cycles</font>
<font color="#CC0000">[FATAL] [1668725757.965369198] [publisher]: Logger level: Fatal, Fatal error due to overuse</font>
</pre>


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