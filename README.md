# Simple Line Follower

## About

This repository demonstrates a basic line-following PID-controlled robot. 

This ROS 2 package is used to demonstrate these examples :

- Code which is tough to test (Ex : [naive_line_follower.cpp](src/naive_line_follower.cpp))
- Code which is easy to test (Ex : [line_follower.cpp](src/line_follower.cpp))
- Basic unit-tests taking the line follower as an example(Ex : [test_line_follower_good.cpp](test/test_line_follower_good.cpp) and [test_line_follower_poor.cpp](test/test_line_follower_poor.cpp))
- Good unit tests taking the line follower as an example(Ex : [test_line_follower.cpp](test/test_line_follower.cpp))


## Dependencies

- Ubuntu 22.04
- ROS 2 Humble
- Ignition Gazebo(Fortress)

## Installation

1. Clone this repository

    ```bash
    git clone git@github.com:ggupta9777/simple_line_follower.git
    ```

2. Install dependencies using `rosdep`:

    ```bash
    rosdep install --from-paths src -y --ignore-src
    ```

3. Build the packages with test config(**Note** : The `--cmake-args`` used above are to enable code coverage analysis)

    ```bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug   -DCMAKE_CXX_FLAGS="--coverage" -DCMAKE_CXX_OUTPUT_EXTENSION_REPLACE=ON
    ```

## Run instructions

1. Launch [bcr_bot](https://github.com/blackcoffeerobotics/bcr_bot/tree/ros2)

    ```bash
    ros2 launch bcr_bot gz.launch.py
    ```

2. Run the [tf_broadcaster](src/tf_broadcaster.cpp) node

    ```bash
    ros2 run simple_line_follower tf_broadcaster
    ```

3. Run the line follower node

    - Run the [line_follower_node](src/line_follower.cpp)

        ```bash
        ros2 run simple_line_follower line_follower_node --ros-args --params-file src/simple_line_follower/config/waypoint.yaml
        ```
    - Run the [naive_line_follower_node](src/naive_line_follower.cpp)

        ```bash
        ros2 run simple_line_follower naive_line_follower_node --ros-args --params-file src/simple_line_follower/config/waypoint.yaml
        ```

## Running Tests

- Run the test scripts(**Note** : The `--event-handler=console_direct+` argument is used to show the logs of the testing process)

    ```bash
      colcon test --event-handler=console_direct+
    ```
### Generate Code Coverage

1. Use [lcov](https://github.com/linux-test-project/lcov) to generate code coverage

    ```bash
    lcov --capture --directory build/ --output-file code_coverage_report.info
    ```    

2. Generate an HTML file to showcase the code coverage report

    ```bash
    genhtml code_coverage_report.info --output-directory html_code_coverage_report
    ```

3. Open the code coverage on your browser(this will only work nativelly)
    
    ```bash
    sensible-browser html_code_coverage_report/index.html
    ```

## Topics

- `/bcr_bot/cmd_vel` : This topic is used to publish velocity messages to the bot in Gazebo

## Demo

This is a demo of the bot following a line defined by the waypoints : `(0, 0, 0)` and `(5, 5, 0)`

<img src="res/line_follower.gif" height=500>