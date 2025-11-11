# ROS 2 Publisher & Subscriber - Programming Assignment 1

## Author

Venkata Madhav Tadavarthi (121058768)

## Overview
- `beginner_publisher`: publishes a custom string message (configured in `src/beginner_publisher.cpp`)
- `beginner_subscriber`: listens to the same topic and logs received strings
- Both nodes are implemented using the Google C++ Style Guide with Doxygen comments

## Project Structure

```
beginner_tutorials/
└── src/
    ├── beginner_publisher.cpp
    └── beginner_subscriber.cpp
```

## Assumptions & Dependencies
- ROS 2 Humble Hawksbill (desktop install)
- `colcon` build tool (included in ROS desktop variants)
- C++17-capable compiler (GCC 11+ on Ubuntu 22.04)
- Workspace sourced in a POSIX-compliant shell (`bash`, `zsh`, etc.)

## Setup (Build & Run)
1. Create (or reuse) a colcon workspace and clone this package:
   ```bash
   mkdir -p ~/tutorials_ws/src
   cd ~/tutorials_ws/src
   git clone https://github.com/Madhav2133/my_begineer_tutorails.git
   cd ~/tutorials_ws
   ```
2. Source ROS 2:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
3. Build the workspace:
   ```bash
   colcon build
   ```
4. Overlay the workspace:
   ```bash
   source install/setup.bash
   ```

## Running the Nodes

- Terminal 1:  
  ```
  ros2 run beginner_tutorials beginner_publisher
  ```

- Terminal 2 (after sourcing step 4): (Optional)  
  ```
  ros2 run beginner_tutorials beginner_subscriber
  ```

## Sample Outputs

```
$ ros2 run beginner_tutorials beginner_publisher 
[INFO] [1762837694.901837044] [beginner_publisher]: Publishing: 'This is message number: 0'
[INFO] [1762837695.401799679] [beginner_publisher]: Publishing: 'This is message number: 1'
[INFO] [1762837695.901715474] [beginner_publisher]: Publishing: 'This is message number: 2'
[INFO] [1762837696.401718960] [beginner_publisher]: Publishing: 'This is message number: 3'
[INFO] [1762837696.901814829] [beginner_publisher]: Publishing: 'This is message number: 4'
[INFO] [1762837697.401855771] [beginner_publisher]: Publishing: 'This is message number: 5'
[INFO] [1762837697.901801000] [beginner_publisher]: Publishing: 'This is message number: 6'
[INFO] [1762837698.401819595] [beginner_publisher]: Publishing: 'This is message number: 7'
[INFO] [1762837698.901846950] [beginner_publisher]: Publishing: 'This is message number: 8'
[INFO] [1762837699.401832718] [beginner_publisher]: Publishing: 'This is message number: 9'
[INFO] [1762837699.901919876] [beginner_publisher]: Publishing: 'This is message number: 10'
...
```

```
$ ros2 topic list
/beginner_publisher
/parameter_events
/rosout
```

```
$ ros2 run beginner_tutorials beginner_subscriber 
[INFO] [1762837818.276779292] [beginner_subscriber]: Heard: 'This is message number: 0'
[INFO] [1762837818.776493385] [beginner_subscriber]: Heard: 'This is message number: 1'
[INFO] [1762837819.276422004] [beginner_subscriber]: Heard: 'This is message number: 2'
[INFO] [1762837819.776603893] [beginner_subscriber]: Heard: 'This is message number: 3'
[INFO] [1762837820.276466864] [beginner_subscriber]: Heard: 'This is message number: 4'
[INFO] [1762837820.776541061] [beginner_subscriber]: Heard: 'This is message number: 5'
[INFO] [1762837821.276470790] [beginner_subscriber]: Heard: 'This is message number: 6'
[INFO] [1762837821.776494639] [beginner_subscriber]: Heard: 'This is message number: 7'
[INFO] [1762837822.276487475] [beginner_subscriber]: Heard: 'This is message number: 8'
[INFO] [1762837822.776459687] [beginner_subscriber]: Heard: 'This is message number: 9'
[INFO] [1762837823.276482660] [beginner_subscriber]: Heard: 'This is message number: 10'
...
```