# MTC Cleaning Task

This folder contains a complete MoveIt Task Constructor (MTC)-based motion planning pipeline to control a robot arm for a **mirror cleaning task** using a brush. It extends nozzle spray logic and lives inside the nolon_robot_pkg package.

## Features

- Pick up a brush tool using top-down grasp
- Approach the mirror with a safe offset
- Perform horizontal cleaning swipes, row by row (top to bottom)
- Return the brush to its original position
- Use of MTC stages: MoveTo, MoveRelative, Connect, ModifyPlanningScene, ComputeIK, etc.

## Folder Structure

```
mtc_cleaning_task/
└── nolon_robot_pkg/
    ├── src/
    │   └── mtc_spray_clean.cpp        # Main task code (spray + cleaning)
    ├── launch/
    │   └── mtc_spray_clean.launch.py  # Launch file to start task
        
    ├── CMakeLists.txt
    ├── package.xml
    └── ignore/                  # Ignored from Git (via .gitignore)
```

## How to Run

1. **Build the package**
   ```bash
   cd ~/ws_ros/ws_moveit
   colcon build --packages-select nolon_robot_pkg
   source install/setup.bash
   ```

2. **Launch the task**
   ```bash
   ros2 launch nolon_robot_pkg mtc_spray_clean.launch.py
   ```
     ```bash
   ros2 launch nolon_robot_pkg simple_panda.launch.py
   ``` 

## Task Breakdown

1. Pick up brush from table
2. Move to mirror start position
3. Clean mirror:
   - Horizontal left-to-right swipes
   - Step down vertically after each row
4. Return brush to original location
5. Return home

## Mirror Cleaning Logic

- Mirror size & spacing are defined by constants (e.g. `MIRROR_X`, `SWIPE_DISTANCE`, etc.)
- Uses `MoveTo` to position for each row, then `MoveRelative` for horizontal swipes
- Alternates direction each row for efficiency
- Visual title messages shown via `drawTitle()` (RViz marker)

##  Notes

- Be sure to set RViz **Fixed Frame** to `"world"`
- Visual markers are published on `/rviz_visual_tools`
- Robot model: **Franka Emika Panda**

## 📄 License

MIT License

##  Author

Maintained by @vijethrai
