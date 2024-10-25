<!-- omit from toc -->
# SCREWDRIVING BOT 1

- [Workspace Structure](#workspace-structure)
  - [ROS Packages](#ros-packages)
  - [Others](#others)
  - [Motion Plan Overview](#motion-plan-overview)
  - [System Flowchart](#system-flowchart)
- [Feature Overview](#feature-overview)
  - [Hole Detection](#hole-detection)
    - [To test on a video](#to-test-on-a-video)
    - [To test on an image](#to-test-on-an-image)
  - [To run tests](#to-run-tests)

### Recent Updates
- [Week 7 Wed][David] Vision and Brain framework completed; testing package added

### Push to main branch: CREATE PULL REQUEST!!!
- rebase first (so that the latest commits are on top) 
  - `git fetch origin            # Updates origin/master`\
    `git rebase origin/master    # Rebases current branch onto origin master`
- Make sure you squash the commits when merging
- ![squash](img/squash.png)
- Make sure the codebase is stable
- Add to "Recent Updates" if it's a feature update 

## Workspace Structure
### ROS Packages
- **movement**  
  (services and publisher model for communicating with the UR5e)  
  - Subtask 1
  - Subtask 2
- **vision**  
  (Centroid locating services and publisher model)  
  - Vision Server
- **end_effector**  
  (end_effector related publisher and control algo)
  - Subtask 1
- **brain**
- - brain
- **interfaces**  
  (Custom messages and services)
  - Src
	- BrainCmd (for testing individual packages)
	- VisionCmd
  - Msg
	- bla
- **transformations**  
  (Static Transformations publishers)
### Others
- end_effector visualisation in Rviz

### Motion Plan Overview
![overview pic](/img/MotionPlan_overview.png)

### System Flowchart
![flowchart](/img/flowchart.jpg)

## Feature Overview

### Hole Detection
The code is tuned for small circles

#### To test on a video
`python3 blob_detection.py`
#### To test on an image
`python3 blob_detection_im.py`

### To run tests
**Step 1**: In one terminal, run `ros2 launch brain system_launch.py`
**Step 2**: In another terminal, run `ros2 run testing brain_vision_test`, or other testing files