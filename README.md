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

### Recent Updates

### Push to main branch
- Make sure you squash the commits
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
  - Subtask 1
  - Subtask 2
- **end_effector**  
  (End-effector related publisher and control algo)
  - Subtask 1
- **brain**
- **interfaces**  
  (Custom messages and services)
  - Src
	- bla
  - Msg
	- bla
- **transformations**  
  (Static Transformations publishers)
### Others
- End-effector visualisation in Rviz

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