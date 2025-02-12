<!-- omit from toc -->
# SCREWDRIVING BOT 1

- [What is the project about?](#what-is-the-project-about)
  - [We won the 2024 UNSW MTRN4231 Robotics Project Competition!](#we-won-the-2024-unsw-mtrn4231-robotics-project-competition)
- [Demo](#demo)
  - [Bugs to fix](#bugs-to-fix)
  - [Recent Updates](#recent-updates)
  - [Push to main branch: CREATE PULL REQUEST!!!](#push-to-main-branch-create-pull-request)
- [Workspace Structure](#workspace-structure)
  - [ROS Packages](#ros-packages)
  - [Others](#others)
  - [Motion Plan Overview](#motion-plan-overview)
  - [System Flowchart](#system-flowchart)
- [Feature Overview](#feature-overview)
  - [Hole Detection](#hole-detection)
    - [To test on a video](#to-test-on-a-video)
    - [To test on an image](#to-test-on-an-image)
  - [To run tests with end-effector connected](#to-run-tests-with-end-effector-connected)
  - [To run tests without end-effector connected](#to-run-tests-without-end-effector-connected)
  - [End-effector visualisation](#end-effector-visualisation)
  - [Custom End-effector](#custom-end-effector)
  - [Housing block](#housing-block)
  - [Screw block](#screw-block)

[nick]: https://www.linkedin.com/in/nickojbell/
[carmen]: https://www.linkedin.com/in/jiawen-zhang-1aa622203/
[daryl]: https://www.linkedin.com/in/daryl-lee-7b022a201/
[david]: https://www.linkedin.com/in/davidnie0418/


## What is the project about?

The project utilizes UR5e from Universal Robots for automated screw driving.

The novelty of this system lies in the integration of a depth camera mounted on the end effector, enabling the robot to **adapt to different screw-hole locations without manual tuning**.

**Project Duration**: 6 weeks

**Authors**:
* Nicholas (James) Bell: [Nick's LinkedIn Profile][nick]
* Jiawen Zhang: [Jiawen's LinkedIn profile][carmen]
* David Nie: [David's LinkedIn profile][david]

**Supervisor**:
* Daryl Lee: [Daryl's LinkedIn profile][daryl]

### We won the 2024 UNSW MTRN4231 Robotics Project Competition!

![we_won](<img/we_won.png>)

## Demo
Video link: 
https://www.youtube.com/shorts/ufF1myLWomA

<div align="center">
  <img src="img/demo_img.png" alt="cad" width="100%">
</div>

### Bugs to fix
- ~~brain_routine_test sometimes send two commands to brain~~  
  - ~~need to add `is_busy` support in brain~~
### Recent Updates
- [Week 10 Mon][Nicholas, Jiawen] Fine tuned brain routine to align with hole displacement and optimised screwdriving control routine.
- [Week 9 Thu][Nicholas, Jiawen] Refined the movement package for precise cartesian control commands.
- [Week 9 Wed][Jiawen] Attempted the jogging command to improve precision of arm movement.
- [Week 9 Tue][David] Move to screw fully working, change home position, change in arm-movement pkg, ignore large screws.
- [Week 9 Mon][David] Add `OOI` frame that indicates the screwhole, add transformation to real-coordinate support.
- [Week 8 Thu][Jiawen] Attempted Cartesian movement for the movement package but MoveIt unable to finish planning path.
- [Week 8 Wed][David] Add support for converting to RealCoor with respect to base_link, bring back transformation pkg.
- [Week 8 Wed][David] Fix centroid locating algorithm.
- [Week 8 Tue][Nicholas, Jiawen] Attempted orientation constraint and established movement package to control UR5e robot arm based on joint constraints.
- [Week 8 Tue][David] Enable collosion check, add toolpoint_link and camera_link, deprecated transformation pkg.
- [Week 8 Mon][Jiawen] Created the base code for the movement package, including an arm brain and arm movement.
- [Week 7 Sun][David] Add dy_trans between camera_socket and camera, tune end_effector scale.
- [Week 7 Sun][David] End_effector_description package complete, system_launch now launches with UR5e and camera and end_effector visualisation.
- [Week 7 Sat][David] Add stub for Screwdriving Routine.
- [Week 7 Wed][David] Vision and Brain framework completed; testing package added.
- [Week 7 Wed][Nicholas] Established arduino serial communication inside of ROS and mapped basic motor control functions inside End Effector package.
- [Week 7 Tue][Nicholas] End Effector design complete and 3D printed.

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
  - arm_brain
  - arm_movement
  - `README.md` (for movement)
- **vision**  
  (Centroid locating services and publisher model)  
  - Vision Server
- **end_effector**  
  (end_effector related publisher and control algo)
  - end_effector (Server offering services such as screwdriving, light on/off, status report)
  - arduino_serial (For bridging Arduino and ROS2)
- **end_effector_description**
  (contains end_effector visualisation, camera launch, and UR5e launch)
- **brain**
- - brain
- **interfaces**  
  (Custom messages and services)
  - Src
	- BrainCmd (for testing individual packages)
	- VisionCmd (interface with Vision Module)
	- EndEffectorCmd 
	- ArmCmd (interface with Moveit and UR5e)
	- BrainRoutineCmd (run the closed-loop operation)
	- PublishOoiCmd (publish a frame `OOI` that indicates the target screwhole)
	- RealCoorCmd (converting to coordinate with respect to `base_link`)
  - Msg
	- N/A
- **transformations**
  (Static Transformations publishers)
  ~~- camera_dy_trans (dy_broadcaster bewteen camera_socket and camera)~~
  - ooi_server (publish `OOI`, convert to RealCoor with respect to base_link)
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

### To run tests with end-effector connected
**Step 1**: In one terminal, run `ros2 launch brain system_launch.py`
**Step 2**: In another terminal, run `ros2 run testing brain_vision_test`, or other testing files

### To run tests without end-effector connected
**Step 1**: In one terminal, run `ros2 launch brain without_endeffector_launch.py`
**Step 2**: In another terminal, run `ros2 run testing brain_vision_test`, or other testing files

### End-effector visualisation
1. `use_fake` indicates whether Real or Fake UR5e is used
2. There are launch files that launches only the end-effector or the end-effector with UR5e with no driver support
   1. `ros2 launch end_effector_description end_effector_only.launch.py`
   2. `ros2 launch end_effector_description end_effector_withModel.launch.py`
![visualisation](img/visualisation_1.png)

### Custom End-effector

<div align="center">
  <img src="img/end_effector_assembly.png" alt="cad" width="50%">
</div>

![drawing](img/end_effector_engineering_drawing.png)

### Housing block

<div align="center">
  <img src="img/housing.png" alt="cad" width="50%">
</div>

<div align="center">
  <img src="img/housing2.png" alt="cad" width="50%">
</div>

### Screw block

<div align="center">
  <img src="img/screwblock.png" alt="cad" width="50%">
</div>

<div align="center">
  <img src="img/screwblock2.png" alt="cad" width="50%">
</div>
