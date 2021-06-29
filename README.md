<!-- Press Ctrl + Shift + m to view markdown language preview
Download from the following atom package
[REF] https://atom.io/packages/markdown-preview -->
# Baxter Experiments
Baxter robot in MuJoCo

- [MuJoCo forum thread which provided the `.xml` Model file](http://mujoco.org/forum/index.php?threads/commanding-to-a-joint-angle-configuration.3520/)

# Camera Calibration
The camera should be calibrated before the usage. Most of the source codes are saved in [openCV](./openCV) directory.

- [Whole Tutorial 1 of the calibration process](https://automaticaddison.com/how-to-perform-camera-calibration-using-opencv/ "tutorial 1")
- [Whole Tutorial 2 of the calibration process](https://www.geeksforgeeks.org/camera-calibration-with-python-opencv/ "tutorial 2")
- [Generate multiple checkboard images for the calibration](https://docs.opencv.org/master/da/d0d/tutorial_camera_calibration_pattern.html "script gen")

# Literatures
The list of literatures related to flexible object manipulation with actual hardware is as follows:
- [Motion planning for dynamic folding of a cloth with two high-speed robot hands and two high-speed sliders](https://ieeexplore.ieee.org/document/5979606)
  - Used a simple algebraic equation to approximate the complex cloth model. Time delay was considered for the model, i.e., time delay from the motion near the grasping point to the distant cloth part.
  - No consideration of dynamics, only static inverse kinematics was used to solve this control problem. Moreover only kinematics of the robot was considered.
  - While the success rate of cloth deformation was 100%, for grasping it was 30%.
