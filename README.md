# Unity Robot Simulation
https://mavsuta-my.sharepoint.com/:v:/r/personal/jxg2371_mavs_uta_edu/Documents/Recordings/Path%20Planning-20241108_225714-Meeting%20Recording.mp4?csf=1&web=1&e=2ovrg0&nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJTdHJlYW1XZWJBcHAiLCJyZWZlcnJhbFZpZXciOiJTaGFyZURpYWxvZy1MaW5rIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXcifSwicGxheWJhY2tPcHRpb25zIjp7InN0YXJ0VGltZUluU2Vjb25kcyI6NTAuMzl9fQ%3D%3D

https://mavsuta-my.sharepoint.com/:f:/g/personal/jxg2371_mavs_uta_edu/Ejq84vZmNj9Lj-RewqHO6xgBv5YzBtQRYDduj_RywQERaA?e=vjyoYT

## Overview
Sprint 5: Advanced Robot Planning

## Features
Task 1: Use computer vision to map obstacles in the scene
Task 2: Use any of the feasible motion planning algorithms to compute valid collision-free trajectory from arbitrary robot starting point to perform a task e.g., picking up a cube hidden behind a wall

## Implementation
- FABRIK: This is a simple and efficient IK algorithm that works well for the Niryo One's 6-DOF configuration. 
- RRT with IK : This approach can be used to generate collision-free paths for the Niryo One, especially in complex environments. 
- Collision and Object Detection using Transform from opencv_receiver.py
- Implemented a Green Box based on where the image is located.

![image](https://github.com/user-attachments/assets/8545c69d-d317-42ad-8dc9-dd05a3960c93)

