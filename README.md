# Teleoperating Fetch robot's manipulation

### In advance
To operate Fetch robot with Moveit cpp code, I firstly created **Moveit!** package for remote PC. 


Tutorial for Fetch robot's Moveit setup assistant is available [here](https://www.youtube.com/watch?v=rfcXZcKZd8A&index=4&t=160s&list=LLVR1lsSDfFCKRlT5Rxl21sA).


### Teleoperation

End-effector moves according to the keyboard input given by user.


 
u, j/ i, k/ and o, l kbd input gives slight difference of x, y, z position w.r.t. tool frame.


e, d/ r, f/ and t, g kbd input gives slight difference of x, y, z rotation w.r.t. tool frame.


This code was successfully tested in ROS Indigo.

