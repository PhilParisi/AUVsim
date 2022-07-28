# AUVsim
The code presented here simulates an autonomous underwater vehicle (AUV) under various conditions and model fidelity. This project is an extension of a course taught at the University of Rhode Island (URI) by Dr. Mingxi Zhou. The code is primarily inspired by his work, but modifications have been made as adaptations for general usage. Additionally, an autonomous surface vessel (ASV) model will be developed in the future.

## 6 DOF Model
The AUV model has 6 degrees of freedom (DOF): x, y, and z (translation) as well as roll, pitch, and yaw (rotation). This differs from a autonomous surface vessel (ASV), which assumes the inability to roll or translate in z. Thus, AUV model is 6DOF and ASV model is 4DOF.

## Notation
There are two frames: earth and body. The earth frame considers where the AUV exists in 3D space and how the vehicle moves with respect to a fixed origin in space. The body frame is a 'first person' perspective ignorant of the earth frame. The body frame moves with the AUV and is typically placed at the AUV's center of gravity. 

### Earth Frame
The earth frame has xyz position and dot_xyz change in position (velocity relative to fixed earth frame origin). Similarly, there exists roll pitch yaw (rpy angles) and dot_rpy (angular velocity). Latitude and Longitude are also earth frame coordinates.

### Body Frame
The body frame does not know where it is in space (xyz) nor it's current rotation angles (rpy); hoewver, it does know its velocities (both in translation and rotation). Body frame translational velocities are surge (+x) called u, sway (+y) called v, and heave (+z) called w.  Hence, uvw. Body frame rotational velocities are roll rate (about x axis) called p, pitch rate (about y axis) called q, and yaw rate (about z axis) called r. Hence, pqr.


# Running on Your System
If this is your first time using git and/or github, please seek assistance through online videos. Once you understand git, go ahead and clone the repository. First, open a terminal. Second, navigate to the folder you want to setup the repository in. Third, clone with:
```
git clone https://github.com/PhilParisi/AUVsim.git
```
you can also simply download the files raw, GitHub has an option to download a .zip file on the main page of the repository.

## Types of Simulations
There are a handful of different simulations you can run. It ranges from the most basic kinematics to advanced path-planning and terran-aided navigation. Select the proper folder of interest and work from there! Note some simulations require considering computating speed, and it is recommended to only run a few time iterations (go to the main for loop and decrease the number of loops to do).

### Kinematics
Kinematics is the study of motion of points and bodies without consideration of the forces acting upon them. This model ignores gravity, buoyancy, and other forces but ensure proper kinematic equations. Check out this [guide to kinematics](https://www.physicsclassroom.com/class/1DKin/Lesson-6/Kinematic-Equations). Think of x2 = x1 + velocity * time, or velocity2 = velocity1 + acceleration * time

### Dynamics
You can think of dynamics as kinematics + forces. It is the study of how moving objects behave and the causes of movement. This is where we add in buoyancy.

### Future Work
This will be updated as more simulations become available.
