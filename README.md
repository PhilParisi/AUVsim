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
you can also simply download the files raw, GitHub has an option to download a zip on the main page of the repository.
