# AUVsim

The code presented here simulates an autonomous underwater vehicle (AUV) under various conditions and model fidelity. This project is an extension of a course taught at the University of Rhode Island (URI) by Dr. Mingxi Zhou. The code is primarily inspired by his work, but modifications have been made as adaptations for general usage. 

## 6 DOF Model
The AUV model has 6 degrees of freedom (DOF): x, y, and z (translation) as well as roll, pitch, and yaw. This differs from a autonomous surface vessel (ASV) 

## Notation
There are two frames: earth and body. The earth frame considers where the AUV exists in 3D space. The earth frame origin is fixed in space. The body frame is a 'first person' perspective ignorant of the earth frame. The body frame moves with the AUV and is typically placed at the AUV's center of gravity. 

### Earth Frame
The earth frame has xyz position and dot_xyz change in position (velocity relative to fixed earth frame origin). Similarly, there exists roll pitch yaw (rpy angles) and dot_rpy (angular velocity). Latitude and Longitude are also earth frame coordinates.

%%% Body Frame
The body frame does not know where it is in space (xyz) nor it's current angles of orientation (rpy); hoewver, it does know its velocities (translation and rotational). Body frame translational velocities are surge (+x) called u, sway (+y) called v, and heave (+z) called w.  Hence, uvw. 
