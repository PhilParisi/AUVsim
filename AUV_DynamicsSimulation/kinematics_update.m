function [AUV] = kinematics_update(AUV,dt)

% Update the kinematics of the AUV!

    % 1 LINEAR VELOCITY from BODY to EARTH
    R_b2e = rotz( rad2deg(AUV.rpy(3))) * roty(rad2deg(AUV.rpy(2))) * rotx(rad2deg(AUV.rpy(1)));
    % new earth velocity (xyz) = rotate body to earth * body velocity (uvw)
    AUV.dot_xyz = R_b2e * AUV.uvw;
    
    % 2 ANGULAR VELOCITY from BODY TO EARTH
    R_b2e_r = transpose( [1,    0,                 -sin(AUV.rpy(2)); ...
                          0,    cos(AUV.rpy(1)),    cos(AUV.rpy(2))*sin(AUV.rpy(1)); ...
                          0,    -sin(AUV.rpy(1)),   cos(AUV.rpy(2))*cos(AUV.rpy(1))]);
    % new earth angles (rpy) = body to earth * body angles (pqr)
    AUV.dot_rpy = R_b2e_r * AUV.pqr;
    
    
    % UPDATE EARTH FRAME --> Time affects position + orientation (update)
    AUV.xyz = AUV.xyz + AUV.dot_xyz*dt; % example: x2 = x1 + x_velocity * delta_t
    AUV.rpy = AUV.rpy + AUV.dot_rpy*dt;

end

