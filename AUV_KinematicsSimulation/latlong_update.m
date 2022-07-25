function [AUV] = latlong_update(AUV,xyzData,latlongData)
% update_latlong_from_xyz takes in the current xyz and calculates the new lat/long from previous lat/long
% input: AUV with AUV.xyz, AUV.lat, and AUV.long
% R is constant 6378e3 m
% it is assumed that the inputted xyzData has already been updated with kinematics
% thus xyz represents the current state (end) and previous state (end-1)
% for the latlong, previous state is still (end) because we haven't updated yet, we update now!

% Establish R and R_Prime
R = 6378e3;
R_Prime = R*cosd(latlongData(1,end)); % recall, lat long are in DEGREES

% Calculate the Delta X and Y
delta_x = xyzData(1,end) - xyzData(1,end-1); %current - previous
delta_y = xyzData(2,end) - xyzData(2,end-1);

% Calculate the New Lat Long using Delta X and Y
AUV.latlong(1,1) = latlongData(1,end) + delta_x / R * 180 / pi;
AUV.latlong(2,1) = latlongData(2,end) + delta_y / R_Prime * 180 / pi;


end

