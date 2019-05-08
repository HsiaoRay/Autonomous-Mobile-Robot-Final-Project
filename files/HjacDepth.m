function [Hdeptht] = HjacDepth(pose, map,sensorOrigin,num_points)
% HjacDepth: compute the jacobian of depth measurement of the robot 
% 
%   INPUTS
%       pose   	3-by-1 pose vector in global coordinates (x,y,theta)
%       map         	N-by-4 matrix containing the coordinates of walls in 
%                   	the environment: [x1, y1, x2, y2]
%       sensorOrigin	origin of the sensor-fixed frame in the robot frame: [x y]
%       num_points      the number of points to return taken in regular intervals 
%                       horizontally across the range
% 
%   OUTPUTS
%       depth       	num_points-by-1 vector of ranges (meters)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 2
%   Liao, Zhihao 
    delta = 0.00000000001;
    depth = depthPredict(pose,map,[0.13, 0],9);
    depth2 = depthPredict([pose(1)+delta;pose(2);pose(3)],map,[0.13, 0],9);
    Hdeptht(:,1) = (depth2-depth)/delta;
    depth2 = depthPredict([pose(1);pose(2)+delta;pose(3)],map,[0.13, 0],9);
    Hdeptht(:,2) = (depth2-depth)/delta;
    depth2 = depthPredict([pose(1);pose(2);pose(3)+delta],map,[0.13, 0],9);
    Hdeptht(:,3) = (depth2-depth)/delta;
    