function[depth] = depthPredict(robotPose,map,sensorOrigin,num_points)
% RANGEPREDICT: predict the depth measurements given the expected
% range measurements
% 
%   depth = depthPredict(robotPose,map,sensorOrigin,num_points) returns an 
%   array of floats,  each element of [depth_array] represents the depth of 
%   the point in meters, from the expected range measurements.
% 
%   INPUTS
%       robotPose   	3-by-1 pose vector in global coordinates (x,y,theta)
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
    angles = 27/180*pi:-54/(num_points-1)/180*pi:-27/180*pi;
    [range] = rangePredict(robotPose,map,sensorOrigin,transpose(angles));
    K = size(range,1);
    depth = zeros(K, 1);
    for i = 1:1:K
        depth(i) = range(i)*cos(angles(i));
    end
    depth(depth>10)=10;
    depth(depth<0.175)=0;
