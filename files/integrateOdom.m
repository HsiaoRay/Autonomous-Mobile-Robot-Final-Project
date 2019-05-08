function [pose] = integrateOdom(robotPose,distance, sigma)
% integrateOdom: Given a known initial configuration (x, y, sigma) 
%of the robot within a global frame, the distance traveled and 
%the angle the robot turned ?, compute the new configuration of the robot.
% 
%   [pose] = integrateOdom(robotPose,distance, sigma) returns 
%   the new robot pose from integrating the robot odometry
% 
%   INPUTS
%       robotPose   	3-by-1 pose vector in global coordinates (x,y,theta)
%       distance     	the distance robot traveled
%       angles      	the angle the robot turned
% 
%   OUTPUTS
%       pose       	    3-by-1 pose vector in global coordinates (x,y,theta)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 2
%   Liao, Zhihao
    pose = zeros(3, 1);
    x = robotPose(1); y = robotPose(2); theta = robotPose(3);
    if sigma ~= 0
        pose(1) = x - distance/sigma*sin(theta) + distance/sigma*sin(theta+sigma);
        pose(2) = y + distance/sigma*cos(theta) - distance/sigma*cos(theta+sigma);
        pose(3) = sigma + theta;
    else
        pose(1) = x + distance*cos(theta);
        pose(2) = y + distance*sin(theta);
        pose(3) = theta;
    end
    
end
%End