function [Gt] = GjacDiffDrive(pose, distance, angle)
% computeZbar: returns the nearest wall position of the robot
%                          
%   INPUTS          
%       pose         1 x 3 vectors with the x, y, and angle information of the robot
%       distance     distance traveled from the measurement
%       angle        angle turned from the measurement
%   OUTPUTS
%       Gt           3 X 3 matrix of e the Jacobian of the update and measurement functions
%                    at a given state
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 3
%   Liao, Zhihao 
        theta = pose(3);
        if angle == 0
            Gt = [1 0 -distance*sin(theta);
                  0 1  distance*cos(theta);
                  0 0  1];
        else
            Gt = [1 0 distance/angle*(cos(theta+angle)-cos(theta));
                  0 1 distance/angle*(sin(theta+angle)-sin(theta));
                  0 0 1];
        end
    