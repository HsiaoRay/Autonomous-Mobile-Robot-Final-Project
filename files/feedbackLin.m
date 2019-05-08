function [cmdV, cmdW] = feedbackLin(velX, velY, Theta, Epsilon)
% Inputs:
%              velX : x-component of desired velocity in inertial frame (m/s)
%              velY : y-component of desired velocity in inertial frame (m/s)
%              Theta : Robot's current orientation [rad]
%              Epsilon : Distance from (0,0) used for feedback linearization

R_BI = [cos(Theta) sin(Theta); -sin(Theta) cos(Theta)];
velocity = [1 0; 0 1/Epsilon] * R_BI * transpose([velX velY]);
cmdV = velocity(1);
cmdW = velocity(2);