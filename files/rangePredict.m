function[range] = rangePredict(robotPose,map,sensorOrigin,angles)
% RANGEPREDICT: predict the range measurements for a robot operating
% in a known map.
% 
%   RANGE = RANGEPREDICT(ROBOTPOSE,MAP,ROBOTRAD,ANGLES) returns 
%   the expected range measurements for a robot operating in a known 
%   map. 
% 
%   INPUTS
%       robotPose   	3-by-1 pose vector in global coordinates (x,y,theta)
%       map         	N-by-4 matrix containing the coordinates of walls in 
%                   	the environment: [x1, y1, x2, y2]
%       sensorOrigin	origin of the sensor-fixed frame in the robot frame: [x y]
%       angles      	K-by-1 vector of the angular positions of the range
%                   	sensor(s) in robot coordinates, where 0 points forward
% 
%   OUTPUTS
%       range       	K-by-1 vector of ranges (meters)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 2
%   Liao, Zhihao 
    x1 = robotPose(1) + sensorOrigin(1);
    y1 = robotPose(2) + sensorOrigin(2);
    K = size(angles,1);
    range = Inf(K,1);
    
    for i = 1:1:K
        theta = robotPose(3) + angles(i);
        for j = 1:1:size(map,1)          
            x2 = x1 + 9999*cos(theta);
            y2 = y1 + 9999*sin(theta);
            wall = map(j,:);
            x3  =wall(1); y3 = wall(2); x4 = wall(3); y4 = wall(4);
            [isect,~,~,ua]= intersectPoint(x1,y1,x2,y2,x3,y3,x4,y4);
            if isect == 1
                range(i) = min(range(i), ua*9999);
            end
            
        end
    end
