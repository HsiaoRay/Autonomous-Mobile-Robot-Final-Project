function [dataStore]= finalCompetition(CreatePort,DistPort,TagPort,tagNum,maxTime, offset_x, offset_y)
% 
%   INPUTStype
%       CreatePort  Create port object (get from running RoombaInit)
%       DistPort    Depth ports object (get from running CreatePiInit)
%       TagPort      Tag ports object (get from running CreatePiInit)
%       tagNum      robot number for overhead localization
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data

% 
%   NOTE: Assume differential-drive robot whose wheels turn at a constant 
%         rate between sensor readings.
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots

%% ==========================================================
% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    DistPort = CreatePort;
    TagPort = CreatePort;
    tagNum = CreatePort;
    maxTime = 4200;
elseif nargin < 3
    TagPort = CreatePort;
    tagNum = CreatePort;
    maxTime = 4200;
elseif nargin < 4
    tagNum = CreatePort;
    maxTime = 4200;
elseif nargin < 5
    maxTime = 4200;
end


if ~exist('offset_x','var') || isempty(offset_x), offset_x = 0.13; end
if ~exist('offset_y','var') || isempty(offset_y), offset_y = 0; end
calibrate = [offset_x, offset_y];

%% ========================================================================
% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;
% global truthPose; 

% initialize datalog struct (customize according to needs)
dataStore = struct('odometry', [], ...
                   'bump', [], ...
                   'rsdepth', [], ...
                   'beacon', [],...
                   'pose', [],...
                   'visited_waypoints',[],...
                   'finalMap',[]);
% truthPose = [];

% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;
tic

% plot map
figure
load('compMap.mat');
labmap = map;
plot_map(labmap)
xlabel('X / m')
ylabel('Y / m')

%% =============================================================================
% initial localization

% initializing PF
number = 100;
tmp = repmat(waypoints,[number,1]);
x = tmp(:,1); y= tmp(:,2); theta = (rand([number*size(waypoints,1),1]) - 0.5)*2*pi;
% particles: 30 x 4 array of [x, y, theta, weight]
particles =[x(:), y(:), theta(:), ones(size(theta,1),1)/size(theta,1);];

i = 0;
beacon_size = 0;
t = 0;
while i <= 3
    
    % set control
    fwdVel = 0; angVel = -pi/20;
    maxV = 0.08;  wheel2Center = 0.13;
    [cmdV, cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center);
    SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
    
    % READ & STORE SENSOR DATA
    [dataStore]=readStoreSensorData(CreatePort,DistPort,TagPort,dataStore);
    odom = dataStore.odometry(end, 2:3);    
    func_gt = @(pose) integrateOdom(pose, odom(1), odom(2));
    
    if t >= 60
        % do PF using dapth data
        func_htDepth = @(pose) depthPredict(pose,labmap,calibrate,9);
        depth = transpose(dataStore.rsdepth(end, 3:11));
        particles = particleFilter(particles, depth, func_gt, func_htDepth);
        i = i + 0.3;
    else
        % Do particle filter and updating particles
        if beacon_size ~= size(dataStore.beacon, 1)
            particles = beacon_localization(dataStore, particles, beaconLoc, beacon_size, func_gt);

            % update loop variable
            beacon_size = size(dataStore.beacon, 1);
            i = i + 1;
        end
    end
    
    % plot particles
    x = particles(:,1)'; y = particles(:,2)';
    plot(x,y,'r*')
    plot_map(labmap); xlabel('X / m'); ylabel('Y / m')
    hold off
    
    t = t + 1;
end

SetFwdVelAngVelCreate(CreatePort, 0,0);
BeepRoomba(CreatePort)
figure
%% ============================================================================
% run competition

% initializing PF
mu = weighted_mean(particles);
tmp = repmat(mu,size(waypoints,1),1);
[~,i] = min(sum((tmp(:,1:2) - waypoints).^2, 2));
dataStore.pose = tmp(i,1:2);
number = 150;
x = repmat(mu(1), number, 1);
y = repmat(mu(2), number, 1);
theta = (rand(number, 1) - 0.5)*2*pi;
%  particles: 30 x 4 array of [x, y, theta, weight]
particles =[x, y, theta, ones(size(theta,1),1)/size(theta,1)];


% initial mapping
see = 0;
mapping_results = ones(1,size(optWalls, 1));
newmap = [map, zeros(size(map,1),1); optWalls, ones(size(optWalls,1),1)];

% initial motion plan
goals = [waypoints; ECwaypoints];

dataStore.visited_waypoints = [dataStore.visited_waypoints; goals(i,:)];

goals(i,:) = [];
labmap_motionPlan = map;

% initializing visit waypoints
closeEnough = 0.1;
Epsilon = 0.13;
doplan = 1;
count_num = 0;
while toc < maxTime
    
    % motion plan generate waypoints
    if doplan == 1
        if isempty(goals)
            disp('=========END============')
            break;
        else
            close
            figure(2)
            radius = 0.25;
            q_start = mu(1:2);
            [goals, wayPoints, next] = motionPlan(goals, labmap_motionPlan, q_start, radius);
            hold on;
            plot(wayPoints(:,1),wayPoints(:,2),'b')
            plot_map(labmap_motionPlan); xlabel('X / m'); ylabel('Y / m')
            gotopt = 1;
            figure(1)
        end
        doplan = 0;
    end
          
    % READ & STORE SENSOR DATA
    [dataStore]=readStoreSensorData(CreatePort,DistPort,TagPort,dataStore);
    odom = dataStore.odometry(end, 2:3);    
    func_gt = @(pose) integrateOdom(pose, odom(1), odom(2)); 

    % Do particle filter and updating particles
    if beacon_size ~= size(dataStore.beacon, 1)
        particles = beacon_localization(dataStore, particles, beaconLoc, beacon_size, func_gt);
        beacon_size = size(dataStore.beacon, 1);
    else
        depth = transpose(dataStore.rsdepth(end, 3:11));
        func_htDepth = @(pose) depthPredict(pose,labmap,calibrate,9);
        particles = particleFilter(particles, depth, func_gt, func_htDepth);
    end

    % update localization
    mu = weighted_mean(particles);
    dataStore.pose = [dataStore.pose; mu(1),mu(2)];
    
    
    
    % control
    velX = wayPoints(gotopt,1) - mu(1); velY = wayPoints(gotopt,2) - mu(2);
    maxV = 0.1; wheel2Center = 0.13;
    tmp = sqrt(velX^2 + velY^2); velX = velX / tmp; velY = velY / tmp;
    [fwdVel,angVel] = feedbackLin(velX, velY, mu(3), Epsilon);
    [cmdV, cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center);
    SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW);
    
    % predict maps
    angles = 27/180*pi:-54/(9-1)/180*pi:-27/180*pi;
    [~,B, newmap, n_walls, see] = optwallpredict(mu,newmap,calibrate,angles, see);
    if B == 1
        SetFwdVelAngVelCreate(CreatePort, 0,0);
        r = 0.25;
        optPolygons = {};
        for i = 1:size(n_walls,1)
            line = newmap(n_walls(i),1:4);
            optPolygons = [optPolygons;lineExpand(line,r)];
        end
        [labmap, labmap_motionPlan,mapping_results] = ...
              mappingOptWalls(n_walls,newmap(:,1:4),mapping_results,optPolygons,CreatePort,DistPort,TagPort,dataStore, mu, angles, size(map,1),labmap_motionPlan);
        doplan = 1;
    end
    
    % if close enough, stop and go to next waypoint
    if (mu(1) - wayPoints(gotopt,1))^2 + (mu(2) - wayPoints(gotopt,2))^2 <= closeEnough^2
        if gotopt >= size(wayPoints,1) 
            if count_num > 2
                doplan = 1;
                dataStore.visited_waypoints = [dataStore.visited_waypoints; goals(next,:)];

                goals(next,:) = [];
                SetFwdVelAngVelCreate(CreatePort, 0,0);
                BeepRoomba(CreatePort)
                count_num = 0;
                pause(0.2)
                gotopt = gotopt + 1;
            else
                count_num = count_num + 1;
            end
        else
            gotopt = gotopt + 1;
        end
        
    end
    
    
    % plot
    plot(dataStore.pose(:,1),dataStore.pose(:,2),'r')
    hold on
    hold on;
    plot(dataStore.visited_waypoints(:,1),dataStore.visited_waypoints(:,2),'b*')
%     [noRobotCount, truthPose]=readTruthPose(tagNum,noRobotCount, truthPose);
%     if ~isempty(truthPose)
%         plot(truthPose(:, 2), truthPose(:, 3),'b')
%     end
    idx = newmap(size(map,1)+1:end,5);
    not_determined = newmap(size(map,1)+1:end,1:4);
    not_determined = not_determined(idx==1,:);
    for i = 1:size(not_determined, 1)
        plot(not_determined(i,[1,3])',not_determined(i,[2,4])','r')
    end
    plot_map(labmap_motionPlan); xlabel('X / m'); ylabel('Y / m')
    
    dataStore.finalMap = labmap_motionPlan;
    
%     pause(0.1);
end

%% =====================================================================
% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0);