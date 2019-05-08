function [noRobotCount, truthPose]=readTruthPose(tagNum,noRobotCount, truthPose)
    
    % read truth pose (from overhead localization system)
    try
        [px, py, pt] = OverheadLocalizationCreate(tagNum);
        if (px == 0 && py == 0 && pt == 0 && ~isa(tagNum,'CreateRobot'))
            disp('Overhead localization lost the robot!')
            noRobotCount = noRobotCount + 1;
        else
            poseX = px; poseY = py; poseTheta = pt;
            truthPose = [truthPose ; toc poseX poseY poseTheta];
            noRobotCount = 0;
        end
    catch
        disp('Error retrieving or saving overhead localization data.');
    end