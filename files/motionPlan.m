function [goals, wayPoints, next] = motionPlan(goals, labmap, q_start, radius)

    mindistance = Inf;
    next = -1;
    PRM = visibility_sample(labmap, 1000, radius);
    for i = 1:size(goals,1)
        q_goal = goals(i,:);
        if ~isempty(findPath(labmap, q_start, q_goal, PRM, 0.16))
            [path, fcost] = findPath(labmap, q_start, q_goal, PRM, 0.16);
        else
            path = [];
        end
        if ~isempty(path)
            path = path(2:end-1);
            if  fcost < mindistance
                mindistance = fcost;
                next = i;
                wayPoints = [];
                for j = 1:length(path)
                    wayPoints = [wayPoints;PRM{path(j)-1, 2}];
                end
            end
        end
    end
    q_goal = goals(next,:);
    wayPoints = [wayPoints; q_goal];