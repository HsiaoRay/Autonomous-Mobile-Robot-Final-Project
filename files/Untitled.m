    load('compMap.mat')
%     q_start = [q_start(1),q_start(2)];
%     q_goal = [q_goal(1), q_goal(2)];
    q_start = waypoints(2,:);
    q_goal = waypoints(1,:);
    radius = 0.2;
    func = @(workspace, n) visibility_sample(workspace, n, radius);
    map = [map;optWalls(1,:)];
    [PRM] = func(map,1000);
    
    [path] = findPath([map;optWalls], q_start, q_goal, PRM, radius);
    for i =1:size(map,1)
        hold on
        plot(map(i,[1,3])',map(i,[2,4]),'k')
    end
    PRM = [{1}, {q_start}, {0}; PRM; {size(PRM,1)+2} ,  {q_goal}, {0}];
    ans = [];
    for i = 1:length(path)
        ans = [ans;PRM{path(i), 2}];
    end
    ans