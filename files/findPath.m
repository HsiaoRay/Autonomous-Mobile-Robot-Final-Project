function [path, fcost] = findPath(map, start, goal, roadmap, radius)
% Find a shortest path in a roadmap.
%
% INPUTS:
%       filename         String containing the .txt file containing the map vertices 
%       start            1 x 2 array of the start point of the roadmap
%       goal             1 x 2 array of the goal point 
%       BoundaryTopRightCorner = 1x2 vector capturing the [x y] coordinates of the top right 
%                      corner of the workspace. Assume the bottom left corner is [0 0]
%       roadmap          n x 3 cell of the node number, the positin of this
%                        node, and all the node numbers and cost of the nodes
%                        connected to this node
% OUTPUTS:
%       path             m x 2 array of edges in the path   
%
% by Zhihao Liao
% for Homework 8

if size(map,2) == 4
    new_map = zeros(size(map,1),16);
    for i = 1:size(map,1)
        if map(i,2) == map(i,4)
            left = min(map(i,1),map(i,3));
            right = max(map(i,1),map(i,3));
            new_map(i,1:8) = [left-radius, map(i,2)-radius, left-radius, map(i,2)+radius,...
                              right+radius, map(i,4)+radius, right+radius, map(i,4)-radius];
        elseif map(i,1) == map(i,3)  
            down = min(map(i,2),map(i,4));
            up = max(map(i,2),map(i,4));
            new_map(i,1:8) = [map(i,1)-radius, down-radius, map(i,1)+radius, down-radius,...
                              map(i,3)+radius, up+radius, map(i,3)-radius, up+radius];
        end
    end
    boundary = [min(min(map(:,[1,3]))), min(min(map(:,[2,4]))),...
                max(max(map(:,[1,3]))), max(max(map(:,[2,4])))];
    map = new_map;
end

% find all wals of all polygons
walls = [];
workspace = map;
for i = 1:size(workspace,1)
    tmp = workspace(i,:);
    tmp = size(tmp(tmp~=0),2);
    walls = [walls; workspace(i,tmp-1), workspace(i,tmp), workspace(i,1), workspace(i,2)];
    for j = 2:tmp/2
        walls = [walls; workspace(i,2*j-3), workspace(i,2*j-2), ...
                                     workspace(i,2*j-1), workspace(i,2*j)];
    end
end

% add the start point and goal to the roadmap
roadmap = [{[1], start, []}; roadmap];
roadmap = [roadmap; {[size(roadmap,1)+1], goal, []}; ];
for i = 2:size(roadmap,1)-1
    x1 = start(1); y1 = start(2); x2 = goal(1); y2 = goal(2);
    node = roadmap{i,2};
    m1 = 0;
    m2 = 0;
    for k = 1:size(walls,1)
        x3 = walls(k,1); y3 = walls(k,2); x4 = walls(k,3); y4 = walls(k,4);
        if (node(1) == x3 && node(2) == y3)|| (node(1) == x4 && node(2) == y4) ||...
                (node(1) == x3 && node(2) == y3)|| (node(1) == x4 && node(2) == y4)
            continue
        end
        [isect1,~,~,~]= intersectPoint(x1,y1,node(1),node(2),x3,y3,x4,y4);
        [isect2,~,~,~]= intersectPoint(x2,y2,node(1),node(2),x3,y3,x4,y4);
        if isect1 == 1
            m1 = 1;
        end
        if isect2 == 1
            m2 = 1;
        end
    end
    if m1 == 0
        roadmap{1,3} = [roadmap{1,3}; i, sqrt((x1-node(1))^2+(y1-node(2))^2)];
    end
    if m2 == 0
        roadmap{end,3} = [roadmap{end,3}; i, sqrt((x2-node(1))^2+(y2-node(2))^2)];
    end
end
% only connect start and goal to the nearest 3 points
if ~isempty(roadmap{1,3})
    
    tmp1 = sortrows(roadmap{1,3},2);
    if size(tmp1,1) <= 3
        roadmap{1,3} = tmp1;
    else
        roadmap{1,3} = tmp1(1:3,:);
    end
end
if ~isempty(roadmap{end,3})
    
    tmp2 = sortrows(roadmap{end,3},2);
    if size(tmp2,1) <= 3
        roadmap{end,3} = tmp2;
    else
        roadmap{end,3} = tmp2(1:3,:);
    end
end
for i = 1:size(roadmap{end,3},1)
    roadmap{roadmap{end,3}(i,1),3} = [roadmap{roadmap{end,3}(i,1),3}; ...
                                        size(roadmap,1), roadmap{end,3}(i,2)];
end


% plot workspace
 hold on
 plot(start(1),start(2),'b*')
%  plot(goal(1),goal(2),'r*')

for i =1: size(roadmap,1)
    for j = 1:size(roadmap{i,3},1)
        p1 = roadmap{i,2};
        tmp = roadmap{i,3};
        tmp = tmp(j,1);
        p2 = roadmap{tmp,2};
        plot([p1(1),p2(1)],[p1(2),p2(2)],'r')
        hold on;
    end
end

% initialize
goal = size(roadmap,1);
Q = roadmap{1,3};
cost = {};
for i= 1:size(roadmap,1)
    cost{i,1} = Inf;
    cost{i,2} = [];
end
cost{1,1} = 0;
for i = 1:size(Q,1)
    cost{Q(i,1),1} = Q(i,2);
    cost{Q(i,1),2} = [1];
end
visited = [1];
path = [];

% dijastra's shortest algorithm
while ~isempty(Q)
    
% cost : {distance: number, path: 1xn array}
% roadmap: {node number, node position, neighbor}
% neighbor: nx2 array [node number, distance]
% Q : nx2 array [node number, cost]

    tmp = Q(:,2);
    [~,tmp] = min(tmp);
    neighbor = roadmap{Q(tmp,1),3};
    for i = 1:size(neighbor,1)
        if ~ismember(visited,neighbor(i,1))
            % if the path from Q(tmp,1) to neighbor(i,1) is shorter than
            % neighbor(i,1)'s original cost, update the cost and path
            if Q(tmp,2) + neighbor(i,2) < cost{neighbor(i,1),1}
                cost{neighbor(i,1),1} = Q(tmp,2) + neighbor(i,2);
                cost{neighbor(i,1),2} = [cost{Q(tmp,1),2}, Q(tmp,1)];
            end
            % if neighbor(i,1) not in Q, add it to Q
            if ~ismember(Q,neighbor(i,1))
                Q = [Q; neighbor(i,1),cost{neighbor(i,1),1}];
            end
        end
    end
    
    % if get the goal, stop looping
    if Q(tmp,1) == goal
        path = [cost{Q(tmp,1),2}, goal];
        fcost = cost{Q(tmp,1),1};
        break;
    end
    
    % add Q(tmp,1) to visited
    visited = [visited, Q(tmp,1)];
    
    % remove Q(tmp,1)
    Q(tmp,:) = [];
end

% plot the shortest path
% if ~isempty(path)
%     for i = 2:size(path,2)
%         p1 = roadmap{path(i-1),2};
%         p2 = roadmap{path(i),2};
%         plot([p1(1),p2(1)],[p1(2),p2(2)],'b')
%         hold on
%     end
% end
% boundary = [min(min(map(:,[1,3]))), min(min(map(:,[2,4]))),...
%                 max(max(map(:,[1,3]))), max(max(map(:,[2,4])))];
% axis(boundary([1,3,2,4]));
% xlabel('X / m')
% ylabel('Y / m')
% title('Shortest Path')