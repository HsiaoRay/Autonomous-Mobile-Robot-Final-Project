function [PRM] = visibility_sample(map, n, radius)
% Creat a visibility-based PRM 
%
% INPUTS:
%       map        n x 16 array of the walls of the obstacles
%       n                the number of vertices in theroadmap
%       BoundaryTopRightCorner = 1x2 vector capturing the [x y] coordinates of the top right 
%                      corner of the map. Assume the bottom left corner is [0 0]
%             
% OUTPUTS:
%       PRM              probability roadmap, n x 3 cell of the node number, the 
%                        positin of this node, and all the node numbers and cost 
%                        of the nodes connected to this node
% 
% by Zhihao Liao

% for Homework 8

if size(map,2) == 4
    new_map = zeros(size(map,1),16);
    
    for i = 1:4
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
    for i = 5:size(map,1)
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

i = 1;
samples = [];
% sampling using halton_sample
while (1)
    r2 = i;
    x = 0;
    p = -1;
    while(r2 > 0)
        x = x + mod(r2,2)*(2^p);
        r2 = (r2 - mod(r2,2)) / 2;
        p = p - 1;
    end
    r3 = i;
    y = 0;
    p = -1;
    while(r3 > 0)
        y = y + mod(r3,3)*(3^p);
        r3 = (r3 - mod(r3,3)) / 3;
        p = p - 1;
    end
    x = x * (boundary(3)-boundary(1)) + boundary(1);
    y = y * (boundary(4)-boundary(2)) + boundary(2);
    
    samples = [samples; x, y];
    
    i = i+1;
    if i > 5*n
        break;
    end
end

% check if the samples are in the polygon, if yes, remove it
for j = 1:size(map,1)
    xq = samples(:,1);
    yq = samples(:,2);
    tmp = [];
    for k = 1:8
        if map(j,2*k-1) ~= 0 && map(j,2*k) ~= 0
            tmp = [tmp;map(j,2*k-1:2*k)];
        end
    end
    [in,on]  =inpolygon(xq,yq,tmp(:,1),tmp(:,2));
    tmp = in+on;
    samples(tmp>0,:) = [];
end

samples = samples(1:n,:);

% find all wals of all polygons
walls = [];
for i = 1:size(map,1)
    tmp = map(i,:);
    tmp = size(tmp(tmp~=0),2);
    walls = [walls; map(i,tmp-1), map(i,tmp), map(i,1), map(i,2)];
    for j = 2:tmp/2
        walls = [walls; map(i,2*j-3), map(i,2*j-2), ...
                                     map(i,2*j-1), map(i,2*j)];
    end
end

v = [1];
e = [];
unconnected_groups = {[1]};
for i = 2:size(samples,1)
    tmp = [];
    
    % find all node(v(j)) that can be connected to i, store them in tmp
    for j = 1:size(v,2)
        m = 0;
        x1 = samples(i,1); y1 = samples(i,2); x2 = samples(v(j),1); y2 = samples(v(j),2);
        for k = 1:size(walls,1)
            x3 = walls(k,1); y3 = walls(k,2); x4 = walls(k,3); y4 = walls(k,4);
            [isect,~,~,~]= intersectPoint(x1,y1,x2,y2,x3,y3,x4,y4);
            if isect == 1
                m = 1;
            end
        end
        if m == 0
            tmp = [tmp, v(j)];
        end
    end
    
    % if tmp is empty, node i is not connected to any other node, add to v
    if isempty(tmp)
        v = [v, i];
        unconnected_groups{1, end+1} = [i];
    % if more than two nodes can be connected to node i
    elseif size(tmp,2) >= 2
        groups = [];
        % find all the groups the nodes are in
        for j =1:size(tmp,2)
            for k = 1:size(unconnected_groups,2)
                if ismember(tmp(j),unconnected_groups{1,k})
                    if ~ismember(k,groups)
                        groups = [groups, k];
                    end
                end
            end
        end
        % if there are more than two groups, add node i to v
        if size(groups,2)>=2
            v = [v, i];
            unconnected_groups{1,groups(1)} = [unconnected_groups{1,groups(1)}, i];
            % connect the group in groups
            for j = 2:size(groups,2)
                unconnected_groups{1,groups(1)} = [unconnected_groups{1,groups(1)},...
                                                        unconnected_groups{1,groups(j)}];
                unconnected_groups{1,groups(j)} = []; 
            end
            unconnected_groups(cellfun('isempty',unconnected_groups)) = [];
            % add [i,j] to e for all the j in tmp
            for j =1:size(tmp,2)
                e = [e; i, tmp(j)];
            end
        end
    end
end

% for each edge, find the node number of its two ends
nodes = samples(v',:);
edge = [samples(e(:,1),:),samples(e(:,2),:)];
roadmap = {};
edge_n = zeros(size(edge,1),2);
for j = 1:size(edge,1)
    for i = 1:size(nodes,1)
        if edge(j,1) == nodes(i,1) && edge(j,2) == nodes(i,2)
            edge_n(j,1) = i;
        elseif edge(j,3) == nodes(i,1) && edge(j,4) == nodes(i,2)
            edge_n(j,2) = i;    
        end
    end
end

% for each nodes, build the road map of its number, position and all the
% neighbors connected to it
for i = 1:size(nodes,1)
    roadmap{i,1} = i+1;
    roadmap{i,2} = nodes(i,:);
    roadmap{i,3} = [];
    for j = 1:size(edge,1)
        cost = sqrt((edge(j,3) - edge(j,1))^2+(edge(j,4) - edge(j,2))^2);
        if edge(j,1) == nodes(i,1) && edge(j,2) == nodes(i,2)
            roadmap{i,3} = [roadmap{i,3};edge_n(j,2)+1, cost];
        elseif edge(j,3) == nodes(i,1) && edge(j,4) == nodes(i,2)
            roadmap{i,3} = [roadmap{i,3};edge_n(j,1)+1, cost];
        end
    end
end

% plot roadmap
% for i =1: size(roadmap,1)
%     for j = 1:size(roadmap{i,3},1)
%         hold on;
%         p1 = roadmap{i,2};
%         tmp = roadmap{i,3};
%         tmp = tmp(j,1);
%         p2 = roadmap{tmp-1,2};
%         plot([p1(1),p2(1)],[p1(2),p2(2)],'r')
%     end
% end

PRM = roadmap;
            
            
