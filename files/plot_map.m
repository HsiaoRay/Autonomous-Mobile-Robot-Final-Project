function plot_map(map)
% plotGridBelief: plot map 
%                          
%   INPUTS
%       map:     n x 4 matrix contain all the walls 

% plot the map
hold on;
for i =1: size(map,1)
x = [map(i,1),map(i,3)];
y = [map(i,2),map(i,4)];
hold on;
plot(x,y,'black');
end