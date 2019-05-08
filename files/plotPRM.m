function plotPRM(obstacles,q_start,q_goal,n,range)



sampleFun = @(n) HaltonSequence(n);
workspace.range = range;      
workspace.obstacles = obstacles;
% roadmap = buildPRM(workspace,n,sampleFun,q_start,q_goal);
roadmap = buildVisibilityPRM(workspace,n,sampleFun,q_start,q_goal);

figure
for i=1:size(obstacles,2),
    
    polygon = obstacles{i};
    
    for j = 1:4
        plot(polygon(j,[1,3]),polygon(j,[2,4]),'k--','LineWidth',1)
    
        hold on
    end
    
end
axis([range(1,1)-0.2,range(1,2)+0.2,range(2,1)-0.2,range(2,2)+0.2])

V = roadmap.vertices;
plot(V(:,1),V(:,2),'k.','Markersize',25)
for i = 3:size(V,1)
    
    text(V(i,1),V(i,2),num2str(i-2),'FontSize',18)
    
    
end
hold on

E = roadmap.edges;

for i = 1:size(E,1),
    
    plot(E(i,[1,3]),E(i,[2,4]),'r--')
    
    hold on
    
end


[path,B] = findPath(V,E);


if B == 1,
    for i = 1:(size(path,1)),
    
        plot(path(i,1),path(i,2),'g.','Markersize',40)
        hold on
        
    
    end
    plot(path(:,1),path(:,2),'g--')
end



xlabel('x (m)')
ylabel('y (m)')
% title('PRM roadmap with deterministic low-dispersion sampling, n = 10')
title('PRM roadmap with uniform random sampling, n = 20')



end