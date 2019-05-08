function [labmap, labmap_motionPlan, mapping_results] = mappingOptWalls(n_walls,labmap_full,mapping_results,optPolygons, CreatePort,DistPort,TagPort,dataStore, miu, angles,num_walls,labmap_motionPlan)

%    Inputs:
% 
%               n_walls:           1-by-n array, opt-walls that seen for the first time.
% 
%               labmap_full:       13-by-4 array, full map, including non-optional and
%                                  optional map.
% 
%               mapping_results:   1-by-3 array of boolean, indicating if
%                                  the optional wall exists or not.
% 
%               mapping_checked:   1-by-3 array of boolean, indicating if
%                                  the optional wall is seen or not.
% 
%               optPolygons:       1 cell of n arrays, polygons defining the optional walls.



% scoring table for the first time seen opt-walls
mapping_optWalls = zeros(1,length(n_walls));

% take depth measurement for 5 seconds
for ii = 1:5
            
     [dataStore]=readStoreSensorData(CreatePort,DistPort,TagPort,dataStore); % call depth measurement
     xy = depthInv(dataStore.rsdepth(end,3:11),miu,angles); % convert to global coordinates
     
     % check if the depth measurement is inside the opt-walls   
     In = zeros(size(xy,2),length(n_walls));
     for jj = 1:size(xy,2) % loop over all 9 measurements
         for kk = 1:length(n_walls) % loop over all first time seen opt-walls
             polygon = optPolygons{kk};
             In(jj,kk) = inpolygon(xy(1,jj),xy(2,jj),polygon(:,1),polygon(:,2));
         end
     end
     
     % award 1 score to the opt-wall if more than one measurement fall into that wall        
     for kk2 = 1:length(n_walls)
         numPts = length(find(In(:,kk2) == 1));
         if numPts > 1
             mapping_optWalls(kk2) = mapping_optWalls(kk2) + 1;
         end
     end
     pause(0.5)
end
        
% update map based on scoring      
for kk3 = 1:length(n_walls)    
    if mapping_optWalls(kk3) < 5 % if only 1 or no measurement fall into the opt-wall, there is no wall
        mapping_results(n_walls(kk3)-num_walls) = 0;
    end
end

% update map
labmap = labmap_full;
labmap(find(mapping_results == 0)+num_walls,:) = [];

add_wall = mapping_results(mapping_results(n_walls-num_walls) == 1);
labmap_motionPlan = [labmap_motionPlan; labmap_full(add_wall+num_walls,:)];

end