function xy = depthInv(data_depth,pose,angle_sensor)


xy = zeros(2,9);
for k = 1:9,

    direc = angle_sensor(k);
    r = data_depth(k) / cos(direc); %  Convert from depth to range
    y2 = r * sin(direc); % find the end point of the measurement beam
    x2 = data_depth(k) + 0.13;
    xy(:,k) = robot2global(pose,[x2,y2])'; % convert the end point to global frame
    
end



end