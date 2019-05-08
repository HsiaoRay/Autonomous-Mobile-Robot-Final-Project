function z_expect = hBeacon(q,tagNum,beaconLoc)

x = q(1);
y = q(2);
theta = q(3);

R = [cos(theta) , sin(theta);
    -sin(theta) , cos(theta)];

z_expect = zeros(2*length(tagNum),1);

for i = 1:length(tagNum)
    
    index = ismember(beaconLoc(:,1),tagNum(i));
    n = find(index == 1);
    q_beacon_g = beaconLoc(n,2:3);
    q_robot_g = [x,y];
%     q_beacon_g' - q_robot_g'
    q_beacon_local = R * (q_beacon_g' - q_robot_g');

    z_expect((2*i-1):2*i,1) = q_beacon_local-[0.13;0];
    
end



end