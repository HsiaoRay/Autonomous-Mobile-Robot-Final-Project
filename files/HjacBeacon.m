function H = HjacBeacon(q,tagNum,beaconLoc)


delta = 0.00000000001;

n = size(beaconLoc,1);

z = hBeacon(q,tagNum,beaconLoc);
H = zeros(2*length(tagNum),3);
for i=1:3
    
    q_perturb = q;
    q_perturb(i) = q_perturb(i) + delta;
    
    z_perturb = hBeacon(q_perturb,tagNum,beaconLoc);
    
    H(:,i) = (z_perturb - z) / delta;
    
end


end