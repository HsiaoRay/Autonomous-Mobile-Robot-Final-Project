function particles = beacon_localization(dataStore, particles, beaconLoc, beacon_size, func_gt)

    beacon_num = size(dataStore.beacon, 1) - beacon_size;
    beacon_idx = dataStore.beacon(end-beacon_num+1:end, 3);
    beacon = dataStore.beacon(end-beacon_num+1:end, 4:5)';
    func_htBeacon = @(pose) hBeacon(pose, beacon_idx, beaconLoc);
    particles = particleFilter(particles, beacon(:), func_gt, func_htBeacon);