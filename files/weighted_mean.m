function mu = weighted_mean(particles)
    
    tmp = sortrows(particles, 4);
    tmp = tmp(end-9:end,:);
    tmpwt = tmp(:,4);
    tmp = tmp(:,1:3);
    tmpwt = tmpwt / sum(tmpwt);
    mu = sum(tmp.*[tmpwt,tmpwt,tmpwt]);