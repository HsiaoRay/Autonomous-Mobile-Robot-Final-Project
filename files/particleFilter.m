function new_set = particleFilter(particle_set, measurement, func_gt, func_ht)
% particleFilter: returns the particle set representing the location of
% the robot 
%                          
%   INPUTS
%       particle_set       Initial particle set of robot's postion
%       depth              Actual measurement data
%       func_gt            Transition function gt
%       func_ht            Measuremnt hunction ht
%   OUTPUTS
%       new_set            Upadated particle set of robot's postion 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Liao, Zhihao 

    % prediction
    wt = particle_set(:, 4);
    particle_set = particle_set(:, 1:3);
    
    M = size(particle_set,1);
    sigma = 0.02 * eye(length(measurement));
    for m = 1:M
        xt_bar(m,:) = func_gt(particle_set(m,:)') + [0.017*randn(2,1); 0.016*randn];
        wt(m,1) = mvnpdf(measurement, func_ht(xt_bar(m,:)), sigma); %* wt(m,1);
    end
    
    for i = 1:size(wt,1)
        if wt(i) <= 1e-10
            wt(i) = 1e-10;
        end
    end
    
    wt = wt/sum(wt);
    % update
    ind = transpose(1:M);
    sample = randsample(ind,M,true,wt);
    
    new_set = [xt_bar(sample,:), wt(sample)];