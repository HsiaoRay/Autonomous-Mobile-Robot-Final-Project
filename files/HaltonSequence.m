function s = HaltonSequence(n)
% 
% inputs:  
% 
%           n:   integer, number of samples
% 
% outputs:
% 
%           s:   n-by-2 array, sample space with n samples
% 

%% ========================================================================
% bases for 2D space
p1 = 5;
p2 = 7;


s = zeros(n,2);
for i = 1:n
    
    p_1 = changeBase(i,p1);
    a_1 = zeros(p1,1);
    for j1 = 1:p1
        a_1(j1,1) = p1^(-j1);   
    end
    s(i,1) = p_1 * a_1;
    
    
    p_2 = changeBase(i,p2);
    a_2 = zeros(p2,1);
    for j2 = 1:p2
        a_2(j2,1) = p2^(-j2);   
    end
    s(i,2) = p_2 * a_2;
    
end



end