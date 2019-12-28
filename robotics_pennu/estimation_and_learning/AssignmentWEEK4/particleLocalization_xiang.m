% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization_xiang(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% the number of grids for 1 meter.
myResol  = param.resol;
% the origin of the map in pixels --> Refer to <<CourseraProgWeek3Instruction.pdf>>
myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,1) were measured.

% Decide the number of particles, M. Not confused with the outside M, which represents map.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 100;                       % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:, 1), [1, M]);
W = ones(1, M) / M;
for j = 2:N    
    for i = 1:M
        dist = 0.025 + randn(1,1) * 0.1;
        heading = P(3, i) + randn(1, 1) * 0.15; %heading is the sum of the previous one plus a random value
        P(1, i) = P(1, i) + dist * cos(heading);
        P(2, i) = P(2, i) - dist * sin(heading);
        P(3, i) = heading;
        
        ids = randsample(size(ranges, 1), 100); %sample the lidar
        phi = P(3, i) + scanAngles(ids);
        
        x_occ = ranges(ids, j) .* cos(phi) + P(1, i);
        y_occ = -ranges(ids, j) .* sin(phi) + P(2, i);
        
        i_x_occ = ceil(myResol * x_occ) + myOrigin(1);
        i_y_occ = ceil(myResol * y_occ) + myOrigin(2);
        
        
        N_occ = size(i_x_occ, 1);
        corr = 0;
        for k = 1:N_occ
            if (i_x_occ(k) > size(map, 2)) || (i_x_occ(k) < 1)
                continue;
            end
            if (i_y_occ(k) > size(map, 1)) || (i_y_occ(k) < 1)
                continue;
            end
            
            if map(i_y_occ(k), i_x_occ(k)) < 0
                %corr = corr - 5 * map(i_y_occ(k), i_x_occ(k));
                corr = corr + map(i_y_occ(k), i_x_occ(k)) + 0.5;
            end
            if map(i_y_occ(k), i_x_occ(k)) > 1
                %corr = corr + 10 * map(i_y_occ(k), i_x_occ(k));
                corr = corr + map(i_y_occ(k), i_x_occ(k)) + 0.5;
            end
        end
        W(i) = W(i) + corr;
    end
    W_norm = W / sum(W);
    
    [W_sorted, sort_ids] = sort(W_norm, 'descend');
    W_sample = W_sorted(1:80);
    P_sorted = P(:, sort_ids);
    P_sample = P_sorted(:, 1:80);
    
    %myPose(:, j) = P_sorted(:, 1);
    myPose(:, j) = mean(P_sorted(:, 1:10), 2);  %The two methods show similar in results.
    
    p_ids = randsample(80, M, 'true', W_sample);
    W = W_sample(p_ids');
    P = P_sample(:, p_ids');
    
    if(mod(j,100)==0)
        fprintf(1,'j = %d\n',j);
    end    
end

end
