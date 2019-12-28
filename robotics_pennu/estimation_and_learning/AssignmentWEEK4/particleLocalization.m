% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

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
M = 100;                      % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P      = repmat(myPose(:,1), [1, M]); % All particle start from the same pose?
Weight = (1.0/M)*ones(1,M);      % Equi-probability initialization
%idxs   = round(linspace(1, size(ranges, 1), 100)); % Sampling the scan angles to reduce computation load.
idxs   = 1:10:size(ranges, 1); % Why the above 'idxs' make simulation very slow?
for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).

    % 1) Propagate the particles     
    %% dist    = 0.025;
    %% P(3, :) = P(3, :) + randn(1,M) * 0.3;                      
    %% P(1, :) = P(1, :) + dist .* cos(P(3, :));
    %% P(2, :) = P(2, :) - dist .* sin(P(3, :));    
    %% P(1, :) = P(1, :) + randn(1,M) * 0.1  + dist .* cos(P(3, :));
    %% P(2, :) = P(2, :) + randn(1,M) * 0.1  - dist .* sin(P(3, :));
    %% P(3, :) = P(3, :) + randn(1,M) * 0.1;                      

    %% dist    = 0.01    + randn(1,M) * 0.005;    
    %% P(1, :) = P(1, :) + dist .* cos(P(3, :));
    %% P(2, :) = P(2, :) - dist .* sin(P(3, :));
    %% P(3, :) = P(3, :) + randn(1,M) * 0.2;

    %--Alternative I
    % dist    = 0.025 + randn(1,M) * 0.01;    
    % P(1, :) = P(1, :) + dist .* cos(P(3, :));
    % P(2, :) = P(2, :) - dist .* sin(P(3, :));       
    % P(3, :) = P(3, :) + randn(1, M) * 0.1; %heading is the sum of the previous one plus a random value
    
    %--Alternative II
    dist    = 0.025 + randn(1,M) * 0.01;    
    P(1, :) = P(1, :) + dist .* cos(P(3, :));
    P(2, :) = P(2, :) - dist .* sin(P(3, :));       
    P(3, :) = P(3, :) + randn(1, M) * 0.1; %heading is the sum of the previous one plus a random value

    % 2) Measurement Update 
    scoring = zeros(M,1);

    for m = 1:M    
        % 2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
        pose    = P(:,m);
        angle   = scanAngles(idxs) + pose(3);    
        x_hit   =  ranges(idxs,j).*cos(angle) + pose(1);
        y_hit   = -ranges(idxs,j).*sin(angle) + pose(2);
        i_x_hit = ceil(x_hit*myResol) + myOrigin(1); % param.resol represents the number of grids for 1 meter 
        i_y_hit = ceil(y_hit*myResol) + myOrigin(2); % param.resol represents the number of grids for 1 meter 

        valid_hits = (i_x_hit>=1) & (i_x_hit <= size(map,2)) & (i_y_hit>=1) & (i_y_hit <= size(map,1));
        i_x_hit = i_x_hit( valid_hits); % Throw away out-of-map results.
        i_y_hit = i_y_hit( valid_hits); % Throw away out-of-map results.

        ocu_cells = sub2ind(size(map),i_y_hit,i_x_hit);
        ocu_cells = unique(ocu_cells);
        
        % 2-2) For each particle, calculate the correlation scores of the particles

        free_cells = [];
        orig = [ceil(pose(1)*myResol)+myOrigin(1),ceil(pose(2)*myResol)+myOrigin(2)]; % start point 
        
        % If the updated position of the current particle is outside of map, set its weight to 0. 
        % This is equivalent to remove this particle.
        if orig(1)<1 || orig(1)>size(map,2) || orig(2)<1 || orig(2)>size(map,1)
            fprintf(1,'j=%d, m=%d, updated particle position outside of map\n', j,m);
            Weight(m)  = 0;
            scoring(m) = 0;
            continue;
        end    

        for k = 1:length(i_x_hit)                          
            % get cells in between            
            [freex, freey] = bresenham(orig(1),orig(2),i_x_hit(k),i_y_hit(k));  
            % convert to 1d index
            free = sub2ind(size(map),freey,freex);
            free_cells = union(free_cells, free);
        end
        free_cells = unique(free_cells);

        % 2-3) Calculate correlation scoring function
        %      Can we think it as something like cross-entropy?        
        th1 = 1.4;% 55; % To be confirmed. From the original probabilistic map, it seems the threshold is around 0.5.
        th2 = -0;% 45; % To be confirmed. From the original probabilistic map, it seems the threshold is around 0.5.
        for k = 1:length(ocu_cells)
            if (map(ocu_cells(k)) > th1) % To be confirmed.
                %scoring(m) = scoring(m) + 10*(map(ocu_cells(k))-th1);
                scoring(m) = scoring(m) + 10;
            elseif(map(ocu_cells(k)) < th2)
                %scoring(m) = scoring(m) + 5*(map(ocu_cells(k))-th2);
                scoring(m) = scoring(m) - 5;
            end    
        end

        for k = 1:length(free_cells)
            if (map(free_cells(k)) < th2) % To be confirmed.
                scoring(m) = scoring(m) + 1;
            elseif(map(free_cells(k)) > th1)
                scoring(m) = scoring(m) - 5;     
            end
        end        

    end
    
    %   2-3) Update the particle weights             
    % figure; 
    % subplot(3,1,1); stem(Weight);
    % subplot(3,1,2); stem(scoring);
    Weight = Weight .* scoring;    
    Weight(Weight<0) = 0.00001;
    % subplot(3,1,3); stem(Weight);
    % pause
    Weight = Weight/sum(Weight); % Normalization to sum of 1.
    assert(abs(1 - sum(Weight)) < 1e-3, 'sum(Weight) = %d', sum(Weight));
    
    %   2-4) Choose the best particle to update the pose
    [maxW, maxIdx] = max(Weight);
    myPose(:,j) = P(:,maxIdx);
    
    % 3) Resample if the effective number of particles is smaller than a threshold
    %   3-1) Calculate the effective number of particles
    %   Note: Normalization to weight has no impact upon the calculation of effective number.
    N_eff = floor(1./sum(Weight.*Weight));
    [W_sorted, sort_ids] = sort(Weight, 'descend');
    P_sorted = P(:, sort_ids);
        
    %   3-2) Resampling.    
    if(N_eff < 0.75*M) % To be confirmed
        fprintf(1,'j=%d; N_eff = %d; Wmax = %g;  Resmapling...\n',j, N_eff, maxW);
        % Weight(Weight<(1/M)) = 0;
        % Weight = Weight/sum(Weight);        
        % figure; plot(Weight);
        % W_sorted(1:10)'
        % sort_ids(1:10)'
        
        p_ids  = randsample(N_eff, M, 'true', W_sorted(1:N_eff));                       
        P      = P_sorted(:, p_ids');        
        Weight = (1.0/M)*ones(1,M);
        
        % figure; stem(p_ids);
        % 
        % pause
        
        
        % nCandidate   = 10000;
        % P_W_candidate  = zeros(4,nCandidate);
        % num_assigned = 0;
        % for m=1:1:M
        %     num = round(nCandidate * Weight(m));
        %     if num > 0
        %         P_W_candidate(1:3,num_assigned+1:num_assigned+num) = repmat(P(:,m),1,num);
        %         P_W_candidate(4,num_assigned+1:num_assigned+num) = repmat(Weight(m),1,num);
        %         num_assigned = num_assigned + num;            
        %     end    
        % end
        % 
        % P_W    = P_W_candidate(:,randi(num_assigned,1,M));
        % P      = P_W(1:3,:);
        % %Weight = P_W(4,:);
        % Weight = (1.0/M)*ones(1,M);
    end
    
    % 4) Visualize the pose on the map as needed
    %% figure; scatter(P(1,:),P(2,:)); hold on;
    %% plot(myPose(1,j),myPose(2,j),'rp', 'MarkerSize',20);
    % if mod(j,100)==0
    %     fprintf(1,'j=%d\n',j);
    %     
    %     figure;imagesc(map); hold on;
    %     colormap('gray'); axis equal; hold on;
    %     plot(myPose(1,:)*param.resol+param.origin(1), ...
    %         myPose(2,:)*param.resol+param.origin(2), 'g.-');
    %             
    %     figure; hist(Weight,20);
    %     pause
    % end        
end

end
