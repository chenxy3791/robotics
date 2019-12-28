% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% the number of grids for 1 meter.
myResol  = param.resol;
% the initial map size in pixels
myMap    = zeros(param.size); % Used to holding log_odd.
% the origin of the map in pixels
myorigin = param.origin; 

% 4. Log-odd parameters 
lo_occ   = param.lo_occ;  % Refer to 3-2-2_-Log-Odd-Update.pdf p10~p12: Constant measurement model
lo_free  = param.lo_free; % Refer to 3-2-2_-Log-Odd-Update.pdf p10~p12: Constant measurement model
lo_max   = param.lo_max;
lo_min   = param.lo_min;

N = size(pose,2); % Number of measurements

for j = 1:N % for each time,
    %%if(mod(j,100)==0) 
    %%    fprintf(1,'j = %d\n',j);
    %%end
    % Find grids hit by the rays (in the grid map coordinate)
    for k = 1:size(ranges,1)  
        angle      = scanAngles(k) + pose(3,j);        
        x_hit(k)   =  ranges(k,j)*cos(angle) + pose(1,j);
        y_hit(k)   = -ranges(k,j)*sin(angle) + pose(2,j);
        i_x_hit(k) = ceil(x_hit(k)*myResol) + myorigin(1); % param.resol represents the number of grids for 1 meter 
        i_y_hit(k) = ceil(y_hit(k)*myResol) + myorigin(2); % param.resol represents the number of grids for 1 meter 
    end
    
    % Find occupied-measurement cells and free-measurement cells
    ocu_cells = sub2ind(size(myMap),i_y_hit,i_x_hit);
    ocu_cells = unique(ocu_cells);
    
    free_cells = [];
    for k = 1:size(ranges,1)  
        orig = [ceil(pose(1,j)*myResol)+myorigin(1),ceil(pose(2,j)*myResol)+myorigin(2)]; % start point
        occ  = [i_x_hit(k),i_y_hit(k)]; % end point 
        % get cells in between
        [freex, freey] = bresenham(orig(1),orig(2),occ(1),occ(2));  
        % convert to 1d index
        free = sub2ind(size(myMap),freey,freex);
        free_cells = union(free_cells, free);
    end
    free_cells = unique(free_cells);
    free_cells;

    % Update the log-odds
    % -- For occupied-measurment cells
    for k = 1:length(ocu_cells)
        myMap(ocu_cells(k))  = myMap(ocu_cells(k)) + lo_occ;
    end    
    % -- For free-measurment cells
    for k = 1:length(free_cells)
        myMap(free_cells(k)) = myMap(free_cells(k)) - lo_free;
    end    

    % Saturate the log-odd values
    myMap(myMap>lo_max) = lo_max;
    myMap(myMap<lo_min) = lo_min;

    % Visualize the map as needed
    % figure;
    % imagesc(myMap);

end

end

