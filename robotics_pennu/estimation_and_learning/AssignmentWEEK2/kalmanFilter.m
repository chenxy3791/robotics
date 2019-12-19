function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0]';
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    dt = t - previous_t;
    A = [[1 0 dt 0];...
         [0 1 0 dt];...
         [0 0 1 0 ];...
         [0 0 0 1 ]];
    C = [[1 0 0 0 ];...
         [0 1 0 0 ]];     

    varX = 1e-2;
    varY = 1e-2;
    varVx= 1e-2;
    varVy= 1e-2;
    
    Q = diag([varX, varY, varVx, varVy]);
    
    varZx= 1e-2;
    varZy= 1e-2;
    R = diag([varZx, varZy]);

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    %% vx = (x - state(1)) / (t - previous_t);
    %% vy = (y - state(2)) / (t - previous_t);
    %% % Predict 330ms into the future
    %% predictx = x + vx * 0.330;
    %% predicty = y + vy * 0.330;
    %% % State is a four dimensional element
    %% state = [x, y, vx, vy];
    
    % Time Update("Update")
    S_prior = A * state;
    P_prior = A * param.P * A' + Q;
    
    % Measurement Update("Correction")
    % (1)Compute the Kalman gain
    K  = P_prior*C'*inv(C*P_prior*C' + R);
    % (2)Update state estimation with measurement Zk
    Zk = [x;y];
    S_post = S_prior + K*(Zk - C*S_prior);
    % (3)Update the state error covariance
    P_post = (eye(4) - K*C)*P_prior;
    
    state   = S_post;
    param.P = P_post;
    predictx= x + state(3)*10*dt; % Assuming uniform time interval.
    predicty= y + state(4)*10*dt; % Assuming uniform time interval.
    
end
