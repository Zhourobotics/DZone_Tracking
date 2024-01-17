function cost = mpc_objectiveN_chance(z, p)

    global index pr model 
    
    %% retrieve necessary information
    env_dim     =   model.stateDim;
    nTarget     =   model.nTarget;
    ego_pos     =   z(index.z.pos);         % current stage position [x, y, z]
    ego_pos     =   reshape(ego_pos, [model.stateDim, model.nRobot])'; % dim should be 3
    inputs      =   z(index.z.inputs);
    inputs      =   reshape(inputs, [model.stateDim, model.nRobot])';
    weights     =   p(index.p.weights);
    target_pos         =  p(index.p.target.pos);
    target_pos_cov_vec =  p(index.p.target.posCov);
    target_pos_cov = [];
    for iTarget = 0 : nTarget-1
%         iTarget_cov = [target_pos_cov_vec(6*iTarget+1), target_pos_cov_vec(6*iTarget+4), target_pos_cov_vec(6*iTarget+6); ...
%             target_pos_cov_vec(6*iTarget+4), target_pos_cov_vec(6*iTarget+2), target_pos_cov_vec(6*iTarget+5); ...
%             target_pos_cov_vec(6*iTarget+6), target_pos_cov_vec(6*iTarget+5), target_pos_cov_vec(6*iTarget+3)];
        iTarget_cov = [target_pos_cov_vec(6*iTarget+1), 0, 0; ...
            0, target_pos_cov_vec(6*iTarget+2), 0; ...
            0, 0, 0];
        sz = size(target_pos_cov);
        target_pos_cov = [target_pos_cov, zeros(sz(1), env_dim); zeros(env_dim, sz(2)), iTarget_cov];
    end
                 
    %% target estimation trace
    %% propagation step of EKF
    % process_noise  =  diag(normrnd(zeros(nTarget*env_dim), model.target_process_noise_cov, [nTarget*env_dim, nTarget*env_dim]));
    target_pos     =  model.target_process_matrix * target_pos;
    prior_pos      =  reshape(target_pos, [model.stateDim, model.nTarget])';
    prior_cov      =  model.target_process_matrix * target_pos_cov * model.target_process_matrix' + model.target_process_noise_cov;
    prior_cov_inv  =  inv(prior_cov);
    
    total_trace = 0;
    rot_matrix = [0, 1, 0; -1, 0, 0; 0, 0, 1];
%     for iRobot = 1 : model.nRobot
%         for jTarget = 1 : model.nTarget
%             q = ego_pos(iRobot, :) - prior_pos(jTarget, :);
%             q = q'; % transformed to col vector
%             q_norm = norm(q);  % norm 
%             %%% This is actually the inverse
%             R_range   = model.range_peak * exp(-model.range_shape*q_norm);
%             R_bearing = model.bearing_peak * exp(-model.bearing_shape*q_norm);
%            
%             meas_range = -q ./ q_norm;
%             meas_bearing = (rot_matrix * q) ./ (q_norm ^2);
%             H_ij = [meas_range, meas_bearing];
%             cov_inv_j = prior_cov_inv((jTarget-1)*env_dim+1: jTarget*env_dim, (jTarget-1)*env_dim+1: jTarget*env_dim);
%             %% get distance-dependent covariance
%             measure_noise_cov_inv = [R_range, 0; 0, R_bearing];
%             post_cov_inv = cov_inv_j + H_ij * measure_noise_cov_inv * H_ij';
%             post_cov = inv(post_cov_inv);
%             total_trace = total_trace + trace(post_cov);
%         end
%     end
    
    H = [];
    R = [];
    for jTarget = 1 : model.nTarget
        H_j = [];
        for iRobot = 1 : model.nRobot
            q = ego_pos(iRobot, :) - prior_pos(jTarget, :);
            q = q'; % transformed to col vector
            q_norm = norm(q);  % norm 
            %%% This is actually the inverse
            R_range   = model.range_peak * exp(-model.range_shape*q_norm);
            R_bearing = model.bearing_peak * exp(-model.bearing_shape*q_norm);
            R = [R; R_range; R_bearing];
            
            meas_range = -q ./ q_norm;
            meas_bearing = (rot_matrix * q) ./ (q_norm ^2);
            H_j = [H_j, meas_range, meas_bearing];
        end
        sz = size(H);
        H = [H, zeros(sz(1), 2*model.nRobot); zeros(3, sz(2)), H_j];
    end

    R = diag(R);
    post_cov_inv = prior_cov_inv + H * R * H';
    post_cov = inv(post_cov_inv);
    total_trace = trace(post_cov);
    
    %% control input cost
    control_effort = 0;
    for iRobot = 1:model.nRobot
        control_effort = control_effort + inputs(iRobot, :) * inputs(iRobot, :)';
    end
    
    %% total cost
%     cost = weights(1) * total_trace + weights(2) * control_effort;
    cost = weights(1) * total_trace;

%     cost = (1/model.nRobot) * total_trace;
    
end
