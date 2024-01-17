
function cost = mpc_objective_chance(z, p)

%     cost = 0;

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


    %% control input cost
    control_effort = 0;
    for iRobot = 1:model.nRobot
%         control_effort = control_effort + inputs(iRobot, :) * inputs(iRobot, :)';
        control_effort = control_effort + inputs(iRobot, 1:2) * inputs(iRobot, 1:2)';
    end
    
    cost = weights(2) * control_effort;
    
end
