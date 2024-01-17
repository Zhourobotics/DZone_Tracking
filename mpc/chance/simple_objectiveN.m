function cost = simple_objective(z, p)
    global index pr model
    
    %% obtaining necessary information
    env_dim     =   model.stateDim;
    nRobot      =   model.nRobot;
    nTarget     =   model.nTarget;
    inputs      =   z(index.z.inputs);
    ego_pos     =   z(index.z.pos);         % current stage position [x, y, z]
    ego_pos     =   reshape(ego_pos, [env_dim, nRobot])'; % dim should be 3

    target_pos  =  p(index.p.target.pos);
    target_pos  =  reshape(target_pos, [env_dim, nTarget])';
    
    dist_norm = 0;
    for iRobot = 1:nRobot
        for jTarget = 1:nTarget
        diff = ego_pos(iRobot, :) - target_pos(jTarget, :);
        dist_norm = dist_norm + norm(diff) ;
        end
    end
%     control_efforts = norm(inputs);
    
    cost =  dist_norm;
end