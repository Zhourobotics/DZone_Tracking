% Script for simulating multi-quad collision avoidance using mpc

%% Clean workspace
clear
close all
clearvars
clearvars -global
clc

%% load initial configurations
global config
configFile = 'config.json'; 
configHandler = fopen(configFile);
raw = fread(configHandler, +inf);
str = char(raw'); % Transformation
fclose(configHandler);
config = jsondecode(str);

%% Initialization
initialize;
exp = config.exp;
if config.exp == 0
%     con = Controller(cfg);
    con = Controller_new(cfg);
else
    con = Controller_exp(cfg);
end
% TODO: initialize ROS %%
con.initializeROS();
pause(1);

if exp
    iterations = 300;
else
    iterations = 60;
end


%% initial states of targets
targetStartPos = config.targetStartPos;
targetStartVel = config.targetStartVel;
if cfg.modeSim == 1
    for jTarget  =  1:model.nTarget
        Target(jTarget) = CTarget(jTarget, cfg);
        Target(jTarget).initializeROS();
        Target(jTarget).initializeState(targetStartPos(:, jTarget), targetStartVel(:, jTarget), ...
                                     cfg.target.noise.pos, cfg.target.noise.vel);
       %% initialize the estimation for better performance
%         dpos = zeros(3, 1);
%         for i = 1 : 3
%             dpos(i) = random('Normal', 0, ...
%                  sqrt(cfg.target.noise.pos(i,i)));
%         end
%         for iStage = 1:con.N_
%             con.target_est_(:, iStage, jTarget) = Target(jTarget).pos_real_(:, 1) + dpos;
%         end
    end
end

%% Initialization quad simulated initial state and mpc plan
if config.exp
    con.getEstimatedSystemState();
%     pause(1);
    
    quadStartPos = con.pos_real_;
else
    quadStartPos = config.quadStartPos;
end
quadStartVel = config.quadStartVel;
disp("Starting pos");
disp(quadStartPos);

con.weights_ = config.weights';
for iQuad = 1 : model.nRobot
    % initial state
    con.pos_real_((iQuad-1)*3+1 : (iQuad-1)*3+3)   = quadStartPos(1:3, iQuad);
    con.vel_real_((iQuad-1)*3+1 : (iQuad-1)*3+3)   = quadStartVel(1:3, iQuad);
end


iter = 0;


pathX = [quadStartPos(1, :)];
pathY = [quadStartPos(2, :)];
pathTargetX = [];
pathTargetY = [];
estTargetX = [];
estTargetY = [];

exitflags = [0]; % 1 for wrong, 0 for correct
trace_hist = [];
typeII_hist = [];
typeI_hist = [];
mpc_exitflag = [];
drone_flags = [];
type = [];

while(iter<iterations)
    % record the time at the start of loop
    t_loop_start  =  tic;
    % loop count
    iter = iter + 1;
    iter
    %% publish target path in simulation mode
    if ~config.exp
        if cfg.modeSim == 1     % if in simulation
            for jTarget = 1 : model.nTarget
                if jTarget == 3
                    Target(jTarget).circleMotion();
                else
                    Target(jTarget).propagatePathConstantV();
                end
%                 Target(jTarget).propagatePathConstantV();
                Target(jTarget).sendPath();
            end
        end
    end
    
    % ===== Get estimated state of the ego quad =====
    con.getEstimatedSystemState();
    disp("exp mode");
    disp(config.exp);
    if config.exp
        if any(con.msg_empty == 1)
%             disp("while");
             pause(0.1);
            continue;
        end
    else
        disp("out")
        
    end
    
    con.getTargetPath();

    trace = con.compute_trace_value();
    if ~exp
        drone_flags = con.if_in_zone_sensor();
    end
    con.ekf();
%     con.visualize("objective");
%     con.target_est_
    con.setOnlineParameters();
    
    %% control all quads
    con.solveMPC();
    %%%%%%%%%%%%%%%%
%     con.solveMPC_input(9);
    %%%%%%%%%%%%
%     if all(drone_flags==0)
%         con.solveMPC_input(2);
%     else 
%         con.solveMPC_input(1);
%     end
    %%%%%%%%%%%%%

%     if all (drone_flags == 0)
% %         con.solveMPC_input(3);
% %         a = 3;
%         a = 8;
%     elseif drone_flags(2,1) == 1
% %         con.solveMPC_input(4);
%         a = 4;
%     elseif drone_flags(2,1) == 2 && drone_flags(1,1) == 0
% %         con.solveMPC_input(7);
%         a = 7;
%     elseif drone_flags(1,1) == 1
% %         con.solveMPC_input(5);
%         a = 5;
%     else 
% %         con.solveMPC_input(6);
% %         a = 6;
%         con.weights_ = [4, 0.0015]';
%         a = 9;
%     end
%     con.solveMPC_input(a);
%     type = [type; a];
    
%     mpc_exitflag = [mpc_exitflag; con.mpc_exitflag_];
    con.step();

    if model.nTypeI > 0   
        typeI_value = con.typeI_zone_value(con.pos_real_', iter);
        typeI_hist = [typeI_hist; typeI_value];
    end
    
    if model.nTypeII > 0
        typeII_value = con.typeII_zone_value(con.pos_real_');
        typeII_hist = [typeII_hist; typeII_value];
    end
    
%     if con.mpc_exitflag_ == -6
%         exitflags = [exitflags; 1];
%     else
%         exitflags = [exitflags; 0];
%     end
    exitflags = [exitflags; con.mpc_exitflag_];
    pathX = [pathX; con.pos_real_(1, :)];
    pathY = [pathY; con.pos_real_(2, :)];
    pathTargetXiter = [];
    pathTargetYiter = [];
    for jTarget = 1:model.nTarget
        pathTargetXiter = [pathTargetXiter, Target(jTarget).getCurrentPosX()];
        pathTargetYiter = [pathTargetYiter, Target(jTarget).getCurrentPosY()];
        target_loc_real = con.target_path_(:, :, jTarget);
%         pathTargetXiter = [pathTargetXiter, con.target_path_(:, :, jTarget)];
%         pathTargetYiter = [pathTargetYiter, Target(jTarget).getCurrentPosY()];
    end
    pathTargetX = [pathTargetX; pathTargetXiter];
    pathTargetY = [pathTargetY; pathTargetYiter];
    estTargetX = [estTargetX; reshape(con.target_est_(1, 1, :), 1, nTarget)];
    estTargetY = [estTargetY; reshape(con.target_est_(2, 1, :), 1, nTarget)];
    trace_hist = [trace_hist; trace];


   
    if config.exp
        pause(1);
    end

end
% 
disp(con.wrong_count);
%%%%%% write a, b, or c to a csv file
% save_config = "test.csv";
save_config = model.nRobot + "_" + model.nTarget + "_exp" + config.expID + ".csv";
writematrix(exitflags, "results2/exitflags_" + save_config);
writematrix(pathX, "results2/X_" + save_config);
writematrix(pathY, "results2/Y_" + save_config);
%%%%%%%
writematrix(pathTargetX, "results2/Target_X_" + save_config);
writematrix(pathTargetY, "results2/Target_Y_" + save_config);
writematrix(trace_hist, "results2/trace_hist_" + save_config);
if config.nTypeI > 0
    writematrix(typeI_hist, "results2/typeI_hist_" + save_config);
end
if config.nTypeII > 0
    writematrix(typeII_hist, "results2/typeII_hist_" + save_config);
end

plot_figures