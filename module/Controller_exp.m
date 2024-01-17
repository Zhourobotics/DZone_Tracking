classdef Controller_exp < handle
    properties
        %% id, configs for all robots
        id_       =  [];
        exp_id_   =  [];
        exp = 1;
        real_exp_id_ = [2, 3, 4]; % [2, 5]; % 
%         real_exp_id_ = [2];
        cfg_;
        nRobot_   =  0;
        nTarget_  =  0;
        nTypeI_   =  0;
        nTypeII_  =  0;
        flags     =  [];
        msg_empty =  [];
        
        %% mpc param
        N_         =  0;
        dt_        =  0;
        nvar_      =  0;
        npar_      =  0;
        index_;
        
        %% real state
        pos_real_       = [];      % position
        vel_real_       = [];      % velocity
        
        %% estimated state
        pos_est_        =  [];
        pos_est_cov_    =  [];
        vel_est_        =  [];
        vel_est_cov_    =  [];
        
        %% control input
        u_mpc_          = [];      % phi_c, theta_c, vz_c, psi_rate_c
        u_body_         = [];      % transformed
        
        %% information of targets
        target_path_    =  [];
        target_est_     =  [];
        target_est_cov_ =  [];
        
        %% mpc plan
        mpc_pAll_       = [];               % parameters fed into MPC
        mpc_exitflag_                       % MPC solving exitflag
        mpc_info_                           % MPC solving information
        mpc_Xk_         = [];               % store initial conditions for MPC
        mpc_Zk_         = [];               % store computed current stage from MPC
        old_mpc_Zk_     = [];
        mpc_Zk2_        = [];               % store computed next stage from MPC
        mpc_ZPlan_      = [];               % store entire planned MPC
        mpc_Path_       = [];               % store MPC planned path
        mpc_PathCov_    = [];               % store MPC planned path with
                                            % position covariance
        weights_;

        %% ROS subscriber/publisher
        pose_measure_sub_                   % subscribe to mocap raw pose data
        state_est_sub_                      % subscribe to estimated state
        target_path_sub_   = {};
        cmd_vel_pub_                        % publish computed control input
        cmd_pub_   = {};                      % publish computed control input
        state_sub_ = {};
        %%%%%  debug the exitflag = -6 bug
        wrong_count = 0;
        tftree = 0
        targetFrame = '';
        counter = 0;
        node;
    end
    
    methods
        
        function con = Controller_exp(cfg)
            %% constructor 
            
            global pr model index
            nRobot_       =   model.nRobot;
            con.id_       =   1:nRobot_;
            con.exp_id_   =   1:nRobot_;
            %% For real experiment
%             con.real_exp_id_ = [2, 3, 4, 5];

            con.cfg_      =   cfg;
            con.nRobot_   =   nRobot_;
            con.nTarget_  =   model.nTarget;
            con.nTypeI_   =   model.nTypeI;
            con.nTypeII_  =   model.nTypeII;
            con.flags     =   zeros(con.nRobot_, con.nTypeI_);
            
            %% real state
            con.pos_real_    =   zeros(3, nRobot_);
            con.vel_real_    =   zeros(3, nRobot_); 
            
            %% estimated state
            con.pos_est_        = zeros(3, nRobot_);
            con.pos_est_cov_    = repmat(eye(3), [1, nRobot_]);
            con.vel_est_        = zeros(3, nRobot_);
            con.vel_est_cov_    = repmat(eye(3), [1, nRobot_]);

            con.u_mpc_ = zeros(4*nRobot_, 1);
            con.u_body_ = zeros(4*nRobot_, 1);
            
            con.N_         =  model.N;
            con.dt_        =  model.dt;
            con.nvar_      =  model.nvar;
            con.npar_      =  model.npar;
            con.index_     =  index;
            
            % target state
            con.target_path_    =  zeros(3, con.N_, con.nTarget_);
            con.target_est_cov_ =  1*ones(6, con.N_, con.nTarget_);
%             con.target_est_cov_(3, :, :) = 0.0;
            con.target_est_cov_(3:6, :, :) = 0.0;
            con.target_est_     =  zeros(3, con.N_, con.nTarget_);
            
            con.weights_ = [1.0; 1.0];
            con.tftree = rostf;
            con.targetFrame = 'map';
            con.counter = 0;
            con.msg_empty = ones(length(con.real_exp_id_), 1);
        end
        
        %%  =======================================================================
        function initializeROS(con)
            % Initialize ROS publishers and subscribers for the quadrotors
            % predicted path of targets
            %%%%%%%%%%%% Assume what we get here is the ground truth
            for jTarget = 1 : con.nTarget_
                if ~con.exp
                    con.target_path_sub_{jTarget} = rossubscriber(...
                        ['/Target', num2str(jTarget), '/path_prediction'], ...
                        'std_msgs/Float64MultiArray');
                end
            end
            
            if ~con.exp
                for i = 1:length(con.exp_id_)
                    
                    quadExpID = con.exp_id_(i);
    
                    %% ROS subscribers
                    % mocap raw data
                    con.pose_measure_sub_ = rossubscriber(...
                        ['/Bebop', num2str(quadExpID), '/pose'], 'geometry_msgs/PoseStamped');
                    % bebop 2 estimator data
                    con.state_est_sub_ = rossubscriber(...
                        ['/Bebop', num2str(quadExpID), ...
                        '/position_velocity_orientation_estimation'], ...
                        'nav_msgs/Odometry');
    
                    %% ROS publisher
                    % to publish mpc control input
                    con.cmd_vel_pub_ = rospublisher(...
                        ['/bebop_auto_', num2str(quadExpID), '/cmd_vel'], ...
                        'geometry_msgs/Twist');
                end
            else
                for i =1:length(con.real_exp_id_)
%                 con.state_sub_{i} = rossubscriber(...
%                     ['/cf', num2str(con.real_exp_id_(i)), '/pose'], ...
%                     'geometry_msgs/PoseStamped');

                    con.state_sub_{i} = rossubscriber(...
                        ['/cf', num2str(con.real_exp_id_(i)), '/pose_log'], ...
                        'geometry_msgs/PoseStamped');
    
                    con.cmd_pub_{i} = rospublisher(...
                        ['/uav', num2str(con.real_exp_id_(i)), '/cmd_position'], ...
                        "geometry_msgs/PointStamped");
                end
            end
        end
        
        %%  =======================================================================
        function initializeMPC(con, x_start, mpc_plan)
            % Initialize the intial conditions for the MPC solver with
            % x_start and mpc_plan, only used when necessary
            state_sub1_;
            state_sub2_;
            con.mpc_Xk_    = x_start;
            con.mpc_ZPlan_ = mpc_plan;
          
        end

        %%  =======================================================================
        function getTargetPath(con)
            % Get predicted path of all moving obstacles
            % This function takes a long time
            
            for jTarget = 1 : con.nTarget_
                if ~con.exp
                    msg = con.target_path_sub_{jTarget}.LatestMessage;
                    if ~isempty(msg)        % read only if the msg is not empty
                        con.target_path_(:, :, jTarget) = reshape(msg.Data, 3, con.N_);
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                     con.target_est_(:, :, jTarget)  = reshape(msg.Data, 3, con.N_);
                    else
                        warning('Target %i path is not obtained! \n', jTarget);
                    end
                else
                    for jTarget = 1 : con.nTarget_
                        sourceFrame = append('robot', num2str(jTarget), '/base_footprint');
%                         disp("Starting to wait")
%                         con.tftree
%                         con.targetFrame
%                         sourceFrame
                        waitForTransform(con.tftree, con.targetFrame, sourceFrame);
%                         disp("Wait finished")
                        msg = getTransform(con.tftree, con.targetFrame, sourceFrame);
        %                 msg = con.target_path_sub_{jTarget}.LatestMessage;
                        if ~isempty(msg)        % read only if the msg is not empty
        %                     con.target_path_(:, :, jTarget) = reshape(msg.Data, 3, con.N_);
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                     con.target_est_(:, :, jTarget)  = reshape(msg.Data, 3, con.N_);
        %                     p = msg.Pose.Pose.Position;
                            p = msg.Transform.Translation;
                            new_pos = repmat([p.X; p.Y; 1.2], 1, con.N_);
                            con.target_path_(:, :, jTarget) = new_pos;
%                             disp("dfffffffdddddddddd");
%                             disp(con.target_path_);
                        else
                            con.target_path_ = con.target_path_;
                            disp(con.target_path_);
                            warning("target position not available");
                            % disp(con.target_path_);
                        end
                    end           
                end           
            end
        end
        
        function setOnlineParameters(con)
            envDim        = con.cfg_.ws;
            targetPath    = con.target_est_;
            targetPathCov = con.target_est_cov_;

            %% all stage parameters
            pStage = zeros(con.npar_, 1);               % some stage
            con.mpc_pAll_= repmat(pStage, [con.N_, 1]); % all stages
            for iStage = 1 : con.N_
                % general parameter
                pStage(con.index_.p.envDim)   = envDim;   	 % environment dimension
                pStage(con.index_.p.weights)  = con.weights_;
                for iTarget = 1 : con.nTarget_
                    pStage(con.index_.p.targetParam(1:3, iTarget)) = ...
                        targetPath(:, iStage, iTarget);
                    pStage(con.index_.p.targetParam(4:9, iTarget)) = ...
                        targetPathCov(:, iStage, iTarget);
                end
                % insert into the all stage parameter
                con.mpc_pAll_((1 + con.npar_*(iStage-1)) : ...
                    (con.npar_ + con.npar_*(iStage-1))) = pStage;
            end
            
        end
        
        function solveMPC(con)
            
            global model config
            
            problem.all_parameters = con.mpc_pAll_;

            % set initial conditions
            con.mpc_Xk_ = [reshape(con.pos_est_, [con.nRobot_*3, 1])];
            problem.xinit = con.mpc_Xk_;

           % prepare initial guess
            if con.mpc_exitflag_ == 1
                x0_temp = [con.mpc_ZPlan_(:, 2); con.mpc_ZPlan_(:, 2)];

            else
                x0_temp_stage  =  zeros(con.nvar_, 1);
                x0_temp_stage([con.index_.z.pos]) = con.mpc_Xk_;
                x0_temp = repmat(x0_temp_stage, con.N_, 1);
            end

             problem.x0  =  x0_temp;
             
%               obj_name  = strcat('FORCESNLPsolver_zone1_zone2_', ...
%                   num2str(con.nRobot_), '_', ...
%                   num2str(con.nTarget_), '_', num2str(con.N_), '_', num2str(1000*con.dt_), '_dynamics');
%               NLPSolver_objective = str2func(obj_name);
%               [objV, obj_jacob] = NLPSolver_objective(problem.x0(1:27), problem.all_parameters(1:33), 1);
%               assert(any(~isnan(objV) & ~isinf(objV), 'all'), ['NaN or INF in value']);
%               assert(any(~isnan(obj_jacob) & ~isinf(obj_jacob), 'all'), ['NaN or INF in jacob']);
             
%             con.visualize("objective");

%             func_name = 'FORCESNLPsolver_test
%             func_name = strcat('FORCESNLPsolver_zone2_', ...
%                 num2str(con.nRobot_), '_', ...
%                 num2str(con.nTarget_), '_exp700');
            func_name = strcat('FORCESNLPsolver_zone2_', ...
                num2str(con.nRobot_), '_', ...
                num2str(con.nTarget_), '_exp', ...
                num2str(config.expID));
            NLPSolver = str2func(func_name);
            
            [output, exitflag, info] = NLPSolver(problem); 
          
            % store mpc sovling information
            con.mpc_exitflag_  =  exitflag;
            con.mpc_info_      =  info;

            for iStage = 1 : con.N_
                con.mpc_ZPlan_(:, iStage) = output.(['x', sprintf('%d', iStage)]);
                con.mpc_Path_(:, iStage)  = con.mpc_ZPlan_(con.index_.z.pos, iStage);
            end
            %%% make a copy of previous solution 
            con.old_mpc_Zk_ = con.mpc_Zk_;
            if all(size(con.old_mpc_Zk_) == [0 0])
                con.old_mpc_Zk_ = con.mpc_ZPlan_(:, 1);
            end  
            con.mpc_Zk_  = con.mpc_ZPlan_(:, 1);
%             con.mpc_Zk2_ = obj.mpc_ZPlan_(:, 2);

            % check the exitflag and get optimal control input
           if exitflag == 0                 % solver reaching maximum iterations
               warning('MPC: Max iterations reached!');
           elseif exitflag == -4
               warning('MPC: Wrong number of inequalities input to solver!');
           elseif exitflag == -5
               warning('MPC: Error occured during matrix factorization!');
           elseif exitflag == -6
               con.wrong_count = con.wrong_count + 1;
               warning('MPC: NaN or INF occured during functions evaluations!');
           elseif exitflag == -7
%                con.wrong_count = con.wrong_count + 1;
               warning('MPC: Infeasible! The solver could not proceed!');
           elseif exitflag == -10
               warning('MPC: NaN or INF occured during evaluation of functions and derivatives!');
           elseif exitflag == -11
               warning('MPC: Invalid values in problem parameters!');
           elseif exitflag == -100
               warning('MPC: License error!');
           end
          
           con.u_mpc_ = con.mpc_Zk_(con.index_.z.inputs);
           if exitflag == 1 || exitflag == 0
               % do nothing 
%            else
%                v1 = con.old_mpc_Zk_(con.index_.z.inputs);
%                if model.nTypeI > 0
%                    zone_pos = repmat(model.typeI_mu, model.nRobot, 1);
%                elseif model.nTypeII > 0
%                    zone_pos = repmat(model.typeII_mu, model.nRobot, 1);
%                end
%                v2 = con.mpc_Xk_ - zone_pos;
%                v1_ = reshape(v1, [3, model.nRobot]);
%                v2_ = reshape(v2, [3, model.nRobot]);
%                v1_ = v1_(1:2, :); %% throw away z axis
%                v2_ = v2_(1:2, :);
%                v1_norm = vecnorm(v1_, 2, 1);
%                v2_norm = vecnorm(v2_, 2, 1);
%                v = 10 * (v1_ ./ v1_norm) + (v2_ ./ v2_norm);
%                v = 0.01 * v;
%                v = vertcat(v, zeros(1, model.nRobot));
%                v = reshape(v, [1, 3 * model.nRobot]);
%                con.mpc_Zk_(con.index_.z.pos) = con.mpc_Xk_;
%                con.mpc_Zk_(con.index_.z.inputs) = v;
%                con.mpc_Zk_(con.index_.z.inputs) = 0;
           elseif (model.nTypeII > 0 && con.target_path_(1, 1, 1) > 0)
               v1 = con.old_mpc_Zk_(con.index_.z.inputs);
                   zone_pos = repmat(model.typeII_mu, model.nRobot, 1);
               
                   v2 = con.mpc_Xk_ - zone_pos;
                   v1_ = reshape(v1, [3, model.nRobot]);
%                    size(con.mpc_Xk_)
%                    size(zone_pos)
                   v2_trunc = v1*0;
                   for iRobot = 1:model.nRobot
                       new_vec = v2(3*(iRobot-1)+1:3*(iRobot-1)+2);
                       norm = vecnorm(v2(3*(iRobot-1)+1:3*(iRobot-1)+2), 2, 1);
                       [min_val, min_loc] = min(norm);
                       v2_trunc(3*(iRobot-1)+1:3*(iRobot-1)+3) = v2(3*(iRobot-1)+1:3*(iRobot-1)+3, min_loc);
                   end
                        
    
%                    v2_ = reshape(v2_trunc, [3, model.nRobot]);
                   v2_ = reshape(v2, [3, model.nRobot]);
                   v1_ = v1_(1:2, :); %% throw away z axis
                   v2_ = v2_(1:2, :);
                   v1_norm = vecnorm(v1_, 2, 1);
                   v2_norm = vecnorm(v2_, 2, 1);
                   v = 10 * (v1_ ./ v1_norm) +  10*(v2_ ./ v2_norm);
                   v = 0.01 * v;
                   v = vertcat(v, zeros(1, model.nRobot));
                   v = reshape(v, [1, 3 * model.nRobot]);
                   
                   con.mpc_Zk_(con.index_.z.pos(1:6)) = con.mpc_Xk_(1:6);
                   con.mpc_Zk_(con.index_.z.inputs(1:6)) = v(1:6);
                   disp("con.mpc_Zk_")
                   disp(con.mpc_Zk_)
           end
        end
        
        function solveMPC_input(con, id)
            
            global model config
            
            problem.all_parameters = con.mpc_pAll_;

            % set initial conditions
            con.mpc_Xk_ = [reshape(con.pos_est_, [con.nRobot_*3, 1])];
            problem.xinit = con.mpc_Xk_;

           % prepare initial guess
            if con.mpc_exitflag_ == 1
                x0_temp = [con.mpc_ZPlan_(:, 2); con.mpc_ZPlan_(:, 2)];

            else
                x0_temp_stage  =  zeros(con.nvar_, 1);
                x0_temp_stage([con.index_.z.pos]) = con.mpc_Xk_;
                x0_temp = repmat(x0_temp_stage, con.N_, 1);
            end

             problem.x0  =  x0_temp;
             
%               obj_name  = strcat('FORCESNLPsolver_zone1_zone2_', ...
%                   num2str(con.nRobot_), '_', ...
%                   num2str(con.nTarget_), '_', num2str(con.N_), '_', num2str(1000*con.dt_), '_dynamics');
%               NLPSolver_objective = str2func(obj_name);
%               [objV, obj_jacob] = NLPSolver_objective(problem.x0(1:27), problem.all_parameters(1:33), 1);
%               assert(any(~isnan(objV) & ~isinf(objV), 'all'), ['NaN or INF in value']);
%               assert(any(~isnan(obj_jacob) & ~isinf(obj_jacob), 'all'), ['NaN or INF in jacob']);
             
%             con.visualize("objective");

%             func_name = 'FORCESNLPsolver_test
%             func_name = strcat('FORCESNLPsolver_zone2_', ...
%                 num2str(con.nRobot_), '_', ...
%                 num2str(con.nTarget_), '_exp700');
            func_name = strcat('FORCESNLPsolver_zone2_', ...
                num2str(con.nRobot_), '_', ...
                num2str(con.nTarget_), '_exp', ...
                num2str(id));
            NLPSolver = str2func(func_name);
            
            [output, exitflag, info] = NLPSolver(problem); 
          
            % store mpc sovling information
            con.mpc_exitflag_  =  exitflag;
            con.mpc_info_      =  info;

            for iStage = 1 : con.N_
                con.mpc_ZPlan_(:, iStage) = output.(['x', sprintf('%d', iStage)]);
                con.mpc_Path_(:, iStage)  = con.mpc_ZPlan_(con.index_.z.pos, iStage);
            end

            con.mpc_Zk_  = con.mpc_ZPlan_(:, 1);
%             con.mpc_Zk2_ = obj.mpc_ZPlan_(:, 2);

            % check the exitflag and get optimal control input
           if exitflag == 0                 % solver reaching maximum iterations
               warning('MPC: Max iterations reached!');
           elseif exitflag == -4
               warning('MPC: Wrong number of inequalities input to solver!');
           elseif exitflag == -5
               warning('MPC: Error occured during matrix factorization!');
           elseif exitflag == -6
               con.wrong_count = con.wrong_count + 1;
               warning('MPC: NaN or INF occured during functions evaluations!');
           elseif exitflag == -7
%                con.wrong_count = con.wrong_count + 1;
               warning('MPC: Infeasible! The solver could not proceed!');
           elseif exitflag == -10
               warning('MPC: NaN or INF occured during evaluation of functions and derivatives!');
           elseif exitflag == -11
               warning('MPC: Invalid values in problem parameters!');
           elseif exitflag == -100
               warning('MPC: License error!');
           end
          
           con.u_mpc_ = con.mpc_Zk_(con.index_.z.inputs);
           if exitflag == 1
               % do nothing 
           else
               con.mpc_Zk_(con.index_.z.inputs) = 0;
           end
        end
        
        function getEstimatedSystemState(con)
            if con.exp
                for i = 1:length(con.real_exp_id_)
                    msg = con.state_sub_{i}.LatestMessage;
%                     
%                       disp("ffff");
%                       disp(msg);
%                       disp(con.real_exp_id_(i))
%                     while isempty(msg)
%                         pause(0.001)
%                     end
                    if ~isempty(msg)
                        p = msg.Pose.Position;
                        new_pos = [p.X; p.Y; 1.2];
                        con.pos_real_(:, i) = new_pos;
%                         disp("in controller pos")
                        disp(new_pos)
                        con.msg_empty(i) = 0;
                    else
                        disp("empty message!!!!!")
                        con.msg_empty(i) = 1;
                    end
                end
            con.pos_est_    =   con.pos_real_;
            con.vel_est_    =   con.vel_real_;
            end    
        end
        
        function step(con)
            switch con.cfg_.modeSim
                case 1                  
                    xNext = first_order_dynamics(con.mpc_Zk_);
                    con.pos_real_ = xNext(con.index_.x.pos);
%                     con.vel_real_ = xNext(con.index_.x.vel);                   
                    con.pos_real_ = reshape(con.pos_real_, [3, con.nRobot_]);
%                     con.vel_real_ = reshape(con.vel_real_, [3, con.nRobot_]);
                    if con.exp
%                         disp("in experiment");
                        for i = 1:con.nRobot_
                            msg = rosmessage("geometry_msgs/PointStamped");
                            msg.Point.X = con.pos_real_(1, i);
                            msg.Point.Y = con.pos_real_(2, i);
                            msg.Point.Z = 1.2;
                            msg.Header.Stamp = rostime("now");
%                             disp(msg.Point);
%                             disp("sending!!!!!");
                            con.counter = con.counter + 1;
                            %disp(con.cmd_pub_{i});
%                             if ~mod(con.counter, 8)
%                                 send(con.cmd_pub_{i}, msg);
%                             end
                            send(con.cmd_pub_{i}, msg);

                        end
                    end
                otherwise
                    % TODO
            end
                    
        end
        
        function ekf(con)
            
            global model 
            
            A = eye(2*con.nTarget_); %% process matrix
            Q = 0.05 * eye(2*con.nTarget_);  %% process noise covariance
            robot_pos = con.pos_real_(1:2, :)';  %% size: (nRobot, 2)
            x         = reshape(con.target_est_(1:2, 1, :), 2, con.nTarget_);
            x = x'; %% size: (nTarget, 2)
            x_real    = reshape(con.target_path_(1:2, 1, :), 2, con.nTarget_); 
            x_real = x_real'; %% real state, (nTarget, 2)
            P_last    = reshape(con.target_est_cov_([1, 2, 4], 1, :), 3, con.nTarget_);
            P_last = P_last';  %% size: (nTarget, 3)
            
            P = zeros(con.nTarget_*2, con.nTarget_*2);
            for jTarget = 1 : con.nTarget_
%                 jTarget_cov = [P_last(jTarget, 1), P_last(jTarget, 3); 
%                                P_last(jTarget, 3), P_last(jTarget, 2)];
                jTarget_cov = [P_last(jTarget, 1), 0; 0, P_last(jTarget, 2)];
                P((jTarget-1)*2+1:jTarget*2, (jTarget-1)*2+1:jTarget*2) = jTarget_cov;
            end
            
            %% prediction step
            x_ = A * reshape(x', [con.nTarget_*2, 1]);
            x_ = reshape(x_, [2, con.nTarget_])';
            P_ = A * P * A' + Q;
            
            %% update step
            rot_matrix = [0, 1; -1, 0];
            %%% first 2 means 1 range + 1 bearing sensor, second 2 is dim
            H = zeros(con.nRobot_ * con.nTarget_ * 2, con.nTarget_ * 2); 
            meas_error = []; %% measurement error
            R = [];
            for jTarget = 1 : con.nTarget_
                for iRobot = 1 : con.nRobot_
                    %%% measurement matrix
                    q = robot_pos(iRobot, :) - x_(jTarget, :);
                    q = q'; % transformed to col vector
                    q_norm = norm(q);  % norm 
                    
                    h_range = -q ./ q_norm;  %% column vector
                    h_bearing = (rot_matrix * q) ./ (q_norm ^2); %% column vector
                    offset = (jTarget-1)*2*con.nRobot_;
                    H(offset+(iRobot-1)*2+1, (jTarget-1)*2+1:jTarget*2) = h_range';
                    H(offset+(iRobot-1)*2+2, (jTarget-1)*2+1:jTarget*2) = h_bearing';
                    
                    %%% simulate the predicted and true measurements
                    diff_real   = x_real(jTarget, :) - robot_pos(iRobot, :);
                    d_real      = norm(diff_real);
                    theta_real  = atan2(diff_real(2), diff_real(1));
                    
                    %%% this is actually the inverse!!
                    R_real_range   = model.range_peak * exp(-model.range_shape*d_real);
                    R_real_bearing = model.bearing_peak * exp(-model.bearing_shape*d_real);
%                     R_real_range = 500/(d_real^2);
%                     R_real_bearing = 500/(d_real^2);
                    
                    d_real_noise = normrnd(0, 1/R_real_range);
                    theta_real_noise = normrnd(0, 1/R_real_bearing);
                    d_real = d_real + d_real_noise;
                    theta_real = theta_real + theta_real_noise;
                    
                    diff_est    = x_(jTarget, :) - robot_pos(iRobot, :);
                    d_est       = norm(diff_est);
                    theta_est   = atan2(diff_est(2), diff_est(1));
                     
%                     R_est_range = model.range_peak * exp(-model.range_shape*d_est);
%                     R_est_bearing = model.bearing_peak * exp(-model.bearing_shape*d_est);                       
%                     d_est_noise = normrnd(0, R_est_range);
%                     theta_est_noise = normrnd(0, R_est_bearing);
%                     d_est = d_est +  d_est_noise;
%                     theat_est_bearing = theta_est + theta_est_noise;

                    %%%%%%%%%% exact measurement error %%%%%%%%%%%%%%
%                     d_error = d_real - d_est;
%                     theta_error = theta_real - theta_est;
%                     meas_error = [meas_error; d_error; theta_error];
                    %%%% linearization of the measurement error %%%%%%
                    x_tilde = (x_real(jTarget, :) - x_(jTarget, :))';
                    err = [h_range'; h_bearing'] * x_tilde + [d_real_noise; theta_real_noise];
                    meas_error = [meas_error; err];
                    R =  [R; 1/R_real_range; 1/R_real_bearing];
                end
            end
            R = diag(R);
            S = H * P_ * H' + R;
            K = P_ * H' * inv(S);
            x_updated = reshape(x_', con.nTarget_*2, 1) + K * meas_error;
            P_updated = P_ - K * S * K';
            x_updated = reshape(x_updated, [2, con.nTarget_]);
%             disp(x_updated);
            con.target_est_  =  zeros(3, con.N_, con.nTarget_);
            for i = 1:con.N_
                con.target_est_(1:2, i, :) = x_updated;
                con.target_est_(3, i, :) = 1.2;
            end
            con.target_est_cov_  =  zeros(6, con.N_, con.nTarget_);
            for jTarget = 1 : con.nTarget_
               con.target_est_cov_(1, :, jTarget) = P_updated((jTarget-1)*2+1, (jTarget-1)*2+1);
               con.target_est_cov_(2, :, jTarget) = P_updated((jTarget-1)*2+2, (jTarget-1)*2+2);
%                con.target_est_cov_(4, :, jTarget) = P_updated((jTarget-1)*2+1, (jTarget-1)*2+2);
            end
            
            diff = con.target_est_(1:2, 1, :) - con.target_path_(1:2, 1, :);
            diff = reshape(diff, 2, con.nTarget_);
            error = vecnorm(diff, 2, 1);
            mean_error = mean(error);
%             disp(mean_error);
% %             disp(diff);
        end
        
        function visualize(con, mode)
            if mode == "objective"
                xx =-10:0.5:10;
                yy =10:-0.5:-10;
                obj_values = [];
                for i = 1:40
                    for j = 1:40
                        ego_pos = [xx(i), yy(j), 1.2; xx(i), yy(i), 1.2];
                        target_pos_cov_vec = reshape(con.target_est_cov_(:, 1, :), 6*con.nTarget_, 1);
                        val = con.objective_value(ego_pos, reshape(con.target_est_(:, 1, :), 3*con.nTarget_, 1), target_pos_cov_vec);
                        obj_values = [obj_values; val];
                    end
                end
                obj_values = reshape(obj_values, 40, 40);
                h = heatmap(obj_values);
                c1  = [0, 1, 1];
                c2  = [0, 0, 1];
                length = 20;
                colors_p = [linspace(c1(1),c2(1),length)', linspace(c1(2),c2(2),length)', linspace(c1(3), c2(3),length)'];
                colormap(colors_p)
%                 colormap(hot(256));
                h.Units = "normalized";
                h.Position = [0.1, 0.05, 0.75, 0.9];
                h.XDisplayLabels = linspace(1, 40, 40);
                h.YDisplayLabels = linspace(1, 40, 40);
                saveas(gcf, "heatmap.jpg");
            end
        end
        
        function trace = compute_trace_value(con)
            robot_pos = con.pos_real_';
            target_pos = reshape(con.target_est_(:, 1, :), 3*con.nTarget_, 1);
            target_pos_cov_vec = reshape(con.target_est_cov_(:, 1, :), 6*con.nTarget_, 1);
            trace = con.objective_value(robot_pos, target_pos, target_pos_cov_vec);
        end
        
        function cost = objective_value(con, robot_pos, target_pos, target_pos_cov_vec)

            % define stage cost function for mpc

            global index pr model                       % global index information

            %% obtaining necessary information
            env_dim     =   model.stateDim;
            nTarget     =   model.nTarget;
            ego_pos     =   robot_pos;

            % Compute the trace at new position
            % structure of p: xyz of each target, cov of each target
            target_pos         =  reshape(target_pos, nTarget*3, 1);
            target_pos_cov = [];
            for iTarget = 0 : nTarget-1
%                 iTarget_copos_cov_vec(6*iTarget+4), target_pos_cov_vec(6*iTarget+2), target_pos_cov_vec(6*iTarget+5); ...
%                     target_v = [target_pos_cov_vec(6*iTarget+1), target_pos_cov_vec(6*iTarget+4), target_pos_cov_vec(6*iTarget+6); ...
%                     target_pos_cov_vec(6*iTarget+6), target_pos_cov_vec(6*iTarget+5), target_pos_cov_vec(6*iTarget+3)];
                iTarget_cov = [target_pos_cov_vec(6*iTarget+1), 0, 0;
                               0, target_pos_cov_vec(6*iTarget+2), 0; 
                               0, 0, 0];
                sz = size(target_pos_cov);
                target_pos_cov = [target_pos_cov, zeros(sz(1), env_dim); zeros(env_dim, sz(2)), iTarget_cov];
            end

            % propagation step of EKF
        %     process_noise  =  diag(normrnd(zeros(nTarget*env_dim), model.target_process_noise_cov, [nTarget*env_dim, nTarget*env_dim]));
            target_pos     =  model.target_process_matrix * target_pos;
            prior_pos      =  reshape(target_pos, [model.stateDim, model.nTarget])';
            prior_cov      =  model.target_process_matrix * target_pos_cov * model.target_process_matrix' + model.target_process_noise_cov;
            cov_inv  =  inv(prior_cov);
            rot_matrix = [0, 1, 0; -1, 0, 0; 0, 0, 1];
%             for jTarget = 1 : model.nTarget
%                 for iRobot = 1 : model.nRobot
%                     q = ego_pos(iRobot, :) - prior_pos(jTarget, :);
%                     q = q'; % transformed to col vector
%                     q_norm = norm(q);  % norm
%                     %%% This is actually the inverse
%                     R_range   = model.range_peak * exp(-model.range_shape*q_norm);
%                     R_bearing = model.bearing_peak * exp(-model.bearing_shape*q_norm);
% %                     R_range = 500/(q_norm^2);
% %                     R_bearing = 500/(q_norm^2);
% 
%                     meas_range = -q ./ q_norm;
%                     meas_bearing = (rot_matrix * q) ./ (q_norm ^ 2);
%                     H_ij = [meas_range, meas_bearing];
%                     cov_inv_j = cov_inv((jTarget-1)*env_dim+1: jTarget*env_dim, (jTarget-1)*env_dim+1: jTarget*env_dim);
%                     %% get distance-dependent covariance
%                     measure_noise_cov_inv = [R_range, 0; 0, R_bearing];
%         %             measure_noise_cov_inv = [0.01, 0; 0, 0.02];
%                     post_cov_inv = cov_inv_j + H_ij * measure_noise_cov_inv * H_ij';
% %                     cov_inv((jTarget-1)*env_dim+1: jTarget*env_dim, (jTarget-1)*env_dim+1: jTarget*env_dim) = post_cov_inv;
%                     post_cov = inv(post_cov_inv);
%                     total_trace = total_trace + trace(post_cov);
%                 end
%             end
%             cost = total_trace / model.nRobot;
            
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
            post_cov_inv = cov_inv + H * R * H';
            post_cov = inv(post_cov_inv);
            cost = trace(post_cov);
            
        end
   
        
        function value = typeII_zone_value(con, ego_pos)
            global model
            env_dim = 3;
            pos_typeII  =   reshape(model.typeII_mu, [model.stateDim, model.nTypeII])';
            cons_type_two = [];
            for iRobot = 1 : model.nRobot
               for iZone = 1 : model.nTypeII
                  ai = pos_typeII(iZone, :) - ego_pos(iRobot, :); % row vector
                  ai_normalized = ai / norm(ai);
                  erf_value = erfinv(2 * model.eps2 - 1);
                  sqr_term = 2 * ai_normalized * model.typeII_cov((iZone-1)*env_dim+1: iZone*env_dim, (iZone-1)*env_dim+1 : iZone*env_dim) * ai_normalized';
                  max_neighbor_dist = 1e-6;
                  for jRobot = 1 : model.nRobot
                      if jRobot == iRobot
                         neighbor_dist = 1e-6;
                      else
                         diff = ego_pos(iRobot, :) - ego_pos(jRobot, :);
                         neighbor_dist = sqrt(diff*diff');
                      end
                      max_neighbor_dist = max(max_neighbor_dist, neighbor_dist);
                  end
                  max_neighbor_dist = min(max_neighbor_dist, model.max_dist);
                  cons = ai_normalized * ai' - erf_value * sqrt(sqr_term) - model.delta(iZone) * max_neighbor_dist;
                  cons_type_two = [cons_type_two; cons];
               end
            end
            value = min(cons_type_two);
        end
        
        function value = typeI_zone_value(con, ego_pos, iter)
            global model
            env_dim = 3;
            pos_typeI  =   reshape(model.typeI_mu, [model.stateDim, model.nTypeI])';
            cons_type_one = [];
            cons_left = [];
            cons_right = [];
            cons_robot = [];
            for iRobot = 1 : model.nRobot
                for iZone = 1 : model.nTypeI
                    ai =  pos_typeI(iZone, :) - ego_pos(iRobot, :); % row vector
                    ai_normalized = ai / norm(ai);
                    erf_value = erfinv(1 - 2 * model.eps1(iRobot));
                    sqr_term = 2 * ai_normalized * model.typeI_cov((iZone-1)*env_dim+1: iZone*env_dim,  (iZone-1)*env_dim+1: iZone*env_dim) * ai_normalized';
                    cons  =  ai_normalized * ai' - erf_value * sqrt(sqr_term) - model.typeI_d(iZone);
                    cons_left = [cons_left; ai_normalized * ai' - model.typeI_d(iZone)];
                    cons_right = [cons_right; erf_value * sqrt(sqr_term)];
% 
                    if iRobot == 2
                        cons_robot = [cons_robot; ai_normalized * ai'-model.typeI_d(iZone), erf_value * sqrt(sqr_term)]
                    end
                    cons_type_one = [cons_type_one; cons];
                end
            end
            value = min(cons_type_one);
%             value = cons_type_one(2);
        end

        function flags = if_in_zone_sensor(con)
            global model
            flags = con.flags;
            robots_loc = con.pos_real_';
            for iRobot = 1:con.nRobot_
                for iZone = 1:con.nTypeI_
                    drone_dist = norm(robots_loc(iRobot, 1:2) - model.typeI_mu(1:2)')
                    if drone_dist < 0.85*model.typeI_d ...
                            && flags(iRobot, iZone) == 0
                        flags(iRobot, iZone) = 1;
                    elseif drone_dist > model.typeI_d ...
                            && flags(iRobot, iZone) == 1
                        flags(iRobot, iZone) = 2;
                    end
                end
            end
            con.flags = flags;
        end
    end
end