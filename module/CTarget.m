classdef CTarget < handle
    % Common base class for targets, used in simulation
    properties
        
        %% id, configuration
        id_
        cfg_
        step_
        v_

        %% state
        pos_real_       =   zeros(3, 2);    % real value
        vel_real_       =   zeros(3, 2);
        
        %% predicted state/path
        dt_             =   0;              % time step
        pos_cov_  =  [];
        vel_cov_  =  [];
        
%         pred_path_      =   [];             % predicted path
%         pred_pathcov_   =   [];             % corresponding covariance
%         pred_state_     =   [];
%         pred_statecov_  =   [];
        
        %% ROS publisher
        pred_state_pub_
        path_pub_
        pred_pathcov_pub_     
        
        
    end
    
    
    methods
        
        %%  =======================================================================
        function obj = CTarget(tID, cfg)
            % constructor
            
            global model
            
            obj.id_     =   tID;
            obj.cfg_    =   cfg;
            obj.step_   =   0;
            
            obj.dt_     =   model.dt;
            obj.v_      =   model.target_start_vel(1,1);
% 
%             obj.pred_path_       =   zeros(3, 1);
%             obj.pred_pathcov_    =   zeros(6, 1);
%             obj.pred_state_      =   zeros(6, 1);
%             obj.pred_statecov_   =   zeros(12, 1);
            
        end
        
        
        %%  =======================================================================
        function initializeROS(obj)
            % Initialize ROS publishers
            
            % publisher for predicted path
             obj.path_pub_ = rospublisher(...
                ['/Target', num2str(obj.id_), '/path_prediction'], ...
                'std_msgs/Float64MultiArray', 'IsLatching', false);
            
        end
        
        
        %%  =======================================================================
        function initializeState(obj, pos, vel, pos_cov, vel_cov)
            % Initilize obs state
            for iStage = 1:2
                obj.pos_real_(:, iStage)    =   pos;
                obj.vel_real_(:, iStage)    =   vel;
            end
            
            obj.pos_cov_ =   pos_cov;
            obj.vel_cov_ =   vel_cov;
            
        end
        
        function circleMotion(obj)
%             radii = [1.0, 3.0, 5.0];
            obj.step_  =   obj.step_ +  1;
            radius     =   0.3;
            offset     =   [0.2, 1.1];
%             if obj.id_  == 3
%                 offset = [6, 0];
% %             end
%             mask       =   0;
%             if mod(obj.id_, 2) == 0
%                 mask   =   1;
%             end
%             theta      =   pi * mod(obj.step_+180*(obj.id_-1), 360) / 180;  % suppose each step corresponds to 1 degree
            theta      =   mod((obj.v_*obj.step_ / radius) + pi * (obj.id_-1), 2*pi);
            theta
            x          =   radius * cos(theta) + offset(1);
            y          =   radius * sin(theta) + offset(2);
            for iStage = 1:2
                obj.pos_real_(:, iStage) = [x; y; 1.2];
            end
        end
        
        %%  =======================================================================
        function propagatePathConstantV(obj)
            
            xpred = [obj.pos_real_(:, 1); obj.vel_real_(:, 1)];
            Ppred = [obj.pos_cov_, zeros(3, 3); ...
                     zeros(3, 3), obj.vel_cov_];
            F = [1, 0, 0, obj.dt_, 0, 0; ...
                 0, 1, 0, 0, obj.dt_, 0; ...
                 0, 0, 1, 0, 0, obj.dt_; ...
                 0, 0, 0, 1, 0, 0; ...
                 0, 0, 0, 0, 1, 0; ...
                 0, 0, 0, 0, 0, 1];

            xpred = F * xpred;
            Ppred = F * Ppred * F';
            
            % publish the path
            if obj.cfg_.addTargetStateNoise == 1
                dpos = zeros(3, 1);
                dvel = zeros(3, 1);
                for i = 1 : 3
                    dpos(i) = random('Normal', 0, ...
                        sqrt(obj.pos_cov_(i,i)));
                    dvel(i) = random('Normal', 0, ...
                        sqrt(obj.vel_cov_(i,i)));
                end
                xpred(1:3)    =   xpred(1:3) + dpos;
                xpred(4:6)    =   xpred(4:6) + dvel;
            end
            
            for iStage = 1:2
                obj.pos_real_(:, iStage) = xpred(1:3);
                obj.vel_real_(:, iStage) = xpred(4:6);
            end
            
            for i = 1:3
                for j = 1:3
                    obj.pos_cov_(i, j) = Ppred(i, j);
                    obj.vel_cov_(i, j) = Ppred(3+i, 3+j);
                end
            end
          
        end
        
        
        %%  =======================================================================
        function sendPath(obj)
  
            msg = rosmessage('std_msgs/Float64MultiArray');
            msg.Data = reshape(obj.pos_real_, [], 1);
            obj.path_pub_.send(msg);
            
        end
            
        function X =  getCurrentPosX(obj)
         
            X = obj.pos_real_(1, 1);
            
        end
        
        function Y = getCurrentPosY(obj)
            
            Y = obj.pos_real_(2, 1);
            
        end
    end
    
   
end