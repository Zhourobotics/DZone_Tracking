
%% problem setup
global model                           % the model setup is accessed globally
model.stateDim         =   3;          % actually 2D problem, z held constant
model.nRobot           =   nRobot;     % number of mavs in the problem
model.nTarget          =   nTarget;    % number of targets
model.nTypeI           =   nTypeI;     % number of type I zones
model.nTypeII          =   nTypeII;    % number of type II zones
model.nParamPerTarget  =   9  ;        % number of parameters each target
                                       % 3 + 6 (pos, cov)
model.N                =   2 ;        % horizon length
model.dt               =   0.75;       % time step

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% model.nvar             =   model.nRobot * 3 + model.nRobot * model.stateDim * 2; % number of stage variables (z)                        
% model.neq              =   model.nRobot * model.stateDim * 2 ;        % number of equality constraints (x)
% model.nh               =   model.nRobot * model.nTypeI + model.nRobot * model.nTypeII + ...
%       model.nRobot * model.nRobot; % number of inequality constraints
% model.nh               =   model.nRobot * model.nTypeI + model.nRobot * model.nTypeII;
% model.nh               =   model.nRobot * model.nTypeI; 
% model.nin              =   model.nRobot * 3   ;      % number of control inputs (u)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% set up the optimizer dimensions, first-order control
model.nvar             =    model.nRobot * model.stateDim + model.nRobot * model.stateDim;
model.neq              =    model.nRobot * model.stateDim;
model.nin              =    model.nRobot * model.stateDim;
% model.nh  =  0;
model.nh               =    model.nRobot * model.nTypeI + model.nRobot * model.nTypeII;
model.npar             =   5 + model.nTarget * model.nParamPerTarget;  % run time parameters per stage
%% parameters related to dangerous zones, read from a config file                             
model.typeI_mu         =  config.typeI_mu;
model.typeI_cov        =  diag(config.typeI_cov);
model.typeII_mu        =  config.typeII_mu;
model.typeII_cov       =  diag(config.typeII_cov);
model.typeI_d          =  config.typeI_d;     % radius of each type I zone
model.typeII_d         =  config.typeII_d;    % radius of each type II zone
model.delta            =  config.delta;       % delta of type II zones
model.eps1             =  config.eps1;        % probablity parameter for type I zones
model.eps2             =  config.eps2;        % probablity parameter for type II zones
model.target_start_vel =  config.targetStartVel;
%%%%%% maybe won't be used %%%%%%
model.min_dist         =  1;        % minimum safety distance
model.max_dist         =  config.max_dist;    %4.0;      % communication range 
model.sigma_scale      = model.max_dist ^ 4 / log(2);  %  scaling factor to normalize the weighted adjacency values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% target process description
model.target_process_matrix  =  eye(nTarget * model.stateDim);
model.target_process_noise_cov = 0.05 * eye(nTarget * model.stateDim);

%% characteristic parameters for the measurement of range and bearing sensors
model.range_peak    =  15.0;
model.range_shape   =  0.2;
model.bearing_peak  =  15.0;
model.bearing_shape =  0.2;

%% Indexing
global index
%% stage vector, each stage
index.z.all         =  1:model.nvar;
index.z.inputs      =  1:model.nin;   % control input, [vx, vy, vz], for each robot
index.z.pos         =  (model.nin+1):model.nvar;  % position, [x, y, z] for each robot

%% state vector
index.x.all         =   1:model.neq;
index.x.pos         =   1:model.stateDim*model.nRobot;

%% in parameter vector, problem, each stage
index.p.all         =   1:model.npar;
index.p.envDim      =   1:3;    % [xdim, ydim, zdim]
index.p.weights     =   4:5;    % [w_trace, w_input]
              
idxBegin = index.p.weights(end) + 1;
index.p.targetParam = [reshape(idxBegin:(idxBegin-1)+model.stateDim*model.nTarget, ...
    [3, model.nTarget]);
    reshape(idxBegin+3*model.nTarget:(idxBegin-1)+model.nParamPerTarget*model.nTarget,...
    [6, model.nTarget])];
% index for each target    
index.p.target.pos     =   idxBegin:(idxBegin-1)+model.stateDim*model.nTarget;
index.p.target.posCov  =   idxBegin+model.stateDim*model.nTarget : ...
    (idxBegin-1) + model.nParamPerTarget*model.nTarget;


