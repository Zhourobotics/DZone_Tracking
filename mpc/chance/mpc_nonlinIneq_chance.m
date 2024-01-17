function ineq = mpc_nonlinIneq_chance(z, p)

    % define nonlinear inequalities for mpc

    global index model                           % global index information
    
    
    %% obtaining necessary information
    env_dim     =   model.stateDim;
    ego_pos     =   z(index.z.pos);         % current stage position [x, y, z]
    ego_pos     =   reshape(ego_pos, [model.stateDim, model.nRobot])'; 
    pos_typeI   =   reshape(model.typeI_mu, [model.stateDim, model.nTypeI])';
    pos_typeII  =   reshape(model.typeII_mu, [model.stateDim, model.nTypeII])';
    
    
    %% constraint for type-I zones
    cons_type_one = [];
    for iRobot = 1 : model.nRobot
        for iZone = 1 : model.nTypeI
            ai =  pos_typeI(iZone, :) - ego_pos(iRobot, :); % row vector
            ai_normalized = ai / norm(ai);
            erf_value = erfinv(1 - 2 * model.eps1(iRobot));
            
%             sqr_term = 2 * ai * model.typeI_cov((iZone-1)*env_dim+1: iZone*env_dim,  (iZone-1)*env_dim+1: iZone*env_dim) * ai';
            sqr_term = 2 * ai_normalized * model.typeI_cov((iZone-1)*env_dim+1: iZone*env_dim,  (iZone-1)*env_dim+1: iZone*env_dim) * ai_normalized';

            cons  =  ai_normalized * ai' - erf_value * sqrt(sqr_term) - model.typeI_d(iZone);
            cons_type_one = [cons_type_one; cons];
        end
    end
    
     %% constraint for type-II zones
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
                  neighbor_dist = min(neighbor_dist, model.max_dist);
                  max_neighbor_dist = max(max_neighbor_dist, neighbor_dist);
              end
%               max_neighbor_dist = min(max_neighbor_dist, model.max_dist);
              cons = ai_normalized * ai' - erf_value * sqrt(sqr_term) - model.delta(iZone) * max_neighbor_dist;
              cons_type_two = [cons_type_two; cons];
          end
     end
             
    %% inter-robot collision avoidance
%      cons_inter_robot_collision = [];
%      for iRobot = 1 : model.nRobot
%          for jRobot = 1 : model.nRobot
%              % dist = if_else(iRobot~=jRobot, norm(ego_pos(iRobot, :) - ego_pos(jRobot, :)), SX.zeros(1));
%              dist = norm(ego_pos(iRobot, :) - ego_pos(jRobot, :));
%              cons_inter_robot_collision = [cons_inter_robot_collision; dist];
%          end
%      end
         
            
    %% connectivity maintenance
    % dist_matrix = SX.sym('dist_matrix', model.nRobot, model.nRobot);
%     dist_matrix = [];
%     for iRobot = 1 : model.nRobot
%         for jRobot = 1 : model.nRobot
%              dist = norm(ego_pos(iRobot, :) - ego_pos(jRobot, :));
%              dist_matrix = [dist_matrix; dist];
%         end
%     end
%     dist_matrix = reshape(dist_matrix, [model.nRobot, model.nRobot]);
%     A = exp((model.max_dist ^ 2 - dist_matrix.^2).^2 ./ model.sigma_scale) - 1;
%     neighbor_idx = dist_matrix <= model.max_dist; % mask
%     A = A .* neighbor_idx;
%     D_vec = sum(A, 2);         % sum of each row
%     D = diag(D_vec);           % weighted degree matrix
%     L = D - A;                 % weighted Laplacian matrix
%     eig_values = eig_symbolic(L);       % get eigen values
%     % eig_values = eig(L);
%     lambda_2 = eig_values(2);  % algebraic connectivity lambda_2
%     
    %% combine inequality constraints
     ineq = [cons_type_one; cons_type_two];
%      ineq = [cons_type_one];
end