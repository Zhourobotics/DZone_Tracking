function xNext = first_order_dynamics(z) 

    global model
    x      = z(model.nin+1: model.nvar);
    u      = z(1:model.nin);
    dim    = model.stateDim;
    dt     = model.dt;
    nRobot = model.nRobot;
    
    posNext  = [];
    %%% size of P: (3, 6)
    P = [eye(dim), dt * eye(dim)];
    for iRobot = 1 : nRobot
        vx  =  u((iRobot-1)*3+1);
        vy  =  u((iRobot-1)*3+2);
%         vz  =  u((iRobot-1)*3+3);
        
        pos_x = x((iRobot-1)*3+1);
        pos_y = x((iRobot-1)*3+2);
        pos_z = x((iRobot-1)*3+3);

%         Next_i = P * [pos_x; pos_y; pos_z; vx; vy; vz];
        Next_i = P * [pos_x; pos_y; pos_z; vx; vy; 0];
        posNext = [posNext; Next_i];

    end
    xNext  =  posNext;
    
end
