function xNext = simple_dynamics(z) 

    global model
    x      = z(model.nin+1: model.nvar);
    u      = z(1:model.nin);
    dim    = model.stateDim;
    dt     = model.dt;
    nRobot = model.nRobot;
    
    posNext  = [];
    velNext  = [];
%     accNext  = [];
    offset = nRobot * 3;
    P = [eye(dim), dt * eye(dim);...
        zeros(dim, dim), eye(dim)];
    for iRobot = 1 : nRobot
        ax  =  u((iRobot-1)*3+1);
        ay  =  u((iRobot-1)*3+2);
        az  =  u((iRobot-1)*3+3);
        
        incre = [0.5*ax*dt^2; 0.5*ay*dt^2; 0.5*az*dt^2;...
            ax*dt; ay*dt; az*dt];
        
        pos_x = x((iRobot-1)*3+1);
        pos_y = x((iRobot-1)*3+2);
        pos_z = x((iRobot-1)*3+3);
       
        vx  =  x(offset+(iRobot-1)*3+1);
        vy  =  x(offset+(iRobot-1)*3+2);
        vz  =  x(offset+(iRobot-1)*3+3);
        
        xNext_i = P * [pos_x; pos_y; pos_z; vx; vy; vz] + incre;
        
        posNext = [posNext; xNext_i(1:3)];
        velNext = [velNext; xNext_i(4:6)];
%         accNext = [accNext; ax; ay; az];
%         xNext = [xNext; xNext_i];

    end
    xNext  =  [posNext; velNext];
    
end