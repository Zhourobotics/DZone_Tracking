function dx = bebop_dynamics(x, u)
        
    global model

    % Bebop 2 dynamics model
    % 
    % state:   x, y, z, vx, vy, vz, phi, theta, psi
    % control: phi_c, theta_c, vz_c, psi_rate_c
    % 
    % f:
    %	\dot( x ) == vx
    %	\dot( y ) == vy
    %   \dot( z ) == vz
    %   \dot( vx) == (cos(psi)*tan(theta)/cos(phi) + sin(psi)*tan(phi))*g - kD_x*vx
    %   \dot( vy) == (sin(psi)*tan(theta)/cos(phi) - cos(psi)*tan(phi))*g - kD_y*vy
    %   \dot( vz) == (k_vz*vz_c - vz) / tau_vz
    %   \dot(phi) == (k_phi*phi_c - phi) / tau_phi
    %   \dot(theta) == (k_theta*theta_c - theta) / tau_theta
    %   \dot(psi) == psi_rate_c
    % 
    %   g           =   9.81
    %   kD_x        =   0.25
    %   kD_y        =   0.33
    %   k_vz        =   1.2270
    %   tau_vz      =   0.3367
    %   k_phi       =   1.1260
    %   tau_phi     =   0.2368
    %   k_theta     =   1.1075
    %   tau_theta   =   0.2318
    %
    % (c) Hai Zhu, TU Delft, 2019, h.zhu@tudelft.nl
    %

    %% Model parameters
    g           =   9.81;
    kD_x        =   0.25;
    kD_y        =   0.33;
    k_vz        =   1.2270;
    tau_vz      =   0.3367;
    k_phi       =   1.1260;
    tau_phi     =   0.2368;
    k_theta     =   1.1075;
    tau_theta   =   0.2318;
    
    nRobot = model.nRobot;
    offset1  = nRobot*3;
    offset2  = nRobot*6;
    
    v  =  [];
    a  =  [];
    deuler  =  [];
    for iRobot = 1 : nRobot
        %% control inputs
        phi_c_i       =   u((iRobot-1)*4+1);
        theta_c_i     =   u((iRobot-1)*4+2);
        vz_c_i        =   u((iRobot-1)*4+3);
        psi_rate_c_i  =   u((iRobot-1)*4+4);

        %% position dynamics
        vx_i          =   x(offset1+(iRobot-1)*3+1);
        vy_i          =   x(offset1+(iRobot-1)*3+2);
        vz_i          =   x(offset1+(iRobot-1)*3+3);

        phi_i         =   x(offset2+(iRobot-1)*3+1);
        theta_i       =   x(offset2+(iRobot-1)*3+2);
    %   psi_i         =   x(offset2+(iRobot-1)*3+3);

    %     ax = (cos(psi)*tan(theta)/cos(phi) + sin(psi)*tan(phi))*g - kD_x*vx;
    %     ay = (sin(psi)*tan(theta)/cos(phi) - cos(psi)*tan(phi))*g - kD_y*vy;

        % in this way, might be computationally easier
        ax_i =  tan(theta_i)*g - kD_x*vx_i;
        ay_i = -tan(phi_i)*g - kD_y*vy_i;
        az_i = (k_vz*vz_c_i - vz_i) / tau_vz;

        %% attitude dynamics
        dphi_i    = (k_phi*phi_c_i - phi_i) / tau_phi;
        dtheta_i  = (k_theta*theta_c_i - theta_i) / tau_theta;
        dpsi_i    = psi_rate_c_i;

        %% output
        v  =  [v; vx_i; vy_i; vz_i];
        a  =  [a; ax_i; ay_i; az_i];
        deuler = [deuler; dphi_i; dtheta_i; dpsi_i];
%         dx_i =  [vx; vy; vz; ax; ay; az; dphi; dtheta; dpsi];
%         dx_   =  [dx_; dx_i];
    end
    dx = [v; a; deuler];
end
