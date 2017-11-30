function [F, M] = controller(t, state, des_state, params)

g = params.gravity;
m = params.mass;

Kpz = 55;             Kpx = -40;                    Kpy = -25;   
Kdz = 14;               Kdx = -10;                    Kdy = -7;

Kpphi =  5;           Kptheta = 5;             Kpsigh = 2;
Kdphi = 0.2;             Kdtheta = 0.2;              Kdsigh = 0.4;

F = params.mass*params.gravity + params.mass * (Kdz * (des_state.vel(3) - state.vel(3) + Kpz * (des_state.pos(3) - state.pos(3))));

% States 
rx = state.pos(1); 
rx_dot = state.vel(1); 

ry = state.pos(2); 
ry_dot = state.vel(2); 

% Desired state 
rx_des = des_state.pos(1); 
rx_dot_des = des_state.vel(1); 
rx_ddot_des = des_state.acc(1);

ry_des = des_state.pos(2); 
ry_dot_des = des_state.vel(2); 
ry_ddot_des = des_state.acc(2);

% Commanded State 
rx_ddot_c = rx_ddot_des - Kdx * (rx_dot_des - rx_dot) - Kpx * (rx_des - rx);
ry_ddot_c = ry_ddot_des - Kdy * (ry_dot_des - ry_dot) - Kpy * (ry_des - ry);

% Current roll, pitch and yaw
phi = state.rot(1);
theta = state.rot(2);
sigh = state.rot(3);

% Desired roll, pitch and yaw 
sigh_des = des_state.yaw;
phi_des = (1/params.gravity) * (rx_ddot_c *sin(sigh_des) - ry_ddot_c * cos(sigh_des)) ;
theta_des = (1/params.gravity) * (rx_ddot_c * cos(sigh_des) + ry_ddot_c * sin(sigh_des));

% Commanded roll, pitch and yaw 
phi_c = (1/params.gravity) * (des_state.acc(1) * sin(sigh_des) - des_state.acc(2) * cos(sigh_des));
theta_c = (1/params.gravity) * (des_state.acc(1) * cos(sigh_des) + des_state.acc(2) * sin(sigh_des));
sigh_c = sigh_des;

% Current Angular velocities
p = state.omega(1);
q = state.omega(2);
r = state.omega(3);

% Desired Angular Velocities 
p_des = 0;
q_des = 0;
r_des = des_state.yawdot;

% Commmanded Angular Velocities
p_c = 0;
q_c = 0;
r_c = r_des;

% Compute Moment Elements

u2_1 = Kpphi * (phi_des - phi) + Kdphi * (p_des - p);
u2_2 = Kptheta * (theta_des - theta) + Kdtheta * (q_des - q);
u2_3 = Kpsigh * (sigh_des - sigh) + Kdsigh * (r_des - r);

M = [u2_1; u2_2; u2_3];

end
