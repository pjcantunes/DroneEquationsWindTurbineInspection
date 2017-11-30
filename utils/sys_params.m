function params = sys_params()
% SYS_PARAMS basic parameters for the quadrotor

m = 1.38; % kg original 0,18
g = 9.81; % m/s/s
I = [0.00025,   0,          2.55e-6;
     0,         0.000232,   0;
     2.55e-6,   0,          0.0003738];

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.gravity = g;
params.arm_length = 0.17; % m  0.086 original

params.minF = 0.0;
params.maxF = 1.5*m*g;

end
