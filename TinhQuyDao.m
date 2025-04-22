% Define the variables
t0 = 0; % Replace with actual value of t_0
tf = 4; % Replace with actual value of t_f
qi = 10;
qf =45; 
qi_dot = 0;
qf_dot = 0;

% Construct the matrix T
M = [1  t0  t0^2  t0^3;
     0  1   2*t0  3*t0^2;
     1  tf  tf^2  tf^3;
     0  1   2*tf  3*tf^2];

% Define the vector q
b = [qi; qi_dot ; qf; qf_dot];


a = inv(M)*b