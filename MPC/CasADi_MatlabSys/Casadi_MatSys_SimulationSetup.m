dt = 0.1; % Sampling time

m = 1; % Input size
n = 2; % State vector size
p = 1; % Number of outputs to be controlled
N = 15; % Horizon length
r_wheel = 0.0350; % Wheel diameter

A_mat = [0 1; 0 -6.87];
B_mat = [0; 1];
C_mat = [0.0217 0]; D_mat = 0;

csys = ss(A_mat,B_mat,C_mat,D_mat);
dsys = c2d(csys,dt,'Tustin');


A_mat = dsys.A; B_mat = dsys.B; C_mat = dsys.C;

% Weight matrices
Q = [1,0.1,10000].*eye(n+p,n+p); R = 0.01*eye(1,1); E = 1000.*eye(n+p,n+p);
