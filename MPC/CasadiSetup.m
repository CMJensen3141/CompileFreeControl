%% This function sets up a CasaDi solver for an MPC problem using the Opti stack
clear all

import casadi.*;

%% Define size of control, state vector, output, and prediction horizon here

m = 1; % Input size
n = 2; % State vector size
p = 1; % Number of outputs to be controlled
N = 3; % Horizon length

%% Define all the necessary (yucky) globals

global opti % Optimizer object
global du % Control input (velocity form)
global usum % Total control input
global x % State (velocity form)
global u0 % Initial control
global x0 % Initial state
global yr % Output reference
global epsi % Slack variable


%% Assign the yucky globals to the Opti object

opti = casadi.Opti('conic');
du = opti.variable(m,N);
usum = opti.variable(m,N);
x = opti.variable(n+p,N);
epsi = opti.variable(n+p,1);

u0 = opti.parameter(m,1);
x0 = opti.parameter(n+p,1);
yr = opti.parameter(p,1);

%% Define model and weight vectors here

dt = 1; % Sampling time

A = [0 1; 0 -6.87];
B = [0; 1];
C = [0.0217 0]; D = 0;

%% Convert
csys = ss(A,B,C,D);
dsys = c2d(csys,dt);

A = dsys.A; B = dsys.B; C = dsys.C;

% Weight matrices
Q = [1,1,1].*eye(n+p,n+p); R = 0.01*eye(1,1); E = 100.*eye(n+p,n+p);


%% Pannochia-Rawlings transformation

Ai = [A zeros(2,1); C*A 1];
Bi = [B;C*B];
Ci = [zeros(1,2) 1];
Di = 0;

intsys = ss(Ai,Bi,Ci,Di,dt);


%% Set up dynamics constraints
J = 0;


for ii = 1:N-1
    opti.subject_to(x(:,ii+1) == x(:,ii) + dt*(Ai*x(:,ii)+Bi*du(:,ii)));
end
for ii = 1:N
    J = J+(x(:,ii)'*Q*x(:,ii)+du(:,ii)'*R*du(:,ii));
end
opti.subject_to(epsi == zeros(n+p,1)-x(:,N));
J = J + epsi'*E*epsi;

opti.minimize(J);
opti.solver('osqp');
