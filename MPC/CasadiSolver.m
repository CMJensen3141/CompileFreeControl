function ret = CasadiSolver(ref,init_state,init_ctrl,du_b,u_b)
% This function calls a globally defined opti solver and returns its
% solution trajectories for supply, return, indoor temperature, and flow

global opti % Optimizer object
global du % Control input (velocity form)
global usum % Total control input
global x % State (velocity form)
global u0 % Initial control
global x0 % Initial state
global yr % Output reference
global epsi % Slack variable

localopti = opti.copy;

% Parameter values
localopti.set_value(x0,init_state); 
localopti.set_value(u0,init_ctrl); 
localopti.set_value(yr,ref);

N = size(x,2);
n = size(x,1);
m = size(du,1);

% Set up constraints
opti.subject_to(usum(:,1) == u0 + du(:,1));
opti.subject_to(x(:,1) == x0);

for ii = 2:N
        opti.subject_to(usum(:,ii) == usum(:,ii-1) + du(:,ii));
end
for ii = 1:N
    localopti.subject_to(usum <= u_b(1));
    localopti.subject_to(u_b(2) <= usum);
    localopti.subject_to(du(:,ii) <= du_b(1));
    localopti.subject_to(du_b(2) <= du(:,ii));
end

try
    sol = localopti.solve();
    xvals = reshape(sol.value(x).',1,[]);
    uvals = reshape(sol.value(du).',1,[]);
    ret = [uvals,xvals];
catch
    ret = zeros(1,N*n+N*m);
end


end

