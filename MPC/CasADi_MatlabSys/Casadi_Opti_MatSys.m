classdef Casadi_Opti_MatSys < matlab.System
    % Implements MPC using the CasADi Opti stack
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    properties
        % Public, tunable properties.

        A % State transition matrix
        B % Input matrix
        C % Output matrix
        D % Feedthrough matrix
        dt % Sampling time
        N % Control horizon
        Q % State weight matrix
        R % Input weight matrix
        E % Slack variable weight
    end

    properties (DiscreteState)
    end

    properties (Access = private)
        % Pre-computed constants.
        opti
        x0
        u0
        x
        du
        usum
        epsi
        u_bounds
        du_bounds
        n
        m
        p
    end

    methods (Access = protected)
        function num = getNumInputsImpl(~)
            num = 4;
        end
        function num = getNumOutputsImpl(~)
            num = 2;
        end
        function [sz1,sz2] = getOutputSizeImpl(obj)
            n = size(obj.A,1); m = size(obj.B,2); p = size(obj.C,1);
        	sz1 = [m,obj.N];
            sz2 = [n+p,obj.N];
        end

        function [out,out2] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = "double";
            out2 = "double";
        end
        function [sz1,sz2,sz3,sz4] = getInputSizeImpl(obj)
            n = size(obj.A,1); m = size(obj.B,2); p = size(obj.C,1);
        	sz1 = [n+p,1];
            sz2 = [m,1];
            sz3 = [m,2];
            sz4 = [m,2];
        end
        function [cp1,cp2,cp3,cp4] = isInputComplexImpl(~)
        	cp1 = false; cp2 = false; cp3 = false; cp4 = false;
        end
        function [cp1,cp2] = isOutputComplexImpl(~)
        	cp1 = false;
            cp2 = false;
        end
        function [fz1,fz2,fz3,fz4] = isInputFixedSizeImpl(~)
        	fz1 = true; fz2 = true; fz3 = true; fz4 = true;
        end
        function [fz1,fz2] = isOutputFixedSizeImpl(~)
        	fz1 = true; fz2 = true;
        end
        function [Ai,Bi,Ci] = MakePannochiaRawlings(obj)
            Ai = [obj.A zeros(obj.n,1); obj.C*obj.A eye(obj.p,obj.p)];
            Bi = [obj.B;obj.C*obj.B];
            Ci = [zeros(obj.p,obj.n) eye(obj.p,obj.p)];
        end
        function setupImpl(obj)
            % Implement tasks that need to be performed only once, 
            % such as pre-computed constants.
            
            import casadi.*

            obj.opti = casadi.Opti('conic'); % Initialize optimizer for QP
            obj.n = size(obj.A,1); % Number of states
            obj.m = size(obj.B,2); % Number of inputs
            obj.p = size(obj.C,1); % Number of outputs

            % Define variables for the optimization problem
            obj.x = obj.opti.variable(obj.n+obj.p,obj.N);
            obj.du = obj.opti.variable(obj.m,obj.N);
            obj.usum = obj.opti.variable(obj.m,obj.N);
            obj.epsi = obj.opti.variable(obj.n+obj.p,1);

            % Define parameters for the optimization problem
            obj.x0 = obj.opti.parameter(obj.n+obj.p,1);
            obj.u0 = obj.opti.parameter(obj.m,1);
            obj.u_bounds = obj.opti.parameter(2*obj.m,1);
            obj.du_bounds = obj.opti.parameter(2*obj.m,1);
            
            % Make the Pannochia-Rawlings integrator matrices

            [Ai,Bi,Ci] = MakePannochiaRawlings(obj);

            J = 0; % Initialize cost function
            % Dynamics constraint
            for ii = 1:obj.N-1
                obj.opti.subject_to(obj.x(:,ii+1) == obj.x(:,ii)+obj.dt*(Ai*obj.x(:,ii)+Bi*obj.du(:,ii)));
            end
            % Initial state constraint
            obj.opti.subject_to(obj.x(:,1) == obj.x0);
            % Initial control constraint
            obj.opti.subject_to(obj.usum(:,1) == obj.u0 + obj.du(:,1));
            % Control summation constraint
            for ii = 2:obj.N
            obj.opti.subject_to(obj.usum(:,ii) == obj.usum(:,ii-1)+obj.du(:,ii));
            end
            % Slackened terminal constraint
            obj.opti.subject_to(obj.epsi == zeros(obj.n+obj.p,1)-obj.x(:,obj.N)); % 
            % Control value and increment constraints
            for ii = 1:obj.N
                for jj = 1:obj.m
                obj.opti.subject_to(obj.usum(jj,ii) <= obj.u_bounds(1,jj));
                obj.opti.subject_to(obj.u_bounds(2,jj) <= obj.usum(jj,ii));
                obj.opti.subject_to(obj.du(jj,ii) <= obj.du_bounds(1,jj));
                obj.opti.subject_to(obj.du_bounds(2,jj) <= obj.du(jj,ii));
                end
            end
            % Cost function summation
            for ii = 1:obj.N
                J = J + obj.x(:,ii)'*obj.Q*obj.x(:,ii)+obj.du(:,ii)'*obj.R*obj.du(:,ii);
            end
            J = J + obj.epsi'*obj.E*obj.epsi;
            obj.opti.minimize(J);
            obj.opti.solver('osqp');
        end

        function [du,x] = stepImpl(obj,u0,x0,u_bounds,du_bounds)
            
            % Update parameters
            obj.opti.set_value(obj.x0,x0);
            obj.opti.set_value(obj.u0,u0);
            obj.opti.set_value(obj.u_bounds,u_bounds);
            obj.opti.set_value(obj.du_bounds,du_bounds);
            
            try 
                sol = obj.opti.solve();
                x = sol.value(obj.x);
                du = sol.value(obj.du);
            catch

                du = zeros(obj.m,obj.N);
                x = nan(obj.n+obj.p,obj.N);
            
            end
        end

        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
end
