classdef LegoEV3Brick < matlab.System
    % LegoEV3Brick Connects to an EV3Brick and its motor over WiFi
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties
        
    end

    properties (Nontunable)
        address = '127.0.0.1'
        motorport = 'A'
        id = '00165340e49b'
        retry_attempts = 5;
    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
    	connection
        motor_handle
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Initialise connection to EV3 brick
            cntr = 0;
            while (exist('myev3') == 0) && (cntr < obj.retry_attempts)
                try
                    myev3 = legoev3('WiFi',obj.address,obj.id);
                catch
                    disp('Retrying connection to EV3')
                    cntr = cntr+1;
                end
            end

            obj.connection = myev3;
            % Initialise motor connection
            obj.motor_handle = motor(obj.connection,obj.motorport);
            obj.motor_handle.Speed = 0;
            resetRotation(obj.motor_handle);
            start(obj.motor_handle);
        end
       
        function motorout = stepImpl(obj,SpeedInput)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.

            % Limit speed to acceptable interval
            SpeedInput = max(min(SpeedInput,100),-100); 
            
            % Write speed to motor
            obj.motor_handle.Speed = SpeedInput;

            % Assign outputs
            motorout(1,2) = SpeedInput;
            motorout(1,1) = readRotation(obj.motor_handle);
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

        function releaseImpl(obj)
            % Release resources, such as file handles
            stop(obj.motor_handle);
            clear obj
            clear myev3
        end

        function num = getNumOutputsImpl(obj)
            % Define total number of outputs for system with optional
            % outputs
            num = 1;
            % if obj.UseOptionalOutput
            %     num = 2;
            % end
        end
        
        function [sz_1] = getOutputSizeImpl(obj)
            outsz = 2;
            sz_1 = [1 outsz];
        end

        function dataout = getOutputDataTypeImpl(~)
         dataout = 'double';
        end

        function cplxout = isOutputComplexImpl(~)
         cplxout = false;
        end

        function fixedout = isOutputFixedSizeImpl(~)
         fixedout = true;
        end
            
    end
end
