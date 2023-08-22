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
       
        function [AngPos,SpeedInput] = stepImpl(obj,SpeedInput)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.

            % Limit speed to acceptable interval
            SpeedInput = max(min(SpeedInput,100),-100); 
            
            % Write speed to motor
            obj.motor_handle.Speed = SpeedInput;

            % Assign outputs
            SpeedInput = SpeedInput;
            AngPos = readRotation(obj.motor_handle);
            AngPos = double(AngPos);
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
            num = 2;
            % if obj.UseOptionalOutput
            %     num = 2;
            % end
        end
        
        function [sz_1, sz_2] = getOutputSizeImpl(obj)
            sz_1 = [1 1];
            sz_2 = [1 1];
        end

        function [type_1, type_2] = getOutputDataTypeImpl(~)
         type_1 = 'double'; type_2 = 'double';
        end

        function [cplxout_1, cplxout_2] = isOutputComplexImpl(~)
         cplxout_1 = false; cplxout_2 = false;
        end

        function [fx_1, fx_2] = isOutputFixedSizeImpl(~)
         fx_1 = true; fx_2 = true;
        end
            
    end
end
