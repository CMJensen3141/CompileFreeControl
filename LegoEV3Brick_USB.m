classdef LegoEV3Brick_USB < matlab.System
    % LegoEV3Brick Connects to an EV3Brick and its motor over USB
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties
    end

    properties (Nontunable)
        motorport = 'A'
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
            obj.connection = legoev3('USB');
            % Initialise motor connection
            obj.motor_handle = motor(obj.connection,obj.motorport);
            obj.motor_handle.Speed = 0;
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
            motorspeed = SpeedInput;
            motorpos = readRotation(obj.motor_handle);

            motorout(1,1) = motorpos;
            motorout(1,2) = motorspeed;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

        function releaseImpl(obj)
            % Release resources, such as file handles

            stop(obj.motor_handle);
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
         dataout = 'int32';
        end

        function cplxout = isOutputComplexImpl(~)
         cplxout = false;
        end

        function fixedout = isOutputFixedSizeImpl(~)
         fixedout = true;
        end
            
    end
end
