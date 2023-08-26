classdef ModbusTCP_RW < matlab.System
    % ModbusTCP_RW Modbus TCP/IP block with read/write capability.
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties
        address = '127.0.0.1' % Server IP Address
        port = 5020 % Server port ID
        sampling_time = 1;
        coils_to_read  = 1; % Coils to read (0 for no read)
        coils_read_start  = 1; % Coil read start address
        hr_to_read  = 1; % Holding registers to read (0 for no read)
        hr_read_start  = 1; % Holding register read start address
        di_to_read  = 1; % Discrete inputs to read (0 for no read)
        di_read_start  = 1; % Discrete input read start
        ir_to_read  = 1; % Input registers to read (0 for no read)
        ir_read_start  = 1; % Input register read start
        coils_write_start  = 1; % Coil write start (0 for no write)
        hr_write_start  = 1; % Holding register write start (0 for no write)
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
    properties (Access = private)
        client
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Set up Modbus client
            obj.client = modbus('tcpip',obj.address,obj.port);
        end

        function [coil_vals,holdingreg_vals,inputreg_vals,discreteinput_vals] = stepImpl(obj,holdingregs_in,coils_in)
            % Read and write calls

            % Handle reads
            if (obj.coils_to_read ~= 0)     
                 coil_vals = zeros(1,obj.coils_to_read);
                 temp = read(obj.client,'coils',obj.coils_read_start,obj.coils_to_read);
                 coil_vals = temp;
            else
                coil_vals = nan(1,1);
            end
            if (obj.hr_to_read ~= 0)     
                 holdingreg_vals = zeros(1,obj.hr_to_read);
                 temp = read(obj.client,'holdingregs',obj.hr_read_start,obj.hr_to_read);
                 holdingreg_vals(1,:) = temp;
            else
                holdingreg_vals = nan(1,1);
            end
            if (obj.ir_to_read ~= 0)     
                 inputreg_vals = zeros(1,obj.ir_to_read);
                 temp = read(obj.client,'inputregs',obj.ir_read_start,obj.ir_to_read);
                 inputreg_vals(1,:) = temp;
            else
                inputreg_vals = nan(1,1);
            end
            if (obj.di_to_read ~= 0)     
                 discreteinput_vals = zeros(1,obj.di_to_read);
                 temp = read(obj.client,'inputs',obj.di_read_start,obj.di_to_read);
                 discreteinput_vals(1,:) = temp;
            else
                discreteinput_vals = nan(1,1);
            end
            % 
            % Handle writes
            if (obj.hr_write_start == 0)
            else
                write(obj.client,'holdingregs',obj.hr_write_start,holdingregs_in);
            end
            if (obj.coils_write_start == 0)
            else
                write(obj.client,'coils',obj.coils_write_start,coils_in);
            end
            
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

        function num = getNumInputsImpl(obj)
            num = 2;
        end

        function num = getNumOutputsImpl(~)
            num = 4;
        end

        function [sz1,sz2,sz3,sz4] = getOutputSizeImpl(obj)
            sz1 = [1,max(obj.coils_to_read,1)];
            sz2 = [1,max(obj.hr_to_read,1)];
            sz3 = [1,max(obj.di_to_read,1)];
            sz4 = [1,max(obj.ir_to_read,1)];
        end
        
        function [dto1,dto2,dto3,dto4] = getOutputDataTypeImpl(~)
            dto1 = 'double';
            dto2 = 'double';
            dto3 = 'double';
            dto4 = 'double';
        end

        function [cplx1,cplx2,cplx3,cplx4] = isOutputComplexImpl(~)
            cplx1 = false;
            cplx2 = false;
            cplx3 = false;
            cplx4 = false;
        end

        function [fxout1,fxout2,fxout3,fxout4] = isOutputFixedSizeImpl(~)
            fxout1 = true;
            fxout2 = true;
            fxout3 = true;
            fxout4 = true;
        end

        function sts = getSampleTimeImpl(obj)
            % Define sample time type and parameters
            sts = obj.createSampleTime("Type", "Discrete", ...
                "SampleTime", obj.sampling_time);

            % Example: specify discrete sample time
            % sts = obj.createSampleTime("Type", "Discrete", ...
            %     "SampleTime", 1);
        end

    end
end
