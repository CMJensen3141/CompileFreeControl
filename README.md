# CompileFreeControl
Contains Matlab classes for compile-free control over Modbus and the Lego EV3 protocols

DEPENDENCIES:

- The Real-Time Pacer for Simulink (https://www.mathworks.com/matlabcentral/fileexchange/29107-real-time-pacer-for-simulink) is required to run all examples.
- The Industrial Communications Toolbox is required to run Modbus examples (https://se.mathworks.com/products/industrial-communication.html)
- The Lego EV3 Hardware Support from MATLAB package is required to run LegoEV3 examples (https://se.mathworks.com/hardware-support/lego-mindstorms-ev3-matlab.html)
- CasADi is required to run MPC examples (https://web.casadi.org/)

NOTA BENE:

- The Interpreted Matlab Function MPC example will become deprecated in the future as Matlab are removing support for this block.
- The DummyServer executable is a very simple localhosted Modbus server written in Rust using the rodbus framework (https://stepfunc.io/) and intended for debugging of Modbus examples.
	- The IP address is 127.0.0.1 and the port number is 5020, which are also the defaults for the Modbus TCP client block in Matlab.
- I may build a Modbus RTU server for the same purpose at some point, but zero guarantees.

