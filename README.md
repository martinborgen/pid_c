# pid_c
A project to implement a genereal purpose PID controller in C. 
The idea is that it should be easy to include in some embedded project, and should provide all the overflow handling etc that otherwise becomes a bother to implement. 

### Current state of the controller
The controller seems to work reasonably well in PI mode and some limited testing in Simulink has been done. The D-part is still untested. 
The current controller uses exclusively 32-bit integers, which means you probably want to scale your input/output values by some factor to use the available resolution. 

## DISCLAIMER:
The author(s) take no responsibility for the safety and/or performance of this controller. It is a hobby project and should not be relied on in safety critical applications. 
