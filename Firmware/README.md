# Firmware 

## Bare metal peripheral drivers for STM32F401 BlackPill

The Firmware presents a high level API representing a DifferentialDrive Mobile Robot; in which the Robots Linear and Angular Velocities are controlled[1]. 

The Firmware implements a cascaded controller to maintain balance. 

More detailed control is available via RPC style Commands (GET/SET/RUN) execute functions via cmd ID lookup. 
Data Is published over topics at 10Hz.




### Cascaded PID Controller
![Cascaded_Controller](../Docs/Charts/CascadedPID.svg)


