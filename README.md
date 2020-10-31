# ECE-6110-Time-of-Flight
 
ECE 6110 - Tyler McCormick - Quiz 1

This code accomplishes the following: using the API developed by ST, the time-of-flight sensor
is initialized and used to provide proximity information, which is then displayed
over a UART (configured as 8n1 at 9600 baud) connection. 

In addition I have added a few useful features. The default measurement is provided in mm (metric) but 
by pressing the blue button, an interrupt is triggered, which changes the measurement to inches (imperial). 
Pressing the button again toggles the measurement back to mm, and the option for mm or inches can be toggled 
as many times as desired.

Lastly, I added an out-of-range detector, as the sensor seems to be accurate only for a short distance. 
Any object that is too far away will display an error on the terminal.
