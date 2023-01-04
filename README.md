# NodeMCU Mixing Valve Controller for Heating System
Many old boilers for heating systems use mixing valve to adjust water temperature in the radiators. In my case, the boiler can operate at internal water temperatures between 50℃ to 90℃ and the user is supposed to adjust the mixing valve manually so that the temperature in radiators is somewhere in the range 30℃ to 50℃, depending on power demands of the house. The internal temperature can vary depending on the type of heat sourse used: external heating pump, firewood, or electric heater. The heating demand of the house can also vary signifgicantly based on external temparatures and desired temperatures in each room. This makes a perfect application of an automatic PID control.

## Valve Operation Hardware
In my case, the valve is quite old, so moving it by hand on the shaft is nearly impossible. I use a standard 37mm 12V DC gear-motor with gear ratio 100:1 and a chain link with bike sprockets. The corresponding blueprints and overall system view is (going to be) available in the "hardware" folder.

## Electronics

## Firmware
The firmware for NodeMCU is developed using PlatformIO add-on for VS-Code. 

### Naming conventions
Underscore is used as a prefix for all variables and objects declared in global scope! This is somewhat none-standard, but it makes it easier at least for me to read the code.

### Valve motor controller
Valve shaft is operated by a geared DC motor. The motor is driven in PWM mode by a PID controller. The feedback is from the quadrature encoder on the motor shaft. 

### Temperature controller
The current temperature is received from a DS1800 sensor on a 1-Wire interface. I my case, I placed the temperature sensor on the return pipe. The reasoning behind this is that the energy demand of the house depends a lot on the external temperature, so the colder it is outside, the warmer the flow pipe has to be to allow every room keep the needed temperature. By making sure that the temperature in the return is no lower than a current house setting (e.g. +22℃ during the day, +18℃ over night, or +10℃ in vacation mode), the flow will be adjusted my the mixing valve to the needed level. 

