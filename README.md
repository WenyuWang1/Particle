# Particle

I've previously received invaluable support here that enabled me to use DMA for accessing the ADC, allowing for direct use of ADC in my project. Here is the [example code](https://github.com/rickkas7/photonAudio/blob/master/audio3/audio3.cpp)
I refered to, which uses Dual ADC Regular Simultaneous. I'm now trying to modify it to use Dual ADC Interleaved Mode to increase the sampling rate.

I'm trying to adapt my code with reference to the [ADC_DualModeInterleaved example](https://github.com/STMicroelectronics/STM32CubeF2/tree/master/Projects/STM322xG_EVAL/Examples/ADC/ADC_DualModeInterleaved)
from STM32CubeF2. 

DMA.cpp is my current implementation.

main.py is the Python server I use.

Currently ADC1 and ADC2 are working, but I'm not sure if they work properly. I'll do further check.
