# Particle

I've previously received invaluable support here that enabled me to use DMA for accessing the ADC, allowing for direct use of ADC in my project. Here is the [example code](https://github.com/rickkas7/photonAudio/blob/master/audio3/audio3.cpp)
I refered to, which uses Dual ADC Regular Simultaneous. I'm now trying to modify it to use Dual ADC Interleaved Mode to increase the sampling rate.

I'm trying to adapt my code with reference to the [ADC_DualModeInterleaved example](https://github.com/STMicroelectronics/STM32CubeF2/tree/master/Projects/STM322xG_EVAL/Examples/ADC/ADC_DualModeInterleaved)
from STM32CubeF2. However, I've encountered issues where the sendBuf is consistently empty, indicating that my ADCs may not be configured correctly.

 DMA.cpp is my current implementation.

main.py is the Python server I use.

I'm unsure where the misconfiguration might be occurring. Could anyone guide me on where to look or what might be missing in my ADC setup? Any insights or suggestions would be greatly appreciated as I work towards resolving this issue.

Thank you for your time and assistance!
