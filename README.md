# ball-on-plate
STM32 implementation of ball on plate.<br/>

A resistor touchscreen has been used for knowing the position of the ball.<br/>
[Alpha Beta](https://en.wikipedia.org/wiki/Alpha_beta_filter) + Low Pass for filtering those position values.<br/>
PID control using [tcleg/PID_Controller](https://github.com/tcleg/PID_Controller) library.<br/>
Complete description (français) : [rapport.pdf](https://github.com/EmileBinot/ball-on-plate/files/6556606/RAPPORT.1.pdf)

Source files located in : <em>./RTE/Device/STM32F446RETx/STCubeGenerated/Src/</em><br/>
Some simulations located in : <em>./Simulations/MATLAB/</em><br/>

## Demo
![ezgif com-gif-maker](https://user-images.githubusercontent.com/8127716/119890968-bc1ca600-bf38-11eb-928b-c824bff84100.gif)<br/> 







