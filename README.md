# BLDC-Halless-Majority-Function

单片机: 基于JMT18F003Plus(51内核) </br>
开发平台：VScode + EIDE + Keil C51编译器 </br>
过零点检测：***纯ADC检测*** </br>
过零点滤波：***择多函数***  </br>
必要资源：  三路ADC(过零点检测)，六路PWM(驱动MOS管) </br>
可选资源：两路ADC(电压, 电流检测)，PWM输入捕获(速度调节)，两个定时器(速度计算和换相延时，堵转保护)，CMP比较器(过流保护) </br>
项目介绍：本项目通过ADC检测过零点信号(利用PWM中央对齐模式，在高电平中间触发，避免了错误信号，并且在每次换相过后，进行几个周期的等待，防止信号抖动产生的误判)，通过择多函数进行滤波，从而得到较为准确的换相条件，目前测试过多个电机，都可以使用。 </br>
Tip：不同电机，需要调整参数：换相延迟，启动的过零点检测延迟。高速不推荐滤波了 </br>
