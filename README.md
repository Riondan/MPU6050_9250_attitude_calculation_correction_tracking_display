# MPU-attitude-calculation-correction-and-data-tracking-display
MPU9250/MPU6050, Mahony attitude calculation, ellipse/circle/maximum correction, Matlab serial port simulation, 2D/3D attitude real-time tracking

废话不多说，demo来一波：https://user-images.githubusercontent.com/50388568/109762700-ff15f880-7c2b-11eb-9461-7ba4df26d1fc.mp4
![image](https://user-images.githubusercontent.com/50388568/109763622-4cdf3080-7c2d-11eb-9e68-8231b53d3f2e.png)

MUP姿态跟踪项目是2019下半年做的一个项目。项目一开始是从STM32单片上开始着手的，参考了不少飞控算法/姿态计算的开源。因此之前着手过几个SMPU6050的姿态应用项目，所以一开始还是很有信心的。后来的我才发现，经验和现实还是有点差距的。过往的项目应用场景对姿态的准确性的容错率较大，因此很少深入研究姿态结算的内容。但是这个项目对姿态数据的准确性要求较高，项目初期因为MPU的零漂/姿态校正等问题摸爬滚打了很久。
Main Idea
* 本项目是基于MPU9250/MPU6050的姿态结算和显示，采用matlab和STM32交互实现。STM32主要负责通过蓝牙串口回传速度、加速度和陀螺仪数据（详细可以参考正点原子、野火的官方demo）。matlab通过串口接收STM32发送的姿态数据，同时实现Mahony姿态解算，椭圆/圆形/最值姿态初始校正、二维/三维姿态实时跟踪/仿真。（注意，整个项目的数据源依赖于串口实现，即使是采用软件仿真模式也需要模拟串口设备，可通过通过仿真/外接串口，并将发送和接收端子短接。）

![image](https://user-images.githubusercontent.com/50388568/109765193-7731ed80-7c2f-11eb-9ec9-1ce0c6c68d67.png)
![image](https://user-images.githubusercontent.com/50388568/109766704-7e59fb00-7c31-11eb-8df1-7b630a705b8b.png)
![image](https://user-images.githubusercontent.com/50388568/109766085-ad23a180-7c30-11eb-903e-4574885ff85e.png)

* 考虑Matlab在数据可视化/分析上的优势，本项目的将姿态结算和校正的核心算法全部移植到了Matlab上。但是Matlab在姿态结算和显示数据的效率上无法同C/C++进行比较。姿态实时跟踪是Matlab上实现的首要考虑的问题。考虑到姿态结算和数据显示跟踪会打断Matlab串口数据，本项目采用竞争方式进行数据的显示和跟踪。这种方式保证了在不指定通信协议/检验的情况下，串口数据不会发生错位导致异常更新。同时，Matlab显示界面和计算只更新当前最新姿态数据，不考虑历史数据。虽然这种方式带来了一定的数据丢失，可能导致姿态丢失无法回调，但是提高姿态跟踪的算法效率。

![demo](https://user-images.githubusercontent.com/50388568/109759148-8f9e0a00-7c27-11eb-8059-3cb549e7d08d.png)

* 本项目更加倾向于学习和掌握MPU姿态校正和跟踪算法，一个仿真和算法验证平台，暂时不适用于实时快速跟踪（需要的话，可以移植到STM32上实现/或采用C++/C开发实时视频接口/或采用更加高速的通信接口）。
* 后面有时间还会更新一个二阶卡尔曼滤波的修正版本，还没有整理出来，期待吧！
* 姿态结算学习可以参考:
  MPU9250九轴姿态解算开发小结_Lynnllh的博客-CSDN博客 https://blog.csdn.net/weixin_38492491/article/details/78799638
  正点原子 十轴/六轴 IMU加速度气压陀螺仪-正点原子官网|广州市星翼电子科技有限公司 http://www.alientek.com/productinfo/503267.html
* 板子的话，可以参考Mini AHRS的方案：

* ![image](https://user-images.githubusercontent.com/50388568/109764075-00e0bb80-7c2e-11eb-9c37-8deb32f969ed.png)

* 或者正点原子/野火的，都可以。
