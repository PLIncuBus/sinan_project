## 代码架构

代码主要构建在User目录下。代码层次为如下。

### 0-MWL(Middleware Layer)

主要是算法中间件，包括CRC校验、PID、自定义的一些数学库。

### 1-APL(Application Layer)

代码最顶层，包含轮询执行的函数，和中断回调函数。

### 2-FML(Function Module Layer)

包括底盘、云台、机械臂三个功能模块的类。相关控制函数在类中封装。

### 3-HDL(Hardware Driver Layer)

底层传感器、执行器等驱动代码，包括控制电机的代码、解析电机数据的代码、读取陀螺仪的代码等。每一个硬件驱动用类来封装。

### 4-HAL(Hardware Abstract Layer)

自己对stm32 HAL库中的CAN、UART等接口进行再封装。
