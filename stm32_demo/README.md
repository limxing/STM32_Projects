
#### 点亮板上的C13 LED灯
  1. RCC->APB2ENR外设时钟使能寄存器（单独的寄存器记录需要使能的外设是那个寄存器）  
  //使能 GPIOC的外设寄存器，也就是让CPU处理这部分的, 设置为1  
  // RCC->APB2ENR |= 4; 4 -> 0000 0000 0000 0100 使能某个寄存器 第2为是GPIOA 所以使能4是使能A  
  SET_BIT(RCC->APB2ENR,RCC_APB2ENR_IOPCEN); 
  
  2. GPIO 工作模式
  // 0-7 是配置低位寄存器CRL 8-15 配置高位寄存器CRH 模式需要配置输入还是输入和速度，需要占用4位，32位只能配置8个PIN因此需要分高低位寄存器设置  
  // 设置 00 通用推挽输出 clear是清除因此设置为  
  CLEAR_BIT(GPIOC->CRH,GPIO_CRH_CNF13);  
  // 设置 11 输出模式最大速度  
  SET_BIT(GPIOC ->CRH,GPIO_CRH_MODE13);  
  3. 配置端口输出数据寄存器ODR(输入就是IDR) （就是端口Pin设置为0输出低电位 还是1输入高电位 ，一共32位，高位的16保留，低位的16一个位对应一个PIN）  
  // ODR 是需要修改的目标寄存器  
  // SET_BIT(GPIOC->ODR,GPIO_ODR_ODR13);//设置为1 为高电平，LED灯不亮，两端都是高电平  
  CLEAR_BIT(GPIOC->ODR,GPIO_ODR_ODR13); //复位设置为0，为低电平，LED灯亮
#### 理解BSRR
  1. 端口位设置/清除寄存器BSRR（Bit Set Reset R）,一一对应ODR ，BSRR设置为1那么对应ODR寄存器会设置为0，(CLEAR_BIT设置为0，SET_BIT设置为1)  
  
    //BSRR 分为BR （Bit Reset） 和BS （Bit set）  
    SET_BIT(GPIOC->BSRR,GPIO_BSRR_BR13); // 设置为1 对应ODR设置为0了低电位,  
    CLEAR_BIT(GPIOC->BSRR,GPIO_BSRR_BR13); //设置为0 对应ODR没有影响  

    SET_BIT(GPIOC->BSRR,GPIO_BSRR_BS13);// 设置为1 对应ODR设置为1了高电位  
    CLEAR_BIT(GPIOC->BSRR,GPIO_BSRR_BS13);//设置为0 对应ODR没有影响  
  2. 端口位清除寄存器BRR （Bit Reset）就是BSRR中的BR单独拿出来  
    SET_BIT(GPIOC->BRR,GPIO_BRR_BR13); //设置为 1 对应ODR设置为1同上  
    CLEAR_BIT(GPIOC->BRR,GPIO_BRR_BR13);//设置为0 对应ODR没有影响  
  
#### 端口配置锁定寄存器
 用于在规定时间内锁定配置（CRL,CRH）不能改变 0-15位对应端口 16位是全局锁 17~31保留
#### 中断
主程序被中断，去处理中断源程序
![](./images/中断系统_中断体系架构.png)
##### NVIC 嵌套向量中断控制器
中断来源：
1. 内核其他控件
  - 系统滴答定时器
  - 复位
  - 其他十种
2. 片上外设
  - 串口
  - 定时器
  - I2C
  - 其他
3. 外部中断 （从通用引脚传递进来GPIOx）
  GPIO -> AFIO引脚复用选择器 -(7个0..15引脚合并选择一个发送,一共16根线)-> EXTI(外部中断控制器EXTI0..7) -> NVIC -> CPU
4. NVIC中断优先级，抢占和响应优先级，占4位，值小的优先级高。先比较抢占，再比较响应，相同的话查找中断向量表，值小的先响应，抢占优先级一样的中断都被挂起则优先处理响应抢占优先级高的。  
NVIC配置寄存器，一共0~20，每个里面有四个，每个占8位，高位的4位有效。总共84个中断。 
NVIC对优先级分了5组，在程序中先对中断进行分组，而且只能分一次，多次分组只有最后一次生效。  
![](./images/中断系统_中断优先级分组.png)
不同的组对应抢占和响应的位数（一共四位），group是抢占优先级，sub是响应优先级。最简单只选择值为3的分组，只有抢占优先级，谁优先看中断向量表序号。  
##### 中断原理
![](./images/中断原理_nvic电路流程.png)
脉冲发生器直接交给处理器，上边NVIC是软件控制管理中断。  
上升沿、下降沿是电信号上下瞬间触发中断。  
##### 中断案例：检测按键按下


