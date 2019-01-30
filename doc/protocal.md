# 与上位机交互协议

> 注意：协议内的数据都是大端序，使用之前需要转换为本地序

版本：draft

协议格式说明：

```c
typedef struct tagXXX
{
    unsigned char cmd;	//目前评估256条命令还是够用的
    unsigned char type;	//用于指示版本号，方便后续不同版本协议前向兼容，第一个版本是0
    unsigned short len; //从len 到chksum(不含自己)，len = sizeof(DATA) + sizeof(chksum);
    DATA data;			//数据体,后续扩展协议请注意要4字节对齐
    unsigned long chksum;
}XXXX_S；
```

| byte0 | byte1 | byte2    | byte3   | byte 4 ...byte n | byte n+1 | byte n+2 | byte n+3 | byte n+4 |
| :---: | ----- | -------- | ------- | ---------------- | -------- | -------- | -------- | -------- |
|  cmd  | type  | len_high | len_low | data             | chk31~24 | chk23~16 | chk15~8  | chk7-0   |

## 数据上报

1. 8个红外数据上报

   ```c
   cmd = 0x10;
   type = 0;
   typedef struct tagLaserData
   {
   	unsigned short value[MAX_LASER_NUM];  //MAX_LASER_NUM 为 8
   }LASER_DATA_S;
   ```

2. 3个超声波传感器数据

   ```c
   cmd = 0x11;
   type = 0;
   
   typedef struct tagUntrasonicWaveData
   {
       unsigned short value[3];
   }UNTRASONIC_WAVE_DATA_S;
   
   ```

3. 陀螺仪

   ```c
   cmd = 0x12;
   type = 0;
   
   typedef struct tagIMU
   {
       unsigned short ax;
       unsigned short ay;
       unsigned short az;
   }IMU_S;
   
   ```

4. 电源状态

   ```c
   cmd = 0x13;
   type = 0;
   typedef struct tagBatteryStatus
   {
       unsigned char charge; //1 表示充电， 0表示不在充电
       unsigned char lv; //电量百分比
   }BATTERY_STATUS_S;
   ```

5. 里程机故障上报

   ```c
   cmd = 0x14;
   type = 0;
   typedef struct tagMotorStatus
   {
   	unsigned char overCurrent; //过流 1表示异常， 0表示正常
       unsigned char overVoltage; //过压 同上
       unsigned char codecErr; //编码器故障
       unsigned char underVoltage; //欠压
       
       unsigned char overload; //过载
       unsigned char rsv; //保留位，用于4字节对齐
       unsigned short voltage; //母线电压，误差2V
       
       unsigned short current; //输出电流
       unsigned short rpm; //转速
       
       unsigned long postion_set; //设定的位置
       unsigned long postion_current; //反馈位置
       
   }MOTOR_STATUS_S;
   ```

6. 预留的触摸板数据

   ```c
   cmd = 0x15;
   type = 0;
   typedef struct tagTouchData
   {
       //暂时未知，预留
   }TOUCH_DATA_S;
   ```

7. 里程计相关数据上报

   ```c
   cmd = 0x16;
   type = 0;
   
   typedef struct tagOdometryData
   {
       float x_pos; //当前位置x 单位:mm
       float y_pos; //当前位置y 单位：mm
       float theta; //角度 theta 单位:rad
       float line_speed; //线速度   单位：mm/s
       float angle_speed; //角速度   单位：rad/s
   }ODOMETRY_DATA_S;
   ```



## 命令下发(上位机发给下位机)

1. 查询红外数据

   ```c
   //0x10 ~ 0x2f 预留给 下位机给上位机发送
   cmd = 0x30；
   type = 0;
   
   typedef struct tagQueryLaser
   {
       unsigned char rsv[4]; //全部填0就可以，通过cmd就能辨别，这个预留位给后续扩展用
   }
   
   ```

2. 查询超声波数据

   ```c
   cmd = 0x31;
   type = 0;
   
   typedef struct tagQueryUltraSonic
   {
       unsigned char rsv[4]; //全部填0就可以，通过cmd就能辨别，这个预留位给后续扩展用
   }QUERY_ULTRASONIC_S
   
   ```

3. 查询陀螺仪

   ```c
   cmd = 0x32;
   type = 0;
   typedef struct tagQueryIMU
   {
       unsigned char rsv[4]; //全部填0就可以，通过cmd就能辨别，这个预留位给后续扩展用
   }QUERY_IMU_S
   ```

4. 设置里程计

    ```c
    cmd = 0x33;
    type = 0;
    typedef struct tagSetMotor
    {
        unsigned long line_speed; //设置线速度， 如果只想改变其中一项，另一项发全F就可以
    	unsigned long angle_speed; //设置角速度
    }SET_MOTOR_S
    ```

5. 查询里程计状态

    ```c
    cmd = 0x34;
    type = 0;
    typedef struct tagQueryMotor
    {
        unsigned char rsv[4]; //全部填0就可以，通过cmd就能辨别，这个预留位给后续扩展用
    }QUERY_MOTOR_S;
    ```

6. 设置里程计原点

   ```c
   cmd = 0x35;
   type = 0;
   
   typedef struct tagSetOdometryOrigin
   {
       
   }SET_ODOMETRY_ORIGIN_S;
   
   ```

7. 查询软件版本号

8. 复位并进入bootload

    ```c
    cmd  = 0x50;
    type = 0;
    
    typedef struct tagReset
    {
        unsigned long rsv;
    }RESET_S;
    
    ```

9. 

## 关于网络序编程说明

如果有使用了位域定义结构体需要用[宏来隔开](http://note.youdao.com/noteshare?id=cc7422fffb927ce70e15958c96fedc50&sub=4C7BE93389A843499E98DB9084137112)，本协议暂时不涉及用位域来代表数据。我们这里只是在接受和发送之前需要做一次网络序和本地序的转换

```c
//就拿红外数据上报举例
typedef struct tagLaserData
{
	unsigned short value[8]; //8个红外传感器的数据
}LASER_DATA_S;

void LaserDataConvert(const LASER_DATA_S *pstIn, LASER_DATA_S *pstOut)
{
    for (int i = 0; i < MAX_LASER_NUM; i++)
    {
        pstOut->value[i] = PUB_HTONS(pstIn->value[i]); //PUB_HTONS宏已经实现，在pub.h
    }
}

//pstOut就是转换完大小端之后的格式了

```

## 下位机各个模块如何使用协议

1. 协议处理模块会处理UART 接收消息，并对消息长度，chksum进行校验
2. 各个模块需要注册自己需要处理的消息
3. 协议处理模块会回调已经注册的对应的函数
4. 协议发送模块有统一的发送模块，会计算好chksum