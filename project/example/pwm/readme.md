# PWM 示例工程

> PWM示例工程展示了输出模式、捕获模式的使用方法。
>
> 本工程中提供以下模块接口使用的示例：
>
> 1. PWM0 循环输出模式
> 2. PWM1 捕获输入模式

---

## 适用平台

> 本工程适用以下芯片类型：
>
> 1. XR806系列芯片

> 本工程适用以下评估板类型：
> 1. 底板：XR806_EVB
> 2. 模组：XR806_MD01

> 本工程在基于"XR806_MD01"的“XR806_EVB”板上测试通过。
> 若需要在其他适用芯片和评估板上运行本工程，请根据快速指南《XRadio_Quick_Start_Guide-CN》的提示进行相关配置修改。

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://docs.xradiotech.com

## 工程配置

> defconfig：
>
> - N/A
>
> Makefile：
>
> - N/A
>
> board_config.h
>
> - N/A
>
> board_config.c
>
> - N/A
>
> prj_config.h
>
> - N/A

## 模块依赖

> N/A

## 工程说明

> 本工程对PWM 模块通道输出模式、捕获模式的使用进行介绍。
>
> PWM0(PA8)配置的是输出频率为1KHz，占空比50%的循环方波信号。PWM1(PA9)配置的是通道捕获模式。在测试时，需要将PA8、PA9用跳线相连接。

### 操作说明：

> 1. 编译工程，烧录镜像，启动。
> 3. 系统启动后，可以看到通道的捕获结果信息。
> 3. 捕获到的数值是PWM计数器的值，转换成对应的时间是T=1/F*C，其中F是PWM通道的工作时钟，C为计数器的值。

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

### 代码结构
```
.
├── gcc
│   ├── defconfig               # 本工程的配置规则，用于覆盖默认配置
│   └── Makefile                # 本工程的编译选项，可覆盖默认配置
├── image
│   └── xr806
│       └── image.cfg           # 本工程的镜像分区配置
├── main.c                      # 本工程的入口，进行PWM捕获信号的示例说明
├── prj_config.h                # 本工程的配置规则
└── readme.md                   # 本工程的说明文档

#本程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xr806_dig_ver          #在project/Kconfig默认指定使用xr806_dig_ver的板级配置
                ├── board_config.h     #本工程的板级配置，
                └── board_config.c     #本工程的板级pin mux的配置。
```
### 代码流程

> 1. main()入口： 执行PWM捕获信号的操作示例。
> 

> 更详细的开发指南请参考《XRADIO_PWM_Developer_Guide-CN.doc》

---

## 常见问题

> N/A

## 参考文档

> 文档资源

1. 《XRADIO_PWM_Developer_Guide-CN.doc》
