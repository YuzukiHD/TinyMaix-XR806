# VFS示例工程

> VFS示例工程展示了VFS模块接口的使用方法。
>
> 本工程中提供以下模块接口使用的示例：
>
> 1. VFS文件的创建、打开与关闭。
> 2. VFS文件的读写。

---

## 适用平台

> 本工程适用以下芯片类型：
> 1. XR806系列芯片：XR806AF2L、XR806BF2L、XR806BM2I等

> 本工程适用以下评估板类型：
> 1. 底板：XR806AF2L_EVB、XR806B_EVB

> 本工程在基于XR806AF2L的“XR806AF2L-EVB_V1_0”板上测试通过。
> 若需要在其他适用芯片和评估板上运行本工程，请根据快速指南《XRadio_Quick_Start_Guide-CN》的提示进行相关配置修改。

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

## 工程配置

> defconfig：
>
> - CONFIG_FILESYSTEM=y
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

> 依赖于LittleFS/SPIFFS、Flash等

## 工程说明

> 本工程对VFS模块的文件开关、读写接口的使用进行介绍。过程是：打开文件、写文件、移动光标、读文件、移动光标、查看光标位置、继续写文件、关闭文件、重命名文件、打开文件、读文件、关闭文件。
>

### 操作说明：

> 1. 连接XR806AF2L-EVB_V1_0板，编译工程，烧录镜像，启动。
> 3. 系统启动后，可以看到文件的打开、写、读、移动光标、关闭、重命名等操作均成功。

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://github.com/XradioTech/xr806_sdk.git

### 代码结构
```
.
├── gcc
│   ├── defconfig               # 本工程的配置规则，用于覆盖默认配置
│   └── Makefile                # 本工程的编译选项，可覆盖默认配置
├── image
│   └── xr806
│       └── xr_system.img       # 本工程的镜像分区配置
├── main.c                      # 本工程的入口，进行VFS打开、读写与关闭示例说明
├── prj_config.h                # 本工程的配置规则
└── readme.md                   # 本工程的说明文档

#本程用到XR806 SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xr806_dig_ver          #本工程的板级配置目录
                ├── board_config.h     #本工程的板级配置
                └── board_config.c     #本工程的板级pin mux的配置。
```
### 代码流程

> 1. main()入口： 执行VFS文件开关、读写的操作示例。
> 

---

## 常见问题

> N/A

## 参考文档

> N/A
