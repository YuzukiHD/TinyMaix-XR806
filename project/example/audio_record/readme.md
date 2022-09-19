# audio record 示例工程

> audio record示例工程展示了XRadio SDK中进行音频录制的代码实现方法。
>
> 本工程中提供以下录音方式的示例：
> 1. 录制amr音频到sd卡
> 2. 录制pcm音频到sd卡
> 3. 以自定义保存地址的方式录制amr音频

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
> * CONFIG_XPLAYER：必选项，配置使用音频播放功能
>
> Makefile：
> * N/A
>
> board_config.h
> * N/A
>
> board_config.c
> * N/A
>
> prj_config.h
> * PRJCONF_SOUNDCARD0_EN: 可选项，配置使用外部声卡
> * PRJCONF_SOUNDCARD2_EN: 必选项，配置使用内部声卡

## 模块依赖

> 必选项
> 1. libcedarx.a： 音频核心模块
> 2. libreverb.a： 音频混响核心模块

> 可选项
>
> 1. libamren.a： 录制amr音频需要的编码库

> 音频的数据流、编码格式可根据需求选择，选择说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki/wiki/dev-about-cedarx-encoder-config

---

## 工程说明

> 本工程的功能为录制amr音频到虚拟文件系统vfs、录制pcm音频到虚拟文件系统vfs、录制amr音频到自定义地址。

### 操作说明

> 1. 创建一个“record”的文件夹。通过文件系统工具将该文件夹打包烧写到flash对应的地址。文件系统工具的使用方式，请参考《XR806_文件系统工具_使用指南》
> 2. 编译工程，烧录镜像，启动即可
> 3. 系统启动后，即会自动进行录音

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki


### 代码结构
```
#本工程
.
├── gcc
│   ├── defconfig               # 本工程的配置选项，主要用于覆盖默认全局配置
│   └── Makefile                # 本工程的编译规则，如ld文件、image.cfg、board_config.h等文件指定，可覆盖默认配置
├── main.c                      # 本工程的入口，完成平台初始化和音频录制
├── ExampleCustomerWriter.c     # 本工程的自定义保存地址实现方式
├── ExampleCustomerWriter.h
├── prj_config.h                # 本工程的配置选项，主要用于功能的选择。
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

> 1. main()入口： 平台初始化，调用录音函数。
> 2. audio_record()函数：
> A）完成录音准备工作，如sd卡初始化
> B）完成录音器创建，即recorder_create()
> C）录制amr音频，保存到sd卡
> D）录制pcm音频，保存到sd卡
> E）使用自定义的音频数据保存方式，录制amr音频

---


## 常见问题

> 问：使用cedarx无法录制两声道音频

答：暂不支持两声道音频录制。请直接使用音频驱动进行录音

> 问：使用cedarx无法录制高于32000采样率的pcm音频

答：暂不支持较高采样率的pcm音频录制。请直接使用音频驱动进行录音

## 参考文档

> 文档资源

无

> WiKi资源

1. https://github.com/XradioTech/xradiotech-wiki/wiki/dev-about-cedarx
