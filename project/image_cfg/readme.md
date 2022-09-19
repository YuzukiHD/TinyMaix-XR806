# XRADIO MKimage & CFG File User Guide

> MKimage（打包工具）是XRadio芯片将编译生成的各个bin文件打包成一个image的工具，在各个系统平台均有对应版本的工具。
> MKimage的使用需要搭配一个对应的CFG文件，该文件使用Json语法标记了需要生成的image以及各个bin文件的信息。
> 本工程中提供以下常用工程配置和系统初始化流程的演示：

---

## CFG文件

> CFG文件包含整个image的组织结构，配置文件使用Json标记语言对bin文件的头部信息进行配置。

### 文件字段
> 如下是一个完整的cfg文件示例。

```
{
    "magic"   :"AWIH",
    "version" :"0.5",
    "extern":{"chk_ota_area":"false","add_ota_area":"false","verify":"CRC32"},
    "image"   :{"max_size": "1020K", "bin_num":6},
    "OTA"     :{"addr": "1024K", "size": "32K"},
    "count"   :6,
    "section" :[
        {"id": "0xa5ff5a00", "bin": "boot_40M.bin", "cert": "null", "flash_offs": "0K", "max_len": "32K", "sram_offs": "0x00268000", "ep": "0x00268101", "attr": "0x1"},
        {"id": "0xa5fe5a01", "bin": "app.bin", "cert": "null", "flash_offs": "32K", "max_len": "48K", "sram_offs": "0x00201000", "ep": "0x00201101", "attr": "0x1","priv":[{"0":"0x1402000"}, {"2":"0x1430000"}]},
        {"id": "0xa5fd5a02", "bin": "app_xip.bin", "cert": "null", "flash_offs": "80K", "max_len": "800K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x2"},
        {"id": "0xa5fa5a05", "bin": "wlan_bl.bin", "cert": "null", "flash_offs": "970K", "max_len": "10K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x1"},
        {"id": "0xa5f95a06", "bin": "wlan_fw.bin", "cert": "null", "flash_offs": "980K", "max_len": "36K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x1"},
        {"id": "0xa5f85a07", "bin": "sys_sdd_40M.bin", "cert": "null", "flash_offs": "1016K", "max_len": "4K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x1"},
		{}
	],
    "raw_bin" :[
		{"bin" :"boot.bin",	"flash_offs": "901K"},
		{"bin" :"sys_sdd.bin",	"flash_offs": "1500K"},
		{}
	]
}
```

各字段含义如下，**加粗字段**为必选字段：
　　**magic** - 固件魔数，表头标识，固定为 AWIH
　　**version** - 版本号
　　extern - 额外参数，非必选项，具体参数如下
　　	chk_ota_area : 默认为true，设置为false时，检查bin文件重叠时不会与OTA区域对比，适用于将某些bin放在flash最后的情况
　　	add_ota_area : 默认为false，设置为true时，打包时候会添加OTA区域，适用于客户使用flash烧写器直接烧写生成的image的情况
　　	verify : 在image的最后添加校验数据（none/CRC32/MD5/SHA1/SHA256)，在OTA升级时会校验，默认为none
　　**image** - image的大小参数
　　	**max_size** : image的最大值（image1和image2区域的最大值）
　　	xz_max_size : ota压缩模式下compressed image的最大值，非ota压缩模式下，禁止使用该字段。
　　	bin_num : 实际进行打包的bin文件个数，不能大于count值
　　**OTA** - ota参数存储地址和大小，需要4K对齐
　　	**addr** : OTA区域的起始地址，必须以4K对齐
　　	**size** : OTA区域的长度，必须以4K对齐
　　count - 打包文件列表中文件的数目，可以忽略不写
　　**section** - 文件列表
　　**id** - bin文件的id
　　	boot_id : 0xa5ff5a00
　　	app_id : 0xa5fe5a01
　　	app_xip_id : 0xa5fd5a02
　　	wlan_bl_id : 0xa5fa5a05
　　	wlan_fw_id : 0xa5f95a06
　　	sys_sdd_id : 0xa5f85a07
　　	app_psram_id : 0xa5f65a09
　　**bin** - bin文件名字
　　**cert** - 该段bin文件对应的证书文件，null表示没有添加证书
　　flash_offs - 该段bin文件存放在 FLASH 中的位置偏移，若不填写，则软件自动放在上一个文件结束的位置，以1024 byte对齐
　　max_len - 该段bin文件可存放的最大空间长度，若不填写，则软件按照文件大小自动计算，以1024 byte对齐
　　**sram_offs** - 该段固件加载到SRAM中的地址偏移，0xFFFFFFFF为无效值，使用时忽略
　　**ep** - 该段固件的ENTRY POINT，0xFFFFFFFF为无效值，使用时忽略
　　**attr** - 表示该段固件属性
　　	[1:0]：文件类型，00-普通文件，01-可执行文件，10-XIP文件，11-保留
　　	[3:2]：加密类型，00-不加密，X1-带证书，1X-flash加密
　　	[4]：是否压缩bin文件，0-不是，1-是
　　	[5]：是否安全文件，0-不是，1-是
　　	[31:6]：保留
　　priv - private数据，用户可以填入每个bin的自定义数据，但某些已被系统使用的字段会被覆盖
　　	boot.bin priv[0]: [7:0]-OTA flash num
　　	boot.bin priv[0]: [31:8]-OTA size
　　	boot.bin priv[1]: [31:0]-OTA address
　　	boot.bin priv[2]: [15:0]-image max size
　　	boot.bin priv[2]: [31:16]-image xz max size
　　	每个bin priv[5]: [31:0]-next safety bin address
　　raw_bin - 裸bin文件，打包时候需要增加 -r 参数才会将其打包
　　	bin - bin文件名字
　　	flash_offs - 该段固件存放在 FLASH 中的位置偏移

### Flash_offs配置

> flash offset的配置要求如下：
> 1. boot.bin固定需要设置为0地址，app.bin地址需要4K对齐，默认设置为32k的偏移地址
> 2. 其他的bin文件的地址并没有强制性的要求，只要没有互相重叠并且不覆盖到sysinfo和OTA区域，都是可以正常运行的

### Bin文件重叠的解决

> 如果bin文件出现了互相重叠的情况，MKimage工具会进行自动计算，然后生成一个“image_auto_cal.cfg”文件
> 用户可以直接使用该cfg文件重新进行打包，也可手动修改默认的image.cfg文件
> **注意:**如果全部的bin文件在经过自动计算后，其大小依然超过sysinfo的默认位置(OTA ADDRESS - 4K)，则不会生成“image_auto_cal.cfg”文件，此时，用户需要自己手动进行cfg文件的修改

```
cfg string:
{
    "magic" : "AWIH",
    "version" : "0.4",
    "OTA" : {"addr": "1024K", "size": "4K"},
    "image" : {"max_size": "1020K"},
    "count" : 6,
    "section" :
    [
        {"id": "0xa5ff5a00", "bin": "boot_40M.bin", "cert": "null", "flash_offs": "0K", "sram_offs": "0x00268000", "ep": "0x00268101", "attr": "0x1"},
        {"id": "0xa5fe5a01", "bin": "app.bin", "cert": "null", "flash_offs": "32K", "sram_offs": "0x00201000", "ep": "0x00201101", "attr": "0x1"},
        {"id": "0xa5fd5a02", "bin": "app_xip.bin", "cert": "null", "flash_offs": "50K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x2"},
        {"id": "0xa5fa5a05", "bin": "wlan_bl.bin", "cert": "null", "flash_offs": "980K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x1"},
        {"id": "0xa5f95a06", "bin": "wlan_fw.bin", "cert": "null", "flash_offs": "985K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x1"},
        {"id": "0xa5f85a07", "bin": "sys_sdd_40M.bin", "cert": "null", "flash_offs": "1017K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x1"},
        {}
    ]
}

err: bin 1 and bin 2 were overlaped!
Overlapped size: 2248 Byte(3kB)
bin 1 name:app.bin    begin: 0x00008000    end: 0x0000D0C8
bin 2 name:app_xip.bin    begin: 0x0000C800

We've rearranged bin files and generated new cfg file 'image_auto_cal.cfg', the new one is recommended.
Generate image file failed
make: *** [image] 错误 255
```

> 上图显示，app.bin和app_xip.bin重叠了，app.bin超出了3K的数据，覆盖了app_xip.bin前面3K数据
> 说明给app.bin的空间太小了，此时可以调整app_xip.bin的flash_offs，由原来的50K调整为80K即可，剩下的27K做为预留空间。

---

## 工具使用

### 参数说明

> MKimage使用命令行方式运行，除了windows平台下的EXE可以双击运行，此时使用的参数都是默认值
> 工具支持的参数说明如下，**加粗**的为常用参数：
|            参数            |                            说明                             | 
| -------------------------- | ----------------------------------------------------------- |
| **-c <cfg file path>**     | cfg文件的路径，默认为“image.cfg”                            |
| -C <image oper>            | image操作模式，默认为0<br>0-仅打包<br>1-仅合并（不含OTA区域，包含第二个文件的boot.bin）<br>2-打包并合并固件（不含OTA区域，包含第二个文件的boot.bin）<br>3-仅合并生成裸数据固件（包含OTA区域，不含第二个文件的boot.bin）<br>4-打包并生成裸数据固件（包含OTA区域，不含第二个文件的boot.bin） |
| -e <flash crypto>          | Flash加密使用，默认不使用                                   |
| -f <Combine File 1st>      | 生成合并固件所需的第一个文件路径，默认与打包输出的文件路径相同（即-o参数的值）|
| -F <Combine File 2nd>      | 生成合并固件所需的第二个文件路径，无默认值                  |
| -I <Combine File path>     | 生成合并固件的路径，无默认值                                |
| **-o <Output image path>** | 打包生成的文件路径，默认为“xr_system.img”                   |
| **-O**                     | 使能OTA功能，默认不使能                                     |
| -p                         | 使能OTA功能，并且将OTA区域写入生成文件，默认不使能          |
| -r                         | 打包时一并打入raw bin字段的文件，默认不打入                 |
| -h                         | 帮助信息                                                    |
| -v                         | 版本查看                                                    |

---

## 使用示例

### 打包生成使能OTA功能的固件

> 使用命令：
```
mkimage.exe -O
```
> 打印输出：
```
E:\project\IOT\tools\xr-mkimage\Release>mkimage.exe -O
cfg string:
{
    "magic" : "AWIH",
    "version" : "0.4",
    "OTA" : {"addr": "1024K", "size": "4K"},
    "image" : {"max_size": "1020K"},
    "count" : 6,
    "section" :
    [
        {"id": "0xa5ff5a00", "bin": "boot_40M.bin", "cert": "null", "flash_offs": "0K", "sram_offs": "0x00268000", "ep": "0x00268101", "attr": "0x1"},
        {"id": "0xa5fe5a01", "bin": "app.bin", "cert": "null", "flash_offs": "32K", "sram_offs": "0x00201000", "ep": "0x00201101", "attr": "0x1","priv":[{"0":"0x1402000"}, {"2":"0x1430000"}]},
        {"id": "0xa5fd5a02", "bin": "app_xip.bin", "cert": "null", "flash_offs": "80K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x2"},
        {"id": "0xa5fa5a05", "bin": "wlan_bl.bin", "cert": "null", "flash_offs": "980K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x1"},
        {"id": "0xa5f95a06", "bin": "wlan_fw.bin", "cert": "null", "flash_offs": "990K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x1"},
        {"id": "0xa5f85a07", "bin": "sys_sdd_40M.bin", "cert": "null", "flash_offs": "1016K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x1"},
        {}
    ]
}


generate image: xr_system.img

```


### 打包并生成合并固件

> 使用命令：
```
mkimage.exe -O -C 2 -F xr872_40m_etf-v1.0.0-rel-20200302.img -I xr_system.cimg
```
> 打印输出：
```
E:\project\IOT\tools\xr-mkimage\Release>mkimage.exe -O -C 2 -F xr872_40m_etf-v1.0.0-rel-20200302.img -I xr_system.cimg
cfg string:
{
    "magic" : "AWIH",
    "version" : "0.4",
    "OTA" : {"addr": "1024K", "size": "4K"},
    "image" : {"max_size": "1020K"},
    "count" : 6,
    "section" :
    [
        {"id": "0xa5ff5a00", "bin": "boot_40M.bin", "cert": "null", "flash_offs": "0K", "sram_offs": "0x00268000", "ep": "0x00268101", "attr": "0x1"},
        {"id": "0xa5fe5a01", "bin": "app.bin", "cert": "null", "flash_offs": "32K", "sram_offs": "0x00201000", "ep": "0x00201101", "attr": "0x1","priv":[{"0":"0x1402000"}, {"2":"0x1430000"}]},
        {"id": "0xa5fd5a02", "bin": "app_xip.bin", "cert": "null", "flash_offs": "80K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x2"},
        {"id": "0xa5fa5a05", "bin": "wlan_bl.bin", "cert": "null", "flash_offs": "980K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x1"},
        {"id": "0xa5f95a06", "bin": "wlan_fw.bin", "cert": "null", "flash_offs": "990K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x1"},
        {"id": "0xa5f85a07", "bin": "sys_sdd_40M.bin", "cert": "null", "flash_offs": "1016K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x1"},
        {}
    ]
}


generate image: xr_system.img

generate combine image: xr_system.cimg

```


### 打包并生成裸数据固件

> 使用命令：
```
mkimage.exe -O -C 4 -F xr872_40m_etf-v1.0.0-rel-20200302.img -I xr_system_raw.bin
```
> 打印输出：
```
E:\project\IOT\tools\xr-mkimage\Release>mkimage.exe -O -C 4 -F xr872_40m_etf-v1.0.0-rel-20200302.img -I xr_system_raw.bin
cfg string:
{
    "magic" : "AWIH",
    "version" : "0.4",
    "OTA" : {"addr": "1024K", "size": "4K"},
    "image" : {"max_size": "1020K"},
    "count" : 6,
    "section" :
    [
        {"id": "0xa5ff5a00", "bin": "boot_40M.bin", "cert": "null", "flash_offs": "0K", "sram_offs": "0x00268000", "ep": "0x00268101", "attr": "0x1"},
        {"id": "0xa5fe5a01", "bin": "app.bin", "cert": "null", "flash_offs": "32K", "sram_offs": "0x00201000", "ep": "0x00201101", "attr": "0x1","priv":[{"0":"0x1402000"}, {"2":"0x1430000"}]},
        {"id": "0xa5fd5a02", "bin": "app_xip.bin", "cert": "null", "flash_offs": "80K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x2"},
        {"id": "0xa5fa5a05", "bin": "wlan_bl.bin", "cert": "null", "flash_offs": "980K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x1"},
        {"id": "0xa5f95a06", "bin": "wlan_fw.bin", "cert": "null", "flash_offs": "990K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x1"},
        {"id": "0xa5f85a07", "bin": "sys_sdd_40M.bin", "cert": "null", "flash_offs": "1016K", "sram_offs": "0xffffffff", "ep": "0xffffffff", "attr": "0x1"},
        {}
    ]
}


generate image: xr_system.img

generate raw image: xr_system_raw.bin

```


## 参考文档

> 《XRADIO_MKimage_Tool_User_Guide-CN》
