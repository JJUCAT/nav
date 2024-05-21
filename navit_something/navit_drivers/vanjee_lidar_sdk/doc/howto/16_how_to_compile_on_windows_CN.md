# 16 如何在Windows上编译vanjee_driver

## 16.1 概述

这里的编译说明，针对vanjee_driver的两个部分。

示例程序，包括demo_online和demo_pcap。

如下步骤在Windows 10系统下完成，使用的编译工具为VS2019。


## 16.2 编译demo_online

演示程序的编译，以demo_online为例说明。demo_pcap的编译步骤与demo_online相同。

## 16.2.1 安装第三方库

如果需要解析PCAP文件，则需要安装libpcap库，包括：
WpdPack_4_1_2.zip, 编译时需要的头文件和库文件
WinPcap_4_1_3.exe, 包括运行时库

将WpdPack_4_1_2.zip解压到目录C:/Program Files下。
运行WinPcap_4_1_3.exe，安装到目录C:/Program Files下。

下载Eigen3(https://eigen.tuxfamily.org/index.php?title=Main_Page#Download)
解压eigen-3.4.0.zip到目录C:/Program Files下。

## 16.2.2 创建demo_online工程

创建demo_online工程，加入源文件demo_online.cpp。

## 16.2.3 配置demo_online工程
遵循C++14标准 
  工程属性->配置属性->c++语言标准(ISO C++14标准)

demo_online依赖vanjee_driver库。设置vanjee_driver的头文件路径和Eigen路径。
  工程属性->C/C++->常规->附加包含路径(../../src)
  工程属性->C/C++->常规->附加包含路径(C:/Program Files/eigen-3.4.0)

设置libpcap库的头文件路径。
  工程属性->配置属性->VC++目录->包含目录(C:/Program Files/WpdPack_4_1_2/WpdPack/Include)

设置libpcap库的库文件路径。
  工程属性->配置属性->VC++目录->库目录(C:/Program Files/WpdPack_4_1_2/WpdPack/Lib/x64)

设置依赖的libpcap库wpcap.lib。也同时设置ws2_32.lib，这是Windows的socket库，vanjee_driver依赖它。
  工程属性->链接器->输入->附加依赖项(Ws2_32.lib;wpcap.lib)

设置编译选项 _CRT_SECURE_NO_WARNINGS，避免不必要的编译错误。
  工程属性->C/C++->预处理器->预处理器定义(_CRT_SECURE_NO_WARNINGS)

若工程提示大量未定义变量可尝试设置附加选项 添加 utf-8
  工程属性->C/C++->所有选项->附加选项(/utf-8)

## 16.3 编译及运行

编译demo_online工程,设置启动项，并运行。
