<p align="center">
<img src="documentation/figures/logo.png" width="60%" >
</p>
[English](README.md) | **中文** 

## 简介

RT-Thread诞生于2006年，是一款以开源、中立、社区化发展起来的物联网操作系统。
RT-Thread主要采用 C 语言编写，浅显易懂，且具有方便移植的特性（可快速移植到多种主流 MCU 及模组芯片上）。RT-Thread把面向对象的设计方法应用到实时系统设计中，使得代码风格优雅、架构清晰、系统模块化并且可裁剪性非常好。

RT-Thread有完整版和Nano版，对于资源受限的微控制器（MCU）系统，可通过简单易用的工具，裁剪出仅需要 3KB Flash、1.2KB RAM 内存资源的 NANO 内核版本；而相对资源丰富的物联网设备，可使用RT-Thread完整版，通过在线的软件包管理工具，配合系统配置工具实现直观快速的模块化裁剪，并且可以无缝地导入丰富的软件功能包，实现类似 Android 的图形界面及触摸滑动效果、智能语音交互效果等复杂功能。

## **RT-Thread架构**

RT-Thread是一个集实时操作系统（RTOS）内核、中间件组件的物联网操作系统，架构如下：

![architecturezh](./documentation/figures/architecturezh.png)



- 内核层：RT-Thread内核，是 RT-Thread的核心部分，包括了内核系统中对象的实现，例如多线程及其调度、信号量、邮箱、消息队列、内存管理、定时器等；libcpu/BSP（芯片移植相关文件 / 板级支持包）与硬件密切相关，由外设驱动和 CPU 移植构成。

- 组件与服务层：组件是基于 RT-Thread内核之上的上层软件，例如虚拟文件系统、FinSH命令行界面、网络框架、设备框架等。采用模块化设计，做到组件内部高内聚，组件之间低耦合。


- RT-Thread软件包：运行于 RT-Thread物联网操作系统平台上，面向不同应用领域的通用软件组件，由描述信息、源代码或库文件组成。RT-Thread提供了开放的软件包平台，这里存放了官方提供或开发者提供的软件包，该平台为开发者提供了众多可重用软件包的选择，这也是 RT-Thread生态的重要组成部分。软件包生态对于一个操作系统的选择至关重要，因为这些软件包具有很强的可重用性，模块化程度很高，极大的方便应用开发者在最短时间内，打造出自己想要的系统。RT-Thread已经支持的软件包数量已经达到450+。



## RT-Thread的特点

- 资源占用极低，超低功耗设计，最小内核（Nano版本）仅需1.2KB RAM，3KB Flash。

- 组件丰富，繁荣发展的软件包生态 。

- 简单易用 ，优雅的代码风格，易于阅读、掌握。

- 高度可伸缩，优质的可伸缩的软件架构，松耦合，模块化，易于裁剪和扩展。

- 强大，支持高性能应用。

- 跨平台、芯片支持广泛。


## **代码目录**

RT-Thread源代码目录结构如下图所示：

| 名称          | 描述                                                    |
| ------------- | ------------------------------------------------------- |
| bsp           | Board Support Package（板级支持包）基于各种开发板的移植 |
| components    | RT-Thread 的各个组件代码，例如 finsh，gui 等。          |
| documentation | 相关文档，如编码规范等                                  |
| examples      | 相关示例代码                                            |
| include       | RT-Thread 内核的头文件。                                |
| libcpu        | 各类芯片的移植代码。                                    |
| src           | RT-Thread 内核的源文件。                                |
| tools         | RT-Thread 命令构建工具的脚本文件。                      |

目前RT-Thread已经针对将近90种开发板做好了移植，大部分 BSP 都支持 MDK﹑IAR开发环境和GCC编译器，并且已经提供了默认的 MDK 和 IAR 工程，用户可以直接基于这个工程添加自己的应用代码。 每个 BSP 的目录结构高度统一，且都提供一个 README.md 文件，包含了对这个 BSP 的基本介绍，以及相应的说明，方便用户快速上手。

Env 是RT-Thread推出的开发辅助工具，针对基于RT-Thread操作系统的项目工程，提供编译构建环境、图形化系统配置及软件包管理功能。其内置的 menuconfig 提供了简单易用的配置剪裁工具，可对内核、组件和软件包进行自由裁剪，使系统以搭积木的方式进行构建。


## **支持的 IDE 和编译器**

RT-Thread主要支持的IDE/编译器包括：

- MDK KEIL

- IAR

- Gcc

- RT-Thread Studio

使用基于 Python 的 [scons](http://www.scons.org/) 进行命令行生成。





## **快速上手**

RT-Thread BSP可以直接编译并下载到相应的开发板使用。此外，RT-Thread还提供 qemu-vexpress-a9 BSP，无需硬件平台即可使用。有关详细信息，请参阅下面的入门指南。

[QEMU 入门指南(Windows)](documentation/quick-start/quick_start_qemu/quick_start_qemu.md)

[QEMU 入门指南(Ubuntu)](documentation/quick-start/quick_start_qemu/quick_start_qemu_linux.md)

## XMC7200EVK例程

[rtthread-xmc7200/bsp/Infineon/xmc7200-kit_xmc7200_evk at jasontek_xmc7200 · Jason0204/rtthread-xmc7200 (github.com)](https://github.com/Jason0204/rtthread-xmc7200/tree/jasontek_xmc7200/bsp/Infineon/xmc7200-kit_xmc7200_evk)



# **许可协议**

RT-Thread 系统完全开源，遵循 Apache License 2.0 开源许可协议，可以免费在商业产品中使用，并且不需要公开私有代码，没有潜在商业风险。

```
/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */
```


