# GD32F450 GPS 终端

GD32F450IK 上的 GPS 采集终端：**USART0** 作控制台与对外数据链路，**USART5** 接 K922 定位模块。
Keil MDK（ARMCC V5）+ FreeRTOS，C99 / MicroLIB。

## 功能

- **控制台命令解释器**（表驱动）：配置读写、诊断、GPS 操作、透明桥、Flash 持久化
- **K922 语句解析**：`$GPYBM` / `$GPNAV` 按逗号位置取字段，空字段判无效，校验和与字段数检查
- **下行链路**（发给第二颗 MCU / 上位机，与控制台共用 USART0）
  - 原报文模式：**一条完整语句一帧**（含它自己的 CR/LF）
  - 二进制模式：帧头 `A6 6A`、帧尾 `B7 7B`、CRC-16/MODBUS；定位数据帧 36 字节、终端信息帧 44 字节，"保最新"不排队
  - 终端信息帧（命令码 `0x01` / 功能码 `0x02`）：固件版本、上电时长、计数器、当前下行配置
- **上行**：`@` 前缀把模块命令直送 K922；`bridge on` 把终端变成双向透明管道（`+++` 退出）
- **配置掉电保存**：片内 Flash 追加日志，掉电最多丢最后一次保存

## 目录

| 路径 | 说明 |
|---|---|
| `Template/` | 本工程：应用源码、Keil/IAR/GD32EBuilder 工程、文档 |
| `Firmware/` | GD32F4xx CMSIS 与标准外设库（工程以 `..\..\Firmware` 相对路径引用） |
| `Utilities/` | GD32F450i 评估板 BSP 与官方预编译固件 |
| `Template/Keil5_project/Project.uvprojx` | 主工程（target `GD32F4XX`，输出 `Project.hex`） |
| `tests/` | PC 端回归测试（解析器 + 协议样例帧） |

应用源码清单：`main.c`（初始化/任务/平台钩子）、`cli.c/.h`（命令解释器）、
`gps_link.c/.h`（下行链路）、`info_bsp.c/.h`（语句解析）、`nvm_config.c/.h`（Flash 配置）、
`buff_config.h`（DMA 与缓冲配置）、`gd32f4xx_it.c`（中断）。

## 编译

1. Keil MDK 5 + ARMCC V5（工程已设 C99、MicroLIB）；
2. 打开 `Template/Keil5_project/Project.uvprojx` 编译，或命令行：
   ```
   UV4 -b Template\Keil5_project\Project.uvprojx -j0 -o build.log
   ```
3. 产物：`Template/Keil5_project/output/Project.hex`（`Project_merge.hex` 为合并后的烧录文件）。

## 上手

USART0：**115200-8-N-1，无流控**。

| 目的 | 操作 |
|---|---|
| 看有什么命令 | `help` |
| 看状态与计数 | `status`、`gstat`、`dma` |
| 让终端往外发定位 | **出厂默认已在发**（`out parsed` + `auto fix` + `rate 20`）；要停就 `out off` |
| 机机对接（干净数据流） | 出厂默认即干净（`verbose=0`、`out=parsed`）；想彻底无声再 `set echo 0` |
| 把命令直送定位模块 | 行首加 `@`，例如 `@log com1 gpnav ontime 1` |
| 手工调模块 | `bridge on` …静默 ≥1 s 后发 `+++` 退出 |

## 文档

| 文档 | 读者 |
|---|---|
| `Template/GD32F450_串行接口协议规范.md` | **对接方**：三处串行接口的字节级约定、帧格式与字段定标、时序语义、命令与配置、异常处理，附接入检查清单与验收测试 |
| `Template/GD32F450_命令系统说明.md` | 本机开发者：命令/配置参考、持久化、扩展指南与排障 |

## 测试

PC 端（需要 gcc，例如 MinGW）：

```bash
cd tests
./run_tests.sh          # Windows: run_tests.cmd
```

| 测试 | 覆盖 |
|---|---|
| `test_info_bsp.c` | 解析器回归：协议文档附录 F 的样例、反例与边界（47 项断言） |
| `tools/check_frames.py` | 重建协议文档里的全部样例帧（定位数据 ×4、终端信息 ×2），校验帧头帧尾、长度、CRC-16/MODBUS 与字段拆解 |

## 许可

`Template/` 下的应用代码由本项目维护；`Firmware/`、`Utilities/` 为 GigaDevice 官方库与例程，
版权与许可条款见 `SOFTWARE LICENSE AGREEMENT SLA-GD0001-version1.1.pdf` 及各文件头部声明。
