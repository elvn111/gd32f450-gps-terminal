# tests —— PC 端回归

不依赖 Keil/ARM 工具链，用 **gcc** 与 **python** 在 PC 上验证三件容易出错的东西：
语句解析、协议样例帧、协议文档本身。

```bash
./run_tests.sh        # Linux / MSYS / Git Bash
run_tests.cmd         # Windows cmd（需要 gcc 与 python 在 PATH）
```

| 文件 | 作用 |
|---|---|
| `test_info_bsp.c` | 解析器回归：`$GPYBM` / `$GPNAV` 的样例、反例、边界，共 47 项断言。直接编译仓库里的 `../Template/info_bsp.c`，所以验的是**真源码** |
| `tools/check_frames.py` | 按协议文档的字段定义重建样例帧（`0x90/0x01`、`0x90/0x02`），校验长度、CRC-16/MODBUS、帧尾与字段拆解；再解析协议文档，校验其中每个十六进制块的帧长与 CRC |
| `tools/mkframes.py` | 生成协议文档需要的全部样例帧（上报、请求、应答、事件），每帧打印十六进制并逐帧自校验 |
| `tools/check_doc.py` | 校验整份协议文档：按“帧头 + 长度 N”定帧，逐帧检查帧头/长度/CRC/帧尾；检查表号是否从 1 连续编号；列出正文里的校验码引用 |
| `tools/md2html.py` | 协议 markdown → HTML（封面居中、标题分级、自动生成目录、管道表格转 `<table>`、代码块转 `<pre>`），供 WPS/Word 导入生成 `.docx` |

`check_frames.py` 也可以单独用作对接工具：

```bash
python tools/check_frames.py                          # 只重建并打印样例帧
python tools/check_frames.py ../Template/GD32F450_串行接口协议规范.md   # 同时校验文档
```

文档回归（写完/改完协议文档后跑一遍，样例帧错了立刻报出来）：

```bash
python tools/check_doc.py "../Template/时变TXP系列定位终端通信协议集V1.0.0_20261016.md"
```

生成 Word 版协议文档：

```bash
python tools/md2html.py "../Template/时变TXP系列定位终端通信协议集V1.0.0_20261016.md" proto.html
# 再用 WPS/Word 打开 proto.html 另存为 .docx（WPS 的 h1~h4 会映射成“标题 1~4”，自动带出大纲）
```

接收端固件可以用同一套样例做上电自检：解码结果应与协议文档表格逐项一致。
