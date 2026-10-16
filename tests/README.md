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
| `tools/md2docx.py` | 协议 markdown → `.docx`（自己拼 WordprocessingML，不经过 Word/WPS 的 HTML 导入）：封面单独一页、每个一级标题分页、目录域（打开后按 F9 出页码）、页脚“第 X 页 / 共 Y 页”、表格跨页重复表头、代码块等宽字体 |

`check_frames.py` 也可以单独用作对接工具：

```bash
python tools/check_frames.py                          # 只重建并打印样例帧
python tools/check_frames.py ../Template/GD32F450_串行接口协议规范.md   # 同时校验文档
```

文档回归（写完/改完协议文档后跑一遍，样例帧错了立刻报出来）：

```bash
python tools/check_doc.py "../Template/时变TXP系列定位终端通信协议集V1.0.0_20261016.md"
```

生成 Word 版协议文档（不依赖 Word/WPS，纯 python）：

```bash
python tools/md2docx.py "../Template/时变TXP系列定位终端通信协议集V1.0.0_20261016.md" out.docx 25
#   第三个参数（页数，可选）写进页脚 NUMPAGES 域的缓存值，只读预览器也能显示“共 N 页”
```

排版自带：封面一页、每章起新页、目录域（打开后 F9 即带页码）、页脚页码、表格表头跨页重复。

接收端固件可以用同一套样例做上电自检：解码结果应与协议文档表格逐项一致。
