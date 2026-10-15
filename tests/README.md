# tests —— PC 端回归

不依赖 Keil/ARM 工具链，用 **gcc** 在 PC 上验证两件容易出错的东西：语句解析、协议样例帧。

```bash
./run_tests.sh        # Linux / MSYS / Git Bash
run_tests.cmd         # Windows cmd（需要 gcc 与 python 在 PATH）
```

| 文件 | 作用 |
|---|---|
| `test_info_bsp.c` | 解析器回归：`$GPYBM` / `$GPNAV` 的样例、反例、边界，共 47 项断言。直接编译仓库里的 `../Template/info_bsp.c`，所以验的是**真源码** |
| `tools/check_frames.py` | 按协议文档的字段定义重建全部样例帧（`TYPE=0x01` ×4、`TYPE=0x02` ×2），校验长度、CRC-16/CCITT-FALSE 与字段拆解；再解析协议文档，校验其中每个十六进制块的帧长与 CRC |

`check_frames.py` 也可以单独用作对接工具：

```bash
python tools/check_frames.py                          # 只重建并打印样例帧
python tools/check_frames.py ../Template/GD32F450_串行接口协议规范.md   # 同时校验文档
```

接收端固件可以用同一套样例做上电自检：解码结果应与协议文档 §3.7 / §3.8 的表格逐项一致。
