# 无人机双天线流动站 RTK 后处理 (RTKLIB 2.4.3 b34)

## 分发:

**推荐:C++ 单一封装程序(只需 3 个文件)**

| 文件             | 作用                                                            |
| -------------- | ------------------------------------------------------------- |
| `rtkproc.exe`  | 封装程序:路径归一化 + 调用解算 + 转换/方差膨胀,一步到位(**静态链接,只依赖 Windows 系统 DLL**) |
| `rnx2rtkp.exe` | RTKLIB 解算内核(同样只依赖系统 DLL)                                      |
| `rtk.conf`     | 解算配置                                                          |

三个文件放同一目录即可。**对方机器要求:Windows x64,无需装任何运行库 / Python / PowerShell 脚本。**

用法(数据文件可在任意目录,正/反斜杠、绝对/相对路径都行——封装内部自动归一化):

```
rtkproc.exe <流动站obs> <基站obs> <混合星历nav> [nav2 ...]
            [-o out.txt] [-c rtk.conf]
            [--base-avg | --base-header | --base-llh 纬 经 高]
            [--floor-h 0.010] [--floor-v 0.015] [--scale 1.0] [--keep-pos]
```

例:

```
rtkproc.exe data\rover.26O data\base.26o data\rover.26P data\base.26p
```

默认输出 `rtk_std_analysis.txt`(生成在流动站文件所在目录),默认已带 1.0cm/1.5cm 协方差地板;
设 `--floor-h 0 --floor-v 0` 则输出原始 RTKLIB std。源码见 `rtkproc.cpp`,
编译:`g++ -O2 -static -static-libgcc -static-libstdc++ -o rtkproc.exe rtkproc.cpp`。

### 基站坐标模式(影响绝对精度,不影响相对轨迹)
| 选项 | 含义 | 绝对精度 |
|------|------|---------|
| `--base-avg` **(默认)** | 对**整个基站文件**做单点定位(SPP)取均值,自动注入。**不限历元数**,任意批次时长通用 | ~1 m,且批次间可复现 |
| `--base-header` | 直接用基站 RINEX 头 APPROX POSITION(单历元粗略 SPP) | ~1~5 m,批次间随机 |
| `--base-llh 纬 经 高` | 用已知/CORS/PPP 精确坐标 | 厘米~分米 |

> 改基站坐标只是把整条轨迹**刚性平移**(实测:换坐标后各历元位移抖动仅 1~3 cm),
> 相对轨迹、速度、姿态、内部几何完全不变。
> **多期/多架次工程**:若是同一个物理基站点,建议先测一次精确坐标,所有批次统一用
> `--base-llh` 同一坐标,保证各期共享同一绝对基准、能相互对齐。

## 数据

| 角色                               | 文件                             |
| -------------------------------- | ------------------------------ |
| 基站观测                             | `data/_3601810.26o`            |
| 基站星历(混合)                         | `data/_3601810.26p`            |
| 流动站观测(天线1)                       | `data/20260630104518_181c.26O` |
| 流动站星历(混合, 含 GPS/GLO/GAL/BDS/QZS) | `data/20260630104518_181c.26P` |

时间重叠段: GPS 周内秒 183057 ~ 184295 (2026-06-30 02:50:57 ~ 03:11:35 GPST), 1 Hz, 共 1236 历元。



## 手动运行(等价于 process.bat 内部做的事)

> ⚠️ 必须**在数据文件所在目录下**运行 exe。RTKLIB 的 `expath()` 在 Windows 下只按反斜杠 `\`
> 拆分路径,用正斜杠或从别处传绝对路径会丢掉目录导致 "no obs data"。process.bat 已自动处理。

```
cd data
..\proc\rnx2rtkp.exe -k ..\proc\rtk.conf 20260630104518_181c.26O _3601810.26o 20260630104518_181c.26P _3601810.26p -o rover.pos
powershell -ExecutionPolicy Bypass -File ..\proc\pos2std.ps1 rover.pos rtk_std_analysis.txt 0.010 0.015 1.0
```

## rtk_std_analysis.txt 的 13 列含义 (已从 src/solution.c 核实)

| 列     | 含义                                      |
| ----- | --------------------------------------- |
| 1     | GPS 周内秒 (SOW)                           |
| 2-4   | 纬度(deg)、经度(deg)、椭球高(m)                  |
| 5-7   | 位置标准差 sdn, sde, sdu (m)                 |
| 8-10  | 速度 vn, ve, vu (m/s) — 需 `out-outvel=on` |
| 11-13 | 速度标准差 sdvn, sdve, sdvu (m/s)            |

即：把 RTKLIB `.pos`(llh + 速度输出)裁掉 Q、ns、协方差、age、ratio 等列。

## 关键配置 (proc/rtk.conf)

- `pos1-posmode=kinematic` 动态
- `pos1-frequency=l1+2+3` (L1/L2/L5)
- `pos1-soltype=combined` 前后向组合(让首历元也收敛/固定)
- `pos1-navsys=61` GPS+GLO+GAL+QZS+BDS
- `pos2-armode=continuous` 连续模糊度固定
- `ant2-postype=rinexhead` 基站坐标用 RINEX 头 APPROX POSITION
- `stats-errdoppler=2` 影响速度标准差量级

## 与参考文件 rtk_std_analysis.txt 的对比 (全 1236 历元)

- 平面差 RMS ≈ 5~12 cm, 高程 RMS ≈ 8 cm, 平面平均差 9 cm。
- 位置量级完全一致, 验证流程正确。
- 位置标准差(5-7列)本方案约为参考文件的 ~1/10:
  参考文件用了更大的观测噪声模型(或来自接收机自带 RTK 引擎)。
  如需让标准差数值也贴近参考, 调大 `stats-errphase`(如 0.006~0.02)。

## 说明

- 双天线: 你说两根天线在同一 .26O 文件。本流程按单天线(天线1 对 基站)解算即得到
  与参考文件一致的单条轨迹。第二根天线若要做测姿/航向, 需要其独立观测流做
  moving-base(`posmode=movingbase`), 那是另一步。
- 基站/天线相位中心未加 ATX 改正, 是残余几 cm 系统差的主要来源。
