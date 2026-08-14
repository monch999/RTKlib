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
            [--floor-h 0.046] [--floor-v 0.075] [--scale 1.0] [--keep-pos]
```

例:

```
rtkproc.exe data\rover.26O data\base.26o data\rover.26P data\base.26p
```

默认输出 `rtk_std_analysis.txt`(生成在流动站文件所在目录),默认已带 4.6cm/7.5cm 协方差地板;
设 `--floor-h 0 --floor-v 0` 则输出原始 RTKLIB std。源码见 `rtkproc.cpp`,
编译:`g++ -O2 -static -static-libgcc -static-libstdc++ -o rtkproc.exe rtkproc.cpp`。

### 输出约定(重要)

第 10 列是 **vd,地向为正(NED)**。RTKLIB 自己的 `.pos` 输出是 ENU 的 `vn ve vu`,
第三个分量天向为正,`convert()` 里已取负转成 NED。**不要把这个负号去掉** ——
下游的 KF-GINS 按 NED 读,符号反了程序不会报错,只表现为 NIS 偏高、垂向新息在
爬升/下降段成片超限,误差被挤进加计零偏。实测本数据未取负 NIS 4.48,取负后 0.98。

自检:相邻历元 `h` 的变化率应与 `-vd` 一致(相关系数 -1)。若与 `+vd` 一致就是反的。

### 协方差地板的标定

RTKLIB 报的形式精度在本机上是毫米级(sdn 中位 2.9 mm),而 RTK 真实精度是厘米级,
直接用会让下游滤波器过度相信位置观测。缺省的 4.6cm/7.5cm 是拿 KF-GINS 的 NIS
标定出来的:

| `--floor-h` / `--floor-v` | KF-GINS 全程 NIS |
| ------------------------- | -------------- |
| 0 / 0 (原始 RTKLIB std)     | ~19            |
| 0.010 / 0.015 (旧缺省)       | 4.5            |
| 0.040 / 0.065             | 1.30           |
| **0.046 / 0.075 (现缺省)**   | **1.02**       |

换接收机或换基线长度后重新标定:跑一遍 KF-GINS,看它打印的 `Mean normalised
innovation squared`,把两个 floor 乘以 `sqrt(NIS)` 再跑,一两轮就收敛到 1 附近。

### 基站坐标模式(影响绝对精度,不影响相对轨迹)

| 选项                    | 含义                                               | 绝对精度         |
| --------------------- | ------------------------------------------------ | ------------ |
| `--base-avg` **(默认)** | 对**整个基站文件**做单点定位(SPP)取均值,自动注入。**不限历元数**,任意批次时长通用 | ~1 m,且批次间可复现 |
| `--base-header`       | 直接用基站 RINEX 头 APPROX POSITION(单历元粗略 SPP)         | ~1~5 m,批次间随机 |
| `--base-llh 纬 经 高`    | 用已知/CORS/PPP 精确坐标                                | 厘米~分米        |

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
- `pos2-armode=fix-and-hold` 固定并保持
- `pos2-gloarmode=off` **异厂商基站必须置 off，见下**
- `ant2-postype=rinexhead` 基站坐标用 RINEX 头 APPROX POSITION
- `stats-errdoppler=2` 影响速度标准差量级

### `pos2-gloarmode` 必须按基站厂商设置

流动站是 Unicore UB4B0，基站是中海达 iRTK10，**异厂商的 GLONASS 频间偏差(IFB)没有标定**。
`pos2-gloarmode=on` 时这些未建模的偏差会污染整组模糊度，表现为**全程 100% float、
ratio 恒在 1.0 附近** —— 不报错，只是永远固定不了。实测 0813 架次二：

| 配置 | 固定率 | 平均 ratio |
| --- | --- | --- |
| `gloarmode=on`, `armode=continuous` (旧缺省) | 0% | 1.0 |
| `gloarmode=off`, `armode=continuous` | 52.1% | 3.8 |
| `gloarmode=off`, `armode=fix-and-hold` (现缺省) | 62.8% | 18.2 |

两者都固定的历元位置一致到 1 mm，fix-and-hold 只是把固定状态保持得更久，没有引入不同的解。
float 相对 fixed 的系统差实测为**高程 0.24~0.52 m**、东向 0.08 m —— 这是绝对高程误差，会
直接进正射产品。

换成与流动站同厂商的基站后可以再打开 `gloarmode=on`(多 8~10 颗可固定卫星)，但换之前
务必先按上表复核一次固定率。另：`pos1-elmask` 不要从 15 降到 10，实测固定率反而掉到 12%。

### 解里混有 fix 和 float 时用 `pos2std_q.py` 而不是 `--floor-h/--floor-v`

`rtkproc.exe` 的地板是全程一个值。一旦解里既有固定又有浮动，这个假设就不成立：
RTKLIB 报的形式精度**完全区分不出两者**(实测浮动解的 sd 有时比固定解还小)，而真实精度
差一个量级。后果是 KF-GINS 在 fix↔float 跳变处成片吃大新息 —— 0813 架次一在跳变处
相邻历元的垂向位置增量与多普勒速度失配中位数 1.24 m，而 Q 不变时只有 0.026 m。

```
python pos2std_q.py rover.pos rtk.txt --float-h 0.15 --float-v 0.35
```

固定历元沿用标定好的 0.046/0.075，浮动历元单独给一个更大的地板，让滤波器在浮动段
多依赖惯导。地板相同时该脚本与 `rtkproc.exe` 的输出逐字节一致(含 vd 取负的约定)。

## 与参考文件 rtk_std_analysis.txt 的对比 (全 1236 历元)

- 平面差 RMS ≈ 5~12 cm, 高程 RMS ≈ 8 cm, 平面平均差 9 cm。
- 位置量级完全一致, 验证流程正确。
- 位置标准差(5-7列)本方案约为参考文件的 ~1/10:
  参考文件用了更大的观测噪声模型(或来自接收机自带 RTK 引擎)。
  如需让标准差数值也贴近参考, 调大 `stats-errphase`(如 0.006~0.02)。

# 
