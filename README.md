# RTKLIB 2.4.3 b34（本分支）

> fork 自 [RTKLIB 2.4.3 b34](https://github.com/tomojitakasu/RTKLIB)，为无人机航飞的
> **GNSS/INS 后处理**加了一层封装。RTKLIB 内核（`src/`、`app/`、`lib/`）未作改动，
> 全部新增内容在 `proc/` 目录。

配套的组合导航软件见 [monch999/INS](https://github.com/monch999/INS)。

## 相对上游的差异

### 新增 `proc/` —— 单文件后处理封装

| 文件 | 作用 |
|---|---|
| `rtkproc.cpp` / `rtkproc.exe` | 封装程序：路径归一化 → 基站坐标确定 → 调用解算 → 转 13 列格式 |
| `rnx2rtkp.exe` | RTKLIB 解算内核（静态链接，只依赖 Windows 系统 DLL） |
| `rtk.conf` | 动态 RTK 解算配置（GPS+GLO+GAL+QZS+BDS，L1/L2/L5，连续模糊度固定） |

`rtkproc` 解决的问题：

1. **路径 bug 绕过**。RTKLIB 的 `expath()` 在 Windows 下只按反斜杠 `\` 拆路径，
   用正斜杠或从别处传绝对路径会丢掉目录、报 "no obs data"。封装内部统一归一化。
2. **基站坐标自动确定**。缺省对整个基站文件做单点定位取 3σ 截断均值再注入，
   不限历元数、任意批次时长通用；也可用 RINEX 头或已知坐标。
3. **格式转换**。把 RTKLIB `.pos` 转成下游 GNSS/INS 要的 13 列
   `时间 纬 经 高 位置std(3) 速度(3) 速度std(3)`。

一条命令从 RINEX 到成品：

```
rtkproc.exe <流动站obs> <基站obs> <混合星历nav> [nav2 ...] [-o out.txt]
```

### 垂向速度符号：ENU → NED 转换

**这是本分支最重要的一处修正。**

RTKLIB `.pos` 的速度列是 ENU，表头自己写的就是 `vn(m/s) ve(m/s) vu(m/s)`——
第三个分量**天向为正**。而下游 GNSS/INS 滤波器按 NED 读第三列，要求**地向为正**。
封装最初原样透传，导致每一个垂向速度观测的符号都是反的。

```c
/* 修正前 */  vu = v[17];
/* 修正后 */  vd = -v[17];   /* ENU up -> NED down */
```

这个错误**不会让任何程序报错**，只表现为组合导航的 NIS 偏高、垂向新息在爬升/下降段
成片超限、误差被挤进加计零偏。实测同一批数据：未取负时 KF-GINS 全程 NIS **4.48**，
取负后 **0.98**（理论值 1.0）。

**自检方法**：相邻历元 `h` 的变化率应与 `−vd` 一致（相关系数 −1）。
若与 `+vd` 一致（+1）就是反的。

### 协方差地板的标定

RTKLIB 报的形式精度在本机上是**毫米级**（`sdn` 中位 2.9 mm），而 RTK 的真实精度是
厘米级。直接用会让下游滤波器过度相信位置观测。`--floor-h` / `--floor-v` 按
`sqrt(sd² + floor²)` 膨胀标准差，缺省值由 KF-GINS 的 NIS 标定：

| `--floor-h` / `--floor-v` | KF-GINS 全程 NIS |
|---|---|
| 0 / 0（原始 RTKLIB std） | ~19 |
| 0.010 / 0.015（旧缺省） | 4.5 |
| 0.040 / 0.065 | 1.30 |
| **0.046 / 0.075（现缺省）** | **1.02** |

换接收机或换基线长度后重新标定：跑一遍 KF-GINS，看它打印的
`Mean normalised innovation squared`，把两个 floor 乘以 `sqrt(NIS)` 再跑，
一两轮就收敛到 1 附近。

## 输出格式

`rtk_std_analysis.txt`，13 列，1 Hz：

| 列 | 含义 |
|---|---|
| 1 | GPS 周内秒 |
| 2–4 | 纬度(deg)、经度(deg)、椭球高(m) |
| 5–7 | 位置标准差 N/E/D (m)，已加协方差地板 |
| 8–10 | 速度 vn / ve / **vd**（m/s，**NED，地向为正**） |
| 11–13 | 速度标准差 (m/s) |

## 构建

```
cd proc
g++ -O2 -static -static-libgcc -static-libstdc++ -o rtkproc.exe rtkproc.cpp
```

RTKLIB 内核按上游方式构建（`app/consapp/rnx2rtkp/gcc/makefile`）。

## 数据

观测数据（RINEX、解算结果）不入库。`proc/README.md` 有更详细的用法、
基站坐标模式对比和配置说明。

---

上游 RTKLIB 的说明见 [`readme.txt`](readme.txt)，许可见 [`LICENSE.txt`](LICENSE.txt)。
