#!/usr/bin/env python3
"""pos2std_q.py —— 把 RTKLIB .pos 转成 KF-GINS 的 13 列文件，按固定/浮动分别设协方差地板。

rtkproc.exe 的 --floor-h/--floor-v 是全程一个地板。当解算里既有固定解又有浮动解时
这个假设不成立：RTKLIB 报的形式精度完全区分不出两者(实测本机数据浮动解的 sd 甚至
比固定解还小)，而两者的真实精度差一个量级。用一个地板的后果是 KF-GINS 在 fix<->float
跳变处吃到成片的大新息 —— 实测 0813 架次一 NIS 4.60，跳变处相邻历元的垂向位置增量
与多普勒速度失配中位数 1.24 m，而 Q 不变时只有 0.026 m。

本脚本对 Q=1(固定) 和 Q!=1(浮动/单点) 分别取地板，让滤波器自己在浮动段更依赖惯导。

    python pos2std_q.py <in.pos> <out.txt> [--fix-h m] [--fix-v m]
                        [--float-h m] [--float-v m] [--scale k]

列定义与 rtkproc 一致(见 proc/README.md)：
    1      GPS 周内秒
    2-4    纬度(deg) 经度(deg) 椭球高(m)
    5-7    位置标准差 sdn sde sdu (m)
    8-10   速度 vn ve vd (m/s)   —— vd 地向为正，由 .pos 的 vu 取负而来，勿改
    11-13  速度标准差
"""
import sys
import math

def infl(sd, fl, sc):
    sd *= sc
    return math.sqrt(sd * sd + fl * fl)

def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    src, dst = argv[1], argv[2]
    o = dict(fix_h=0.046, fix_v=0.075, float_h=0.046, float_v=0.075, scale=1.0)
    i = 3
    while i < len(argv):
        k = argv[i].lstrip('-').replace('-', '_')
        if k not in o:
            print("unknown option: %s" % argv[i])
            return 2
        o[k] = float(argv[i + 1])
        i += 2

    n = {}
    with open(src, 'r', errors='ignore') as fi, open(dst, 'w') as fo:
        for line in fi:
            if not line.strip() or line[0] == '%':
                continue
            v = line.split()
            if len(v) < 13:
                continue
            v = [float(x) for x in v]
            q = int(v[5])
            fh, fv = ((o['fix_h'], o['fix_v']) if q == 1 else
                      (o['float_h'], o['float_v']))
            tow = math.floor(v[1] + 0.5)
            vn = ve = vd = svn = sve = svd = 0.0
            if len(v) >= 21:
                # RTKLIB .pos 速度是 ENU(天向为正)，KF-GINS 按 NED 读，第三列必须取负。
                vn, ve, vd = v[15], v[16], -v[17]
                svn, sve, svd = v[18], v[19], v[20]
            fo.write("%.6f %.10f %.10f %.6f %.6f %.6f %.6f "
                     "%.6f %.6f %.6f %.6f %.6f %.6f\n" % (
                         tow, v[2], v[3], v[4],
                         infl(v[7], fh, o['scale']), infl(v[8], fh, o['scale']),
                         infl(v[9], fv, o['scale']),
                         vn, ve, vd, svn, sve, svd))
            n[q] = n.get(q, 0) + 1

    tot = sum(n.values())
    print("converted %d epochs -> %s" % (tot, dst))
    print("  Q=1 fixed : %4d (%.1f%%)  floor %.3f / %.3f" % (
        n.get(1, 0), 100.0 * n.get(1, 0) / max(tot, 1), o['fix_h'], o['fix_v']))
    print("  Q!=1      : %4d (%.1f%%)  floor %.3f / %.3f" % (
        tot - n.get(1, 0), 100.0 * (tot - n.get(1, 0)) / max(tot, 1),
        o['float_h'], o['float_v']))
    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv))
