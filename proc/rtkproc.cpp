// rtkproc.cpp - single-exe wrapper around RTKLIB rnx2rtkp.exe
//   * normalizes input paths to absolute backslash form (works around the
//     Windows expath() bug where rnx2rtkp only splits paths on '\\')
//   * base coordinate: by default auto-computes the base station position by
//     AVERAGING a single-point (SPP) solution over the WHOLE base file
//     (batch-agnostic - not tied to any fixed duration). Options let you keep
//     the RINEX header, or supply a known coordinate.
//   * runs the kinematic RTK, then converts the .pos to the 13-column
//     rtk_std_analysis format with covariance inflation on the position std.
//   * every stage is reported through Logger: timestamped, levelled and
//     numbered. Diagnostics go to stderr AND to rtkproc.log, written next to
//     the output file so a run and its log stay together. init() REPLACES it
//     on every run - the log always describes the last run only, so it can be
//     pasted into a bug report as-is.
//
// Build (MinGW):
//   g++ -O2 -static -static-libgcc -static-libstdc++ -o rtkproc.exe rtkproc.cpp Logger.cpp
//
// Usage:
//   rtkproc.exe <rover_obs> <base_obs> <nav> [nav2 ...]
//               [-o out.txt] [-c rtk.conf]
//               [--base-avg | --base-header | --base-llh lat lon h]
//               [--floor-h m] [--floor-v m] [--scale k] [--keep-pos]
//               [--no-auto-elmask]
//               [--log file | --no-log] [-v|--verbose] [-q|--quiet]
//
// Elevation mask (--auto-elmask, on by default):
//   pos1-elmask is the one setting that changes from flight to flight, because
//   it depends on what the BASE antenna can see reflecting back at it. Run the
//   config's own value first; if that already fixes well enough, stop there -
//   the common case costs exactly one rnx2rtkp pass, same as before. Otherwise
//   walk up 25/30/35/40/45 deg, score every rung, and keep the best one. The
//   whole ladder is walked, never stopped at the first improvement: the curve
//   is not monotonic (one flight here goes 93.8% at 35 deg, 78.3% at 40, 99.2%
//   at 45) because RTKLIB's LAMBDA fixes the ambiguity set all-or-nothing, so
//   which satellites are in the set matters more than how many.
//   --no-auto-elmask restores the old behaviour: one pass at the config value.
//
// Base coordinate modes:
//   --base-avg    (default) average an SPP solution over the whole base file
//   --base-header use the base RINEX header APPROX POSITION as-is
//   --base-llh    use a known coordinate (deg deg m)
//
// Logging:
//   --no-auto-elmask  one RTK pass at the config's pos1-elmask, no ladder
//
//   --log file    write the run log here (default: rtkproc.log beside -o)
//   --no-log      console only, no log file
//   -v/--verbose  add the DEBUG lines: child command lines, exit codes, timings,
//                 SPP scatter - the detail wanted when a run has to be explained
//   -q/--quiet    silence the console; the log file still gets every line
#include <windows.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <string>
#include <vector>
#include "Logger.h"
using namespace std;

// Shorthand - every line in this file goes through the one singleton.
#define LOG Logger::getInstance()

// Elapsed seconds since construction; each stage reports its own cost so a slow
// run can be attributed to the SPP pass, the RTK pass or the conversion.
struct Stopwatch {
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    double s() const {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    }
};

static string exeDir() {
    char buf[MAX_PATH]; GetModuleFileNameA(NULL, buf, MAX_PATH);
    string s(buf); size_t p = s.find_last_of("\\/");
    return (p == string::npos) ? string() : s.substr(0, p + 1);
}
static string fullPath(const string& in) {
    char buf[MAX_PATH]; DWORD n = GetFullPathNameA(in.c_str(), MAX_PATH, buf, NULL);
    return (n && n < MAX_PATH) ? string(buf) : in;
}
static string quote(const string& s) { return "\"" + s + "\""; }
static string dirOf(const string& p) {
    size_t s = p.find_last_of("\\/");
    return (s == string::npos) ? string() : p.substr(0, s + 1);
}

// run child process; route its stdout/stderr to NUL; return exit code (-1 launch fail)
static int runProc(const string& cmd) {
    // The exact command line is the first thing wanted when a stage misbehaves
    // (quoting, a missing file, the wrong config), so it is always in the log at
    // DEBUG - a failed stage is reproduced by pasting the line into a shell.
    LOG.detailDebug("exec: %s", cmd.c_str());
    Stopwatch sw;
    STARTUPINFOA si; PROCESS_INFORMATION pi;
    ZeroMemory(&si, sizeof si); si.cb = sizeof si;
    ZeroMemory(&pi, sizeof pi);
    SECURITY_ATTRIBUTES sa; sa.nLength = sizeof sa; sa.bInheritHandle = TRUE;
    sa.lpSecurityDescriptor = NULL;
    HANDLE hNul = CreateFileA("NUL", GENERIC_WRITE, FILE_SHARE_WRITE, &sa,
                              OPEN_EXISTING, 0, NULL);
    if (hNul != INVALID_HANDLE_VALUE) {
        si.dwFlags = STARTF_USESTDHANDLES;
        si.hStdOutput = hNul; si.hStdError = hNul;
        si.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
    }
    vector<char> cl(cmd.begin(), cmd.end()); cl.push_back('\0');
    BOOL ok = CreateProcessA(NULL, cl.data(), NULL, NULL, TRUE, 0, NULL, NULL, &si, &pi);
    if (hNul != INVALID_HANDLE_VALUE) CloseHandle(hNul);
    if (!ok) {
        LOG.error("CreateProcess failed (win32 error %lu): %s",
                  (unsigned long)GetLastError(), cmd.c_str());
        return -1;
    }
    WaitForSingleObject(pi.hProcess, INFINITE);
    DWORD rc = 0; GetExitCodeProcess(pi.hProcess, &rc);
    CloseHandle(pi.hProcess); CloseHandle(pi.hThread);
    LOG.detailDebug("exit code %d after %.1f s", (int)rc, sw.s());
    return (int)rc;
}

// 3-sigma-clipped mean of a sample
static double clipMean(const vector<double>& x) {
    int n = (int)x.size(); if (!n) return 0;
    double m = 0; for (double v : x) m += v; m /= n;
    double s = 0; for (double v : x) s += (v - m) * (v - m); s = sqrt(s / n);
    if (s <= 0) return m;
    double m2 = 0; int c = 0;
    for (double v : x) if (fabs(v - m) <= 3 * s) { m2 += v; c += 1; }
    return c ? m2 / c : m;
}

// population std dev - logged only, to show how noisy the averaged SPP was
static double stdDev(const vector<double>& x) {
    int n = (int)x.size(); if (n < 2) return 0;
    double m = 0; for (double v : x) m += v; m /= n;
    double s = 0; for (double v : x) s += (v - m) * (v - m);
    return sqrt(s / n);
}

// SPP-average the base over the whole file. Fills lat/lon/h (deg,deg,m).
static bool baseSppAverage(const string& exe, const string& conf,
                           const string& baseObs, const vector<string>& navs,
                           const string& tmpPos, bool keep,
                           double& lat, double& lon, double& h, int& nused) {
    // run single-point mode on base obs + nav (uses conf's navsys, -p 0 forces single)
    string cmd = quote(exe) + " -k " + quote(conf) + " -p 0 " + quote(baseObs);
    for (const auto& nv : navs) cmd += " " + quote(nv);
    cmd += " -o " + quote(tmpPos);
    int rc = runProc(cmd);
    if (rc != 0) LOG.detail("rnx2rtkp (SPP) returned %d; reading whatever it wrote", rc);
    FILE* f = fopen(tmpPos.c_str(), "r");
    if (!f) { LOG.detail("no SPP solution file at %s", tmpPos.c_str()); return false; }
    vector<double> la, lo, hh; char line[2048];
    long nline = 0, nskip = 0;
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '%' || line[0] == '\n' || line[0] == '\r' || line[0] == '\0') continue;
        ++nline;
        // fields: week tow lat lon h Q ns ...
        vector<double> v; char* ctx = NULL;
        for (char* t = strtok_r(line, " \t\r\n", &ctx); t; t = strtok_r(NULL, " \t\r\n", &ctx))
            v.push_back(atof(t));
        if (v.size() < 6) { ++nskip; continue; }
        int Q = (int)v[5];
        if (Q <= 0) { ++nskip; continue; }           // no fix this epoch
        la.push_back(v[2]); lo.push_back(v[3]); hh.push_back(v[4]);
    }
    fclose(f);
    if (!keep) DeleteFileA(tmpPos.c_str());
    LOG.detail("SPP epochs: %ld read, %ld unusable, %zu averaged", nline, nskip, la.size());
    if (la.empty()) { LOG.detail("not a single usable SPP epoch in the base file"); return false; }
    lat = clipMean(la); lon = clipMean(lo); h = clipMean(hh);
    nused = (int)la.size();
    // Scatter of the raw samples, in metres. Metre-level horizontal scatter is
    // ordinary for SPP; ten metres says the base observations are the problem,
    // and the averaged coordinate that follows should not be trusted.
    const double PI = 3.14159265358979323846, M_PER_DEG = 111320.0;
    LOG.detailDebug("SPP scatter (1-sigma): north %.2f m, east %.2f m, up %.2f m",
                    stdDev(la) * M_PER_DEG,
                    stdDev(lo) * M_PER_DEG * cos(lat * PI / 180.0),
                    stdDev(hh));
    return true;
}

// Below this many satellites an epoch is called out in the log: the solution is
// still written, but its geometry no longer supports the std it reports.
static const int    NS_THIN      = 6;
static double infl(double sd, double fl, double sc) { sd *= sc; return sqrt(sd*sd + fl*fl); }

static bool convert(const string& pos, const string& out,
                    double fh, double fv, double sc) {
    FILE* fi = fopen(pos.c_str(), "r");
    if (!fi) { LOG.error("cannot open %s", pos.c_str()); return false; }
    FILE* fo = fopen(out.c_str(), "w");
    if (!fo) { LOG.error("cannot write %s", out.c_str()); fclose(fi); return false; }
    char line[2048];
    long nrec = 0, nshort = 0, nvel = 0, nfix = 0, nfloat = 0, nother = 0;
    // Satellite count per epoch (the .pos ns column). The fix ratio says how often
    // the ambiguities resolved; this says on how much geometry they resolved, which
    // is the other half of the same judgement - a high fix ratio carried by five
    // satellites is luck, and a float stretch with twenty is a data problem.
    double nsSum = 0, nsSumFix = 0;
    long   nsNFix = 0;
    int    nsMin = 0, nsMax = 0;
    bool   nsFirst = true;
    double tFirstThin = 0; int nThin = 0;   // epochs that fell under NS_THIN
    while (fgets(line, sizeof(line), fi)) {
        if (line[0] == '%' || line[0] == '\n' || line[0] == '\r' || line[0] == '\0') continue;
        vector<double> v; char* ctx = NULL;
        for (char* t = strtok_r(line, " \t\r\n", &ctx); t; t = strtok_r(NULL, " \t\r\n", &ctx))
            v.push_back(atof(t));
        if (v.size() < 13) { ++nshort; continue; }
        double tow = floor(v[1] + 0.5);
        double lat = v[2], lon = v[3], h = v[4];
        double sdn = v[7], sde = v[8], sdu = v[9];
        int ns = (int)v[6];                         // satellites used this epoch
        if (nsFirst || ns < nsMin) nsMin = ns;
        if (nsFirst || ns > nsMax) nsMax = ns;
        nsFirst = false;
        nsSum += ns;
        if (ns < NS_THIN) { if (!nThin) tFirstThin = tow; ++nThin; }
        switch ((int)v[5]) {                        // Q: 1 fix, 2 float, else single/none
            case 1:  ++nfix; nsSumFix += ns; ++nsNFix; break;
            case 2:  ++nfloat; break;
            default: ++nother; break;
        }
        // RTKLIB 的 .pos 速度列是 ENU: 表头写的就是 vn(m/s) ve(m/s) vu(m/s), 第三个分量
        // 天向为正。而本文件的消费者(KF-GINS 等 GNSS/INS 组合)按 NED 读第三列, 要求地向
        // 为正。原样透传会让每一个垂向速度观测的符号都反掉 —— 滤波器在爬升时认为自己在
        // 下降, 整个爬升/下降段的新息成片超限, 误差被挤进加计零偏。症状很隐蔽: 程序不报错,
        // 只是 NIS 偏高。实测本数据未取负时 KF-GINS 全程 NIS 4.48, 取负后 0.98。
        // 自检: 相邻历元 h 的变化率应与 -vd 一致(相关系数 -1)。
        //
        // RTKLIB writes ENU velocity -- its own header says vu(m/s), UP-positive. The
        // consumers of this file (KF-GINS and other GNSS/INS filters) read the third column
        // as NED, DOWN-positive. Passing it through unchanged flips the sign of every
        // vertical velocity measurement: the filter believes it is descending while it
        // climbs, innovations blow past their bounds throughout every climb and descent, and
        // the error is absorbed into the accelerometer bias. Nothing errors out -- the only
        // symptom is an inflated NIS (4.48 before the negation on this dataset, 0.98 after).
        double vn = 0, ve = 0, vd = 0, svn = 0, sve = 0, svd = 0;
        if (v.size() >= 21) {
            vn = v[15]; ve = v[16]; vd = -v[17];   // ENU up -> NED down
            svn = v[18]; sve = v[19]; svd = v[20]; // 标准差是量值, 不随符号变
            ++nvel;
        }
        fprintf(fo, "%.6f %.10f %.10f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
                tow, lat, lon, h,
                infl(sdn, fh, sc), infl(sde, fh, sc), infl(sdu, fv, sc),
                vn, ve, vd, svn, sve, svd);
        ++nrec;
    }
    fclose(fi); fclose(fo);
    LOG.detail("converted %ld epochs -> %s", nrec, out.c_str());
    LOG.detail("inflation: floor_h=%.3f m, floor_v=%.3f m, scale=%.2f", fh, fv, sc);
    if (nshort > 0) LOG.detailDebug("%ld line(s) skipped: fewer than 13 fields", nshort);
    if (nrec == 0) {
        LOG.error("no epoch converted - %s held no usable record", pos.c_str());
        return false;
    }
    // 固定率是判断这一趟 RTK 好不好的第一个数字; 全程没有固定解通常意味着基站坐标或
    // 配置有问题, 而不是数据本身不行。
    // The fix ratio is the first number that says whether this RTK run was any good;
    // no fixed epoch at all usually points at the base coordinate or the config
    // rather than at the observations.
    LOG.detail("solution quality: fix %ld (%.1f%%), float %ld (%.1f%%), other %ld (%.1f%%)",
               nfix,   100.0 * nfix   / nrec,
               nfloat, 100.0 * nfloat / nrec,
               nother, 100.0 * nother / nrec);
    // Satellites used. Reported for the whole run and again for the fixed epochs
    // only: if the second number is well above the first, the fixes are riding on
    // the stretches where the sky opened up, and the float stretches are the ones
    // short of geometry.
    LOG.detail("satellites used: %.1f mean, %d min, %d max (fixed epochs: %.1f mean)",
               nsSum / nrec, nsMin, nsMax, nsNFix ? nsSumFix / nsNFix : 0.0);
    if (nThin > 0)
        LOG.warn("%ld epoch(s) ran on fewer than %d satellites, first at tow %.0f -"
                 " thin geometry, treat those epochs as degraded",
                 (long)nThin, NS_THIN, tFirstThin);
    if (nfix == 0)
        LOG.warn("no fixed epoch in the solution - check the base coordinate and the config");
    if (nvel == 0)
        LOG.warn("the .pos carries no velocity columns; vn/ve/vd written as zero");
    else if (nvel < nrec)
        LOG.warn("%ld of %ld epochs had no velocity columns (written as zero)", nrec - nvel, nrec);
    return true;
}

// --auto-elmask acceptance. A first rung at or above AUTO_MIN_FIX ends the run
// right there, so a healthy flight still costs exactly one rnx2rtkp pass.
// AUTO_MIN_NS is the geometry veto: on this hardware a rung that drops below
// five satellites starts fixing on luck (0135 goes 100% at 40 deg and 58.9% at
// 45 deg for exactly that reason).
static const double AUTO_MIN_FIX = 80.0;   // percent of epochs fixed
static const int    AUTO_MIN_NS  = 5;      // fewest satellites in any epoch

// Everything the elmask ladder needs to rank one rung, read back out of the .pos
// it produced. Fix ratio is the score, mean ratio breaks ties, and ns_min is the
// veto - a rung that fixes well on four satellites is luck, not geometry.
struct PosStats {
    long   nrec = 0, nfix = 0;
    double fixPct = 0, meanRatio = 0, nsMean = 0;
    int    nsMin = 0, nsMax = 0;
    bool   ok = false;
};

static PosStats scorePos(const string& path) {
    PosStats st;
    FILE* f = fopen(path.c_str(), "r");
    if (!f) return st;
    char line[2048];
    double rsum = 0, nssum = 0; long nratio = 0;
    int nsmin = 0, nsmax = 0; bool first = true;
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '%' || line[0] == '\n' || line[0] == '\r' || line[0] == '\0') continue;
        vector<double> v; char* ctx = NULL;
        for (char* t = strtok_r(line, " \t\r\n", &ctx); t; t = strtok_r(NULL, " \t\r\n", &ctx))
            v.push_back(atof(t));
        if (v.size() < 15) continue;                 // need through the ratio column
        ++st.nrec;
        if ((int)v[5] == 1) ++st.nfix;               // Q=1 -> fixed
        int ns = (int)v[6];                          // satellites used this epoch
        if (first || ns < nsmin) nsmin = ns;
        if (first || ns > nsmax) nsmax = ns;
        first = false;
        nssum += ns;
        rsum += v[14]; ++nratio;                     // ratio column
    }
    fclose(f);
    if (st.nrec) {
        st.fixPct    = 100.0 * st.nfix / st.nrec;
        st.meanRatio = nratio ? rsum / nratio : 0.0;
        st.nsMean    = nssum / st.nrec;
        st.nsMin     = nsmin;
        st.nsMax     = nsmax;
        st.ok        = true;
    }
    return st;
}

// The config's own pos1-elmask: the first rung of the ladder, and the only one
// used under --no-auto-elmask. Last occurrence wins, the same way RTKLIB's
// loadopts() treats a repeated key.
static int readElmask(const string& conf, int dflt) {
    FILE* f = fopen(conf.c_str(), "r");
    if (!f) return dflt;
    char line[4096]; int val = dflt;
    while (fgets(line, sizeof line, f)) {
        char* p = line;
        while (*p == ' ' || *p == '\t') ++p;
        if (strncmp(p, "pos1-elmask", 11) != 0) continue;
        char* eq = strchr(p, '=');
        if (eq) val = atoi(eq + 1);
    }
    fclose(f);
    return val;
}

// write an effective config = copy of base conf + appended overrides.
// llh==NULL leaves the base coordinate alone, elmask<=0 leaves pos1-elmask
// alone. Appending works because loadopts() keeps the LAST value read for a key.
static bool writeConfOverride(const string& baseConf, const string& outConf,
                              const double* llh, int elmask) {
    FILE* fi = fopen(baseConf.c_str(), "r");
    if (!fi) { LOG.detail("cannot read config %s", baseConf.c_str()); return false; }
    FILE* fo = fopen(outConf.c_str(), "w");
    if (!fo) { LOG.detail("cannot write temp config %s", outConf.c_str()); fclose(fi); return false; }
    char buf[4096]; size_t n;
    while ((n = fread(buf, 1, sizeof buf, fi)) > 0) fwrite(buf, 1, n, fo);
    fclose(fi);
    if (llh) {
        fprintf(fo, "\n# --- base coordinate injected by rtkproc (SPP average) ---\n");
        fprintf(fo, "ant2-postype=llh\n");
        fprintf(fo, "ant2-pos1=%.9f\n", llh[0]);
        fprintf(fo, "ant2-pos2=%.9f\n", llh[1]);
        fprintf(fo, "ant2-pos3=%.4f\n", llh[2]);
    }
    if (elmask > 0)
        fprintf(fo, "\n# --- elevation mask injected by rtkproc (--auto-elmask) ---\n"
                    "pos1-elmask=%d\n", elmask);
    fclose(fo);
    LOG.detailDebug("effective config written: %s", outConf.c_str());
    return true;
}

enum BaseMode { BASE_AVG, BASE_HEADER, BASE_LLH };

static const char* baseModeName(BaseMode m) {
    switch (m) {
        case BASE_AVG:    return "--base-avg";
        case BASE_HEADER: return "--base-header";
        case BASE_LLH:    return "--base-llh";
        default:          return "?";
    }
}

int main(int argc, char** argv) {
    string conf, out;
    // 协方差地板的缺省值。RTKLIB 报的形式精度在本机上是毫米级(sdn 中位 2.9 mm), 而 RTK 的
    // 真实精度是厘米级, 直接用会让下游滤波器过度相信位置观测。下面的值是拿 KF-GINS 的
    // NIS 标定出来的: 0.046/0.075 时全程 NIS = 1.02(理论值 1.0), 而 0.010/0.015 时 NIS
    // 会到 19 倍。换接收机、换基线长度后应重新标定 —— 跑一遍 KF-GINS, 看它打印的 NIS,
    // 按 sqrt(NIS) 缩放这两个值再跑, 一两轮就收敛。
    // Default covariance floors. RTKLIB's formal precision is millimetre-level here (median
    // sdn 2.9 mm) while real RTK accuracy is centimetre-level, so using it verbatim makes
    // any downstream filter trust the position far too much. These values were calibrated
    // against KF-GINS's NIS: 0.046/0.075 gives a whole-run NIS of 1.02 against a theoretical
    // 1.0, where 0.010/0.015 lands about 19x off. Recalibrate for a different receiver or
    // baseline length: run KF-GINS, read the NIS it prints, scale both by sqrt(NIS), repeat.
    double fh = 0.046, fv = 0.075, sc = 1.0;
    bool keepPos = false;
    BaseMode bmode = BASE_AVG;
    double blat = 0, blon = 0, bh = 0;
    string logFile;                     // empty -> rtkproc.log beside the output
    bool wantLog = true, verbose = false, quiet = false;
    bool autoElmask = true;             // --no-auto-elmask turns the ladder off
    vector<string> inputs;
    for (int i = 1; i < argc; ++i) {
        string a = argv[i];
        if (a == "-o" && i + 1 < argc) out = argv[++i];
        else if (a == "-c" && i + 1 < argc) conf = argv[++i];
        else if (a == "--floor-h" && i + 1 < argc) fh = atof(argv[++i]);
        else if (a == "--floor-v" && i + 1 < argc) fv = atof(argv[++i]);
        else if (a == "--scale" && i + 1 < argc) sc = atof(argv[++i]);
        else if (a == "--keep-pos") keepPos = true;
        else if (a == "--auto-elmask") autoElmask = true;
        else if (a == "--no-auto-elmask") autoElmask = false;
        else if (a == "--base-avg") bmode = BASE_AVG;
        else if (a == "--base-header") bmode = BASE_HEADER;
        else if (a == "--log" && i + 1 < argc) { logFile = argv[++i]; wantLog = true; }
        else if (a == "--no-log") wantLog = false;
        else if (a == "-v" || a == "--verbose") verbose = true;
        else if (a == "-q" || a == "--quiet") quiet = true;
        else if (a == "--base-llh" && i + 3 < argc) {
            bmode = BASE_LLH; blat = atof(argv[++i]); blon = atof(argv[++i]); bh = atof(argv[++i]);
        } else inputs.push_back(a);
    }
    // Console logging is live from here on. The file cannot be opened yet - its
    // default location is the rover directory - so early failures below are
    // console-only, which is where a user who mistyped the command line looks.
    if (verbose) LOG.setLevel(Logger::LogLevel::Debug);
    if (quiet)   LOG.setConsole(false);

    if (inputs.size() < 3) {
        fprintf(stderr,
            "Usage: rtkproc <rover_obs> <base_obs> <nav> [nav2 ...]\n"
            "               [-o out.txt] [-c rtk.conf]\n"
            "               [--base-avg | --base-header | --base-llh lat lon h]\n"
            "               [--floor-h m] [--floor-v m] [--scale k] [--keep-pos]\n"
            "               [--no-auto-elmask]\n"
            "               [--log file | --no-log] [-v|--verbose] [-q|--quiet]\n");
        return 1;
    }
    string dir = exeDir();
    if (conf.empty()) conf = dir + "rtk.conf";
    conf = fullPath(conf);
    string exe = dir + "rnx2rtkp.exe";
    if (GetFileAttributesA(exe.c_str()) == INVALID_FILE_ATTRIBUTES) {
        LOG.error("rnx2rtkp.exe not found next to rtkproc.exe (%s)", exe.c_str());
        return 1;
    }
    for (auto& s : inputs) s = fullPath(s);            // path fix
    string rover = inputs[0], base = inputs[1];
    vector<string> navs(inputs.begin() + 2, inputs.end());

    string roverDir = dirOf(rover);
    if (out.empty()) out = roverDir + "rtk_std_analysis.txt"; else out = fullPath(out);
    string posf = roverDir + "rtkproc_tmp.pos";
    string sppf = roverDir + "rtkproc_base_spp.pos";
    string effConf = conf;                             // config actually used
    string tmpConf = roverDir + "rtkproc_tmp.conf";

    if (wantLog) {
        // Default next to the output file, not next to the rover: the log
        // describes how that output was produced, so the two belong together
        // (they only differ once -o points somewhere else).
        logFile = logFile.empty() ? dirOf(out) + "rtkproc.log" : fullPath(logFile);
        // init() truncates: what is on disk is this run and nothing else.
        if (!LOG.init(logFile))
            LOG.warn("cannot open log file %s - continuing on the console only", logFile.c_str());
    }

    Stopwatch runTime;
    // BASE_HEADER needs no step of its own: nothing is computed for it.
    LOG.setStepTotal(bmode == BASE_HEADER ? 3 : 4);
    LOG.step("Resolving inputs");
    LOG.detail("rover  : %s", rover.c_str());
    LOG.detail("base   : %s", base.c_str());
    for (size_t i = 0; i < navs.size(); ++i)
        LOG.detail("nav[%zu] : %s", i, navs[i].c_str());
    LOG.detail("config : %s", conf.c_str());
    LOG.detail("output : %s", out.c_str());
    if (!LOG.logPath().empty()) LOG.detail("log    : %s", LOG.logPath().c_str());
    LOG.detailDebug("rnx2rtkp: %s", exe.c_str());
    LOG.detailDebug("base mode %s, floor_h=%.3f, floor_v=%.3f, scale=%.2f, keep-pos=%s,"
                    " auto-elmask=%s",
                    baseModeName(bmode), fh, fv, sc, keepPos ? "yes" : "no",
                    autoElmask ? "on" : "off");

    // ---- decide base coordinate ----
    if (bmode == BASE_AVG) {
        LOG.step("Base coordinate: averaging SPP over the whole base file");
        Stopwatch sw;
        double la, lo, h; int nep = 0;
        if (baseSppAverage(exe, conf, base, navs, sppf, keepPos, la, lo, h, nep)) {
            LOG.detail("base SPP average (%d epochs): lat=%.9f lon=%.9f h=%.4f m", nep, la, lo, h);
            double llh[3] = { la, lo, h };
            if (writeConfOverride(conf, tmpConf, llh, 0)) effConf = tmpConf;
            else LOG.warn("could not write the temp config; using the RINEX header instead");
        } else {
            LOG.warn("base SPP failed; falling back to the RINEX header");
        }
        LOG.detailDebug("base coordinate took %.1f s", sw.s());
    } else if (bmode == BASE_LLH) {
        LOG.step("Base coordinate: user-supplied lat=%.9f lon=%.9f h=%.4f m", blat, blon, bh);
        double llh[3] = { blat, blon, bh };
        if (writeConfOverride(conf, tmpConf, llh, 0)) effConf = tmpConf;
        else LOG.warn("could not write the temp config; using the RINEX header instead");
    } else {
        LOG.info("base coordinate: RINEX header APPROX POSITION, used as-is");
    }

    // ---- main kinematic RTK: one pass, or the --auto-elmask ladder ----
    // pos1-elmask is the one setting that is a property of the SITE rather than
    // of the hardware - it has to clear whatever the base antenna sees reflecting
    // back at it. Everything else in the config generalises across flights.
    int confEl = readElmask(conf, 15);
    vector<int> rungs;
    rungs.push_back(confEl);
    if (autoElmask) {
        static const int LADDER[] = { 25, 30, 35, 40, 45 };
        for (int e : LADDER) if (e > confEl) rungs.push_back(e);
    }
    LOG.step("Running kinematic RTK over %zu file(s)", inputs.size());
    LOG.detail("effective config: %s", effConf.c_str());
    if (autoElmask && rungs.size() > 1)
        LOG.detail("auto-elmask: first pass at %d deg, ladder to %d deg if that is not enough",
                   confEl, rungs.back());

    string inputArgs;
    for (auto& s : inputs) inputArgs += " " + quote(s);

    Stopwatch rtkSw;
    vector<string>   rungPos(rungs.size());
    vector<PosStats> rungSt (rungs.size());
    size_t nrun = 0;
    for (size_t k = 0; k < rungs.size(); ++k) {
        int el = rungs[k];
        string cfg = effConf;
        rungPos[k] = posf;
        if (autoElmask) {
            char sfx[32]; snprintf(sfx, sizeof sfx, "_el%d", el);
            cfg        = roverDir + "rtkproc_tmp" + sfx + ".conf";
            rungPos[k] = roverDir + "rtkproc_tmp" + sfx + ".pos";
            if (!writeConfOverride(effConf, cfg, NULL, el)) {
                LOG.warn("cannot write the %d deg config; skipping that rung", el);
                rungPos[k].clear();
                continue;
            }
        }
        int rc = runProc(quote(exe) + " -k " + quote(cfg) + inputArgs + " -o " + quote(rungPos[k]));
        if (rc != 0) LOG.warn("rnx2rtkp exited with %d at elmask %d deg", rc, el);
        if (autoElmask) DeleteFileA(cfg.c_str());
        rungSt[k] = scorePos(rungPos[k]);
        ++nrun;
        if (!rungSt[k].ok) { LOG.detail("elmask %2d deg: no solution", el); continue; }
        LOG.detail("elmask %2d deg: fix %5.1f%%, mean ratio %6.1f, sats %.1f avg / %d min / %d max",
                   el, rungSt[k].fixPct, rungSt[k].meanRatio,
                   rungSt[k].nsMean, rungSt[k].nsMin, rungSt[k].nsMax);
        // The config's own value is the intended one. If it already fixes well on
        // real geometry, stop: the healthy case costs the same single pass it
        // always did, and only a bad first rung pays for the rest of the ladder.
        if (k == 0 && rungSt[k].fixPct >= AUTO_MIN_FIX && rungSt[k].nsMin >= AUTO_MIN_NS) break;
    }
    LOG.detail("rnx2rtkp finished in %.1f s (%zu pass(es))", rtkSw.s(), nrun);

    // Rank by fix ratio, break ties on mean ratio, and veto any rung that had to
    // drop below AUTO_MIN_NS satellites to get there. The whole ladder is ranked,
    // never stopped at the first improvement: the curve is not monotonic, because
    // RTKLIB's LAMBDA fixes the ambiguity set all-or-nothing, so WHICH satellites
    // are in the set swings the result more than how many.
    int best = -1;
    for (size_t k = 0; k < rungs.size(); ++k) {
        if (!rungSt[k].ok || rungSt[k].nsMin < AUTO_MIN_NS) continue;
        if (best < 0 || rungSt[k].fixPct > rungSt[best].fixPct ||
            (rungSt[k].fixPct == rungSt[best].fixPct &&
             rungSt[k].meanRatio > rungSt[best].meanRatio)) best = (int)k;
    }
    if (best < 0) {                      // no rung kept enough satellites
        for (size_t k = 0; k < rungs.size(); ++k) {
            if (!rungSt[k].ok) continue;
            if (best < 0 || rungSt[k].fixPct > rungSt[best].fixPct) best = (int)k;
        }
        if (best >= 0)
            LOG.warn("every elmask rung fell below %d satellites; taking %d deg anyway",
                     AUTO_MIN_NS, rungs[best]);
    }
    if (autoElmask) {
        if (best >= 0) {
            if (nrun > 1)
                LOG.detail("auto-elmask picked %d deg (fix %.1f%%, mean ratio %.1f,"
                           " sats %.1f avg / %d min)",
                           rungs[best], rungSt[best].fixPct, rungSt[best].meanRatio,
                           rungSt[best].nsMean, rungSt[best].nsMin);
            MoveFileExA(rungPos[best].c_str(), posf.c_str(), MOVEFILE_REPLACE_EXISTING);
        }
        for (size_t k = 0; k < rungs.size(); ++k)
            if ((int)k != best && !rungPos[k].empty()) DeleteFileA(rungPos[k].c_str());
    }

    if (effConf == tmpConf && !keepPos) DeleteFileA(tmpConf.c_str());

    if (GetFileAttributesA(posf.c_str()) == INVALID_FILE_ATTRIBUTES) {
        LOG.error("rnx2rtkp produced no solution (%s). Check inputs/config.", posf.c_str());
        return 1;
    }

    LOG.step("Converting the .pos to the 13-column rtk_std_analysis format");
    bool ok = convert(posf, out, fh, fv, sc);
    if (!keepPos) DeleteFileA(posf.c_str());
    else          LOG.detail("kept the intermediate solution: %s", posf.c_str());
    if (!ok) {
        LOG.error("conversion failed; giving up after %.1f s", runTime.s());
        return 1;
    }
    LOG.info("Done in %.1f s. Output: %s", runTime.s(), out.c_str());
    LOG.close();
    return 0;
}
