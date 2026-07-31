// rtkproc.cpp - single-exe wrapper around RTKLIB rnx2rtkp.exe
//   * normalizes input paths to absolute backslash form (works around the
//     Windows expath() bug where rnx2rtkp only splits paths on '\\')
//   * base coordinate: by default auto-computes the base station position by
//     AVERAGING a single-point (SPP) solution over the WHOLE base file
//     (batch-agnostic - not tied to any fixed duration). Options let you keep
//     the RINEX header, or supply a known coordinate.
//   * runs the kinematic RTK, then converts the .pos to the 13-column
//     rtk_std_analysis format with covariance inflation on the position std.
//
// Build (MinGW):
//   g++ -O2 -static -static-libgcc -static-libstdc++ -o rtkproc.exe rtkproc.cpp
//
// Usage:
//   rtkproc.exe <rover_obs> <base_obs> <nav> [nav2 ...]
//               [-o out.txt] [-c rtk.conf]
//               [--base-avg | --base-header | --base-llh lat lon h]
//               [--floor-h m] [--floor-v m] [--scale k] [--keep-pos]
//
// Base coordinate modes:
//   --base-avg    (default) average an SPP solution over the whole base file
//   --base-header use the base RINEX header APPROX POSITION as-is
//   --base-llh    use a known coordinate (deg deg m)
#include <windows.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>
using namespace std;

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
    if (!ok) return -1;
    WaitForSingleObject(pi.hProcess, INFINITE);
    DWORD rc = 0; GetExitCodeProcess(pi.hProcess, &rc);
    CloseHandle(pi.hProcess); CloseHandle(pi.hThread);
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
    (void)rc;
    FILE* f = fopen(tmpPos.c_str(), "r");
    if (!f) return false;
    vector<double> la, lo, hh; char line[2048];
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '%' || line[0] == '\n' || line[0] == '\r' || line[0] == '\0') continue;
        // fields: week tow lat lon h Q ns ...
        vector<double> v; char* ctx = NULL;
        for (char* t = strtok_r(line, " \t\r\n", &ctx); t; t = strtok_r(NULL, " \t\r\n", &ctx))
            v.push_back(atof(t));
        if (v.size() < 6) continue;
        int Q = (int)v[5];
        if (Q <= 0) continue;                       // no fix this epoch
        la.push_back(v[2]); lo.push_back(v[3]); hh.push_back(v[4]);
    }
    fclose(f);
    if (!keep) DeleteFileA(tmpPos.c_str());
    if (la.empty()) return false;
    lat = clipMean(la); lon = clipMean(lo); h = clipMean(hh);
    nused = (int)la.size();
    return true;
}

static double infl(double sd, double fl, double sc) { sd *= sc; return sqrt(sd*sd + fl*fl); }

static bool convert(const string& pos, const string& out,
                    double fh, double fv, double sc) {
    FILE* fi = fopen(pos.c_str(), "r");
    if (!fi) { fprintf(stderr, "cannot open %s\n", pos.c_str()); return false; }
    FILE* fo = fopen(out.c_str(), "w");
    if (!fo) { fprintf(stderr, "cannot write %s\n", out.c_str()); fclose(fi); return false; }
    char line[2048]; long nrec = 0;
    while (fgets(line, sizeof(line), fi)) {
        if (line[0] == '%' || line[0] == '\n' || line[0] == '\r' || line[0] == '\0') continue;
        vector<double> v; char* ctx = NULL;
        for (char* t = strtok_r(line, " \t\r\n", &ctx); t; t = strtok_r(NULL, " \t\r\n", &ctx))
            v.push_back(atof(t));
        if (v.size() < 13) continue;
        double tow = floor(v[1] + 0.5);
        double lat = v[2], lon = v[3], h = v[4];
        double sdn = v[7], sde = v[8], sdu = v[9];
        double vn = 0, ve = 0, vu = 0, svn = 0, sve = 0, svu = 0;
        if (v.size() >= 21) {
            vn = v[15]; ve = v[16]; vu = v[17];
            svn = v[18]; sve = v[19]; svu = v[20];
        }
        fprintf(fo, "%.6f %.10f %.10f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
                tow, lat, lon, h,
                infl(sdn, fh, sc), infl(sde, fh, sc), infl(sdu, fv, sc),
                vn, ve, vu, svn, sve, svu);
        ++nrec;
    }
    fclose(fi); fclose(fo);
    printf("converted %ld epochs -> %s  (floor_h=%.3f floor_v=%.3f scale=%.2f)\n",
           nrec, out.c_str(), fh, fv, sc);
    return nrec > 0;
}

// write an effective config = copy of base conf + appended ant2 base-coordinate override
static bool writeConfOverride(const string& baseConf, const string& outConf,
                              double lat, double lon, double h) {
    FILE* fi = fopen(baseConf.c_str(), "r");
    if (!fi) return false;
    FILE* fo = fopen(outConf.c_str(), "w");
    if (!fo) { fclose(fi); return false; }
    char buf[4096]; size_t n;
    while ((n = fread(buf, 1, sizeof buf, fi)) > 0) fwrite(buf, 1, n, fo);
    fclose(fi);
    fprintf(fo, "\n# --- base coordinate injected by rtkproc (SPP average) ---\n");
    fprintf(fo, "ant2-postype=llh\n");
    fprintf(fo, "ant2-pos1=%.9f\n", lat);
    fprintf(fo, "ant2-pos2=%.9f\n", lon);
    fprintf(fo, "ant2-pos3=%.4f\n", h);
    fclose(fo);
    return true;
}

enum BaseMode { BASE_AVG, BASE_HEADER, BASE_LLH };

int main(int argc, char** argv) {
    string conf, out;
    double fh = 0.010, fv = 0.015, sc = 1.0;
    bool keepPos = false;
    BaseMode bmode = BASE_AVG;
    double blat = 0, blon = 0, bh = 0;
    vector<string> inputs;
    for (int i = 1; i < argc; ++i) {
        string a = argv[i];
        if (a == "-o" && i + 1 < argc) out = argv[++i];
        else if (a == "-c" && i + 1 < argc) conf = argv[++i];
        else if (a == "--floor-h" && i + 1 < argc) fh = atof(argv[++i]);
        else if (a == "--floor-v" && i + 1 < argc) fv = atof(argv[++i]);
        else if (a == "--scale" && i + 1 < argc) sc = atof(argv[++i]);
        else if (a == "--keep-pos") keepPos = true;
        else if (a == "--base-avg") bmode = BASE_AVG;
        else if (a == "--base-header") bmode = BASE_HEADER;
        else if (a == "--base-llh" && i + 3 < argc) {
            bmode = BASE_LLH; blat = atof(argv[++i]); blon = atof(argv[++i]); bh = atof(argv[++i]);
        } else inputs.push_back(a);
    }
    if (inputs.size() < 3) {
        fprintf(stderr,
            "Usage: rtkproc <rover_obs> <base_obs> <nav> [nav2 ...]\n"
            "               [-o out.txt] [-c rtk.conf]\n"
            "               [--base-avg | --base-header | --base-llh lat lon h]\n"
            "               [--floor-h m] [--floor-v m] [--scale k] [--keep-pos]\n");
        return 1;
    }
    string dir = exeDir();
    if (conf.empty()) conf = dir + "rtk.conf";
    conf = fullPath(conf);
    string exe = dir + "rnx2rtkp.exe";
    if (GetFileAttributesA(exe.c_str()) == INVALID_FILE_ATTRIBUTES) {
        fprintf(stderr, "rnx2rtkp.exe not found next to rtkproc.exe (%s)\n", exe.c_str());
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

    // ---- decide base coordinate ----
    if (bmode == BASE_AVG) {
        double la, lo, h; int nep = 0;
        printf("base coordinate: averaging SPP over whole base file ...\n");
        if (baseSppAverage(exe, conf, base, navs, sppf, keepPos, la, lo, h, nep)) {
            printf("  base SPP average (%d epochs): lat=%.9f lon=%.9f h=%.4f m\n", nep, la, lo, h);
            if (writeConfOverride(conf, tmpConf, la, lo, h)) effConf = tmpConf;
            else fprintf(stderr, "  warning: could not write temp config, using RINEX header\n");
        } else {
            fprintf(stderr, "  warning: base SPP failed, falling back to RINEX header\n");
        }
    } else if (bmode == BASE_LLH) {
        printf("base coordinate: user-supplied lat=%.9f lon=%.9f h=%.4f m\n", blat, blon, bh);
        if (writeConfOverride(conf, tmpConf, blat, blon, bh)) effConf = tmpConf;
    } else {
        printf("base coordinate: RINEX header APPROX POSITION\n");
    }

    // ---- main kinematic RTK ----
    string cmd = quote(exe) + " -k " + quote(effConf);
    for (auto& s : inputs) cmd += " " + quote(s);
    cmd += " -o " + quote(posf);
    printf("running: rnx2rtkp kinematic RTK <%zu files> ...\n", inputs.size());
    int rc = runProc(cmd); (void)rc;

    if (effConf == tmpConf && !keepPos) DeleteFileA(tmpConf.c_str());

    if (GetFileAttributesA(posf.c_str()) == INVALID_FILE_ATTRIBUTES) {
        fprintf(stderr, "rnx2rtkp produced no solution. Check inputs/config.\n");
        return 1;
    }
    bool ok = convert(posf, out, fh, fv, sc);
    if (!keepPos) DeleteFileA(posf.c_str());
    if (!ok) return 1;
    printf("Done. Output: %s\n", out.c_str());
    return 0;
}
