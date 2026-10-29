/* MPC solve on the Orin: the gpu_solve.cu prototype (baseline, verbatim apart
   from the VALIDATE noise source) against the optimized kernel, on the same
   real line window and steer map.

     ./bench base K [period_us]       baseline latency
     ./bench fast K [period_us] [L]   optimized latency (L = lanes per sample)
     ./bench_val K                    (built with -DVALIDATE) same noise into
                                      both kernels, compare costs / elites / u

   Latency = host clock from "state known" to "elite mean on the host": launch,
   sync, and whatever host work the variant needs. period_us > 0 duty-cycles
   the loop like the 60 Hz pathfinder tick does. */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <ctime>
#include <algorithm>
#include <vector>
#include <cuda_runtime.h>
#include <curand_kernel.h>
#include "window.h"


/* mpc.py: edge_inner = corridor_half * edge_inner_frac, corridor_half =
   track_half_width - safety_margin - kart_half_width = 2.0 - 0.5 - 0.5025. */
#define EDGE_INNER 0.273315
#define ALAT_MAX   5.3
#define W_ALAT     0.0836

#define NMAX 64
#define MMAX 64
#define NK 17
#define CK(x) do { cudaError_t e_ = (x); if (e_ != cudaSuccess) { \
  fprintf(stderr, "%s:%d %s\n", __FILE__, __LINE__, cudaGetErrorString(e_)); exit(1); } } while (0)

/* ------------------------------------------------------------------------ */
/* Baseline: gpu_solve.cu solve_k                                            */
/* ------------------------------------------------------------------------ */
struct Line { double x[MMAX], y[MMAX], psi[MMAX], s[MMAX], vx[MMAX]; int m; };
struct Map  { double cmd[NK], wheel[NK]; int n; };

__constant__ Line c_line;
__constant__ Map  c_map;
__constant__ double c_umean_d[NMAX], c_umean_a[NMAX];

__device__ __forceinline__ double d_interp(double v, const double *xp,
                                           const double *fp, int n) {
  if (v <= xp[0]) return fp[0];
  if (v >= xp[n-1]) return fp[n-1];
  int lo = 0, hi = n - 1;
  while (hi - lo > 1) { int mid = (lo + hi) >> 1; if (xp[mid] <= v) lo = mid; else hi = mid; }
  double t = (v - xp[lo]) / (xp[hi] - xp[lo]);
  return fma(t, fp[hi] - fp[lo], fp[lo]);
}

__global__ void solve_k(int K, int N, double dt, double L, double alag,
                        double drmax, double smax, double x0, double y0,
                        double psi0, double v0, double vcap, double vtgt,
                        double sig_d, double sig_a, unsigned long long seed,
                        double *cost, double *cmd_o, double *dlt_o, double *acc_o,
                        const double *pre, double dprev) {
  int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= K) return;
#ifndef VALIDATE
  curandStatePhilox4_32_10_t st;
  curand_init(seed, k, 0, &st);
#endif

  double prev = dprev, x = x0, y = y0, psi = psi0, v = v0, da = dprev;
  double cd = 0, ch = 0, cv = 0, cdel = 0, cdr = 0, ca = 0, ce = 0, cal = 0;
  double s0 = 0, s1 = 0, dl = 0, hl = 0;
  const Line &ln = c_line; const Map &mp = c_map;

  for (int i = 0; i < N; ++i) {
#ifdef VALIDATE
    double2 g = make_double2(pre[(k*N+i)*2], pre[(k*N+i)*2+1]);
#else
    double2 g = curand_normal2_double(&st);
#endif
    double nd = (k == 0) ? 0.0 : sig_d * g.x;
    double na = (k == 0) ? 0.0 : sig_a * g.y;
    double want = d_interp(c_umean_d[i] + nd, mp.wheel, mp.cmd, mp.n);
    double lo = prev - drmax, hi = prev + drmax;
    lo = fmax(lo, -smax); hi = fmin(hi, smax);
    double c = fmin(fmax(want, lo), hi);
    double dd = c - prev; prev = c;
    double a = fmin(fmax(c_umean_a[i] + na, -3.0), 5.3);
    double w = d_interp(c, mp.cmd, mp.wheel, mp.n);
    cmd_o[k*N+i] = c; dlt_o[k*N+i] = w; acc_o[k*N+i] = a;

    da = fma(alag, c - da, da);
    double sp_, cp_; sincos(psi, &sp_, &cp_);
    x = fma(dt * v, cp_, x); y = fma(dt * v, sp_, y);
    psi += dt * v / L * tan(d_interp(da, mp.cmd, mp.wheel, mp.n));
    v = fmin(fmax(v + dt * a, 0.0), vcap);
    { double al = v * v * tan(w) / L;
      double ax = fmax(0.0, fabs(al) - ALAT_MAX); cal = fma(ax, ax, cal); }

    int jm = 0; double b2 = 1e300;
    for (int j = 0; j < ln.m; ++j) {
      double ex = x - ln.x[j], ey = y - ln.y[j], q = fma(ex, ex, ey*ey);
      if (q < b2) { b2 = q; jm = j; }
    }
    double ex = x - ln.x[jm], ey = y - ln.y[jm];
    double sps, cps; sincos(ln.psi[jm], &sps, &cps);
    double s = ln.s[jm] + ex*cps + ey*sps, d = fma(-ex, sps, ey*cps);
    double pe = psi - ln.psi[jm];
    pe = pe - 2.0*M_PI*rint(pe/(2.0*M_PI));
    double ve = v - fmin(ln.vx[jm], vtgt);
    double ee = fmax(0.0, fabs(d) - EDGE_INNER);
    cd = fma(d, d, cd); ch = fma(pe, pe, ch); cv = fma(ve, ve, cv);
    cdel = fma(c, c, cdel); cdr = fma(dd, dd, cdr); ca = fma(a, a, ca);
    ce = fma(ee, ee, ce);
    if (i == 0) s0 = s;
    s1 = s; dl = d; hl = pe;
  }
  cost[k] = 8.85*cd + 2.97*ch + 30.69*cv + 0.05*cdel + 0.12*cdr + 2.0*ca
          + 1456.58*ce - 6.23*(s1 - s0) + 11.35*hl*hl + W_ALAT*cal;
}

/* ------------------------------------------------------------------------ */
/* Optimized                                                                 */
/* ------------------------------------------------------------------------ */
#define NELITE 3
#define BLKMAX 256
#define MS 42                 // search points, compile-time; >= proj_back + proj_fwd

/* np.interp as a sum of clamped ramps, one per segment:
     f(v) = fp0 + sum_j dy_j * sat((v - xp_j) / w_j)
   with the end clamps kept explicit. A zero-width segment (repeated knot, as
   at both ends of the steer map) gets iw = 2^100, which turns its ramp into
   the step the bisection produces there. No memory access, no dependent
   loads: every coefficient is a constant-bank operand. */
struct Hinge { float xp0, fp0, xpN, fpN, xp[NK - 1], iw[NK - 1], dy[NK - 1]; };

/* Everything a tick needs rides in the launch itself, so a new line window or
   warm start costs no extra copy. Line coordinates are relative to the kart
   (kart at the origin), which is what keeps fp32 exact enough. */
struct P {
  int K, N, M, prof; unsigned seq; float dprev; float amin, amax; float invL; float dt, dvL, alag, drmax, smax, psi0, v0, vcap, vtgt, sig_d, sig_a;
  unsigned int seed_lo, seed_hi;
  float umd[NMAX], uma[NMAX];
  float lx[MS], ly[MS], lh[MS];                  // search: key = h - x*lx - y*ly
  float lc[MMAX], ls_[MMAX], lpsi[MMAX], lss[MMAX], lvx[MMAX];
  float mc[NK], mw[NK], kc2w[NK], kw2c[NK];     // map knots + segment slopes
  Hinge hc2w, hw2c;
};
struct Cand { float c; int i; };
struct Scratch { unsigned int count; Cand cand[3 * 1024]; };
struct Out { unsigned seq; float best; int best_i; int elite[NELITE]; float cmd0; float ud[NMAX], ua[NMAX]; };

__device__ __forceinline__ uint4 philox(uint4 c, uint2 k) {
  #pragma unroll
  for (int r = 0; r < 10; ++r) {
    unsigned lo0 = 0xD2511F53u * c.x, hi0 = __umulhi(0xD2511F53u, c.x);
    unsigned lo1 = 0xCD9E8D57u * c.z, hi1 = __umulhi(0xCD9E8D57u, c.z);
    c = make_uint4(hi1 ^ c.y ^ k.x, lo1, hi0 ^ c.w ^ k.y, lo0);
    k.x += 0x9E3779B9u; k.y += 0xBB67AE85u;
  }
  return c;
}
/* Four standard normals for steps (2p, 2p+1) of sample k: two Box-Muller pairs. */
__device__ __forceinline__ float4 normals4(const P &p, int k, int pair,
                                           const float *pre) {
#ifdef VALIDATE
  int i = 2 * pair, N = p.N;
  if (i >= N) return make_float4(0.f, 0.f, 0.f, 0.f);
  float n2 = 0.f, n3 = 0.f;
  if (i + 1 < N) { n2 = pre[(k*N+i+1)*2]; n3 = pre[(k*N+i+1)*2+1]; }
  return make_float4(pre[(k*N+i)*2], pre[(k*N+i)*2+1], n2, n3);
#else
  uint4 r = philox(make_uint4(pair, k, 0, 0), make_uint2(p.seed_lo, p.seed_hi));
  float u1 = ((r.x >> 8) + 0.5f) * 5.9604645e-8f, u2 = (r.y >> 8) * 5.9604645e-8f;
  float u3 = ((r.z >> 8) + 0.5f) * 5.9604645e-8f, u4 = (r.w >> 8) * 5.9604645e-8f;
  float ra = sqrtf(-2.f * __logf(u1)), rb = sqrtf(-2.f * __logf(u3));
  float sa, ca, sb, cb;
  __sincosf(6.2831853f * u2, &sa, &ca); __sincosf(6.2831853f * u4, &sb, &cb);
  return make_float4(ra * ca, ra * sa, rb * cb, rb * sb);
#endif
}

/* np.interp over NK non-decreasing knots in shared memory, branch-free: 4 steps
   of binary lifting find the same segment as the baseline's bisection. */
__device__ __forceinline__ float interp(float v, const float *xp, const float *fp,
                                        const float *kk) {
  int lo = 0;
  lo += (xp[lo + 8] <= v) ? 8 : 0;
  lo += (xp[lo + 4] <= v) ? 4 : 0;
  lo += (xp[lo + 2] <= v) ? 2 : 0;
  lo += (xp[lo + 1] <= v) ? 1 : 0;
  float r = fmaf(v - xp[lo], kk[lo], fp[lo]);
  r = (v <= xp[0]) ? fp[0] : r;
  return (v >= xp[NK - 1]) ? fp[NK - 1] : r;
}
__device__ __forceinline__ float hinge(float v, const Hinge &h) {
  float r0 = h.fp0, r1 = 0.f;
  #pragma unroll
  for (int j = 0; j < NK - 1; ++j) {
    float t = __saturatef((v - h.xp[j]) * h.iw[j]);
    if (j & 1) r1 = fmaf(t, h.dy[j], r1); else r0 = fmaf(t, h.dy[j], r0);
  }
  float r = r0 + r1;
  r = (v <= h.xp0) ? h.fp0 : r;
  return (v >= h.xpN) ? h.fpN : r;
}

struct Sh {
  float4 pt[MMAX];            // lx, ly, lh, -
  float4 aux[MMAX];           // cos psi, sin psi, psi, s
  float vx[MMAX];
  float mc[NK], mw[NK], kc2w[NK], kw2c[NK];
  float ew[NELITE][NMAX], ea[NELITE][NMAX], ec0[NELITE];
  Cand wc[BLKMAX / 32 * NELITE];
  unsigned cnt;
};

template <bool HG> __device__ __forceinline__ float w2c(float v, const P &p, const Sh &s) {
  return HG ? hinge(v, p.hw2c) : interp(v, s.mw, s.mc, s.kw2c);
}
template <bool HG> __device__ __forceinline__ float c2w(float v, const P &p, const Sh &s) {
  return HG ? hinge(v, p.hc2w) : interp(v, s.mc, s.mw, s.kc2w);
}

__device__ __forceinline__ Cand cmin(Cand a, Cand b) {
  return (b.c < a.c || (b.c == a.c && b.i < a.i)) ? b : a;
}
/* Warp-wide argmin; every lane gets the winner. */
__device__ __forceinline__ Cand warp_argmin(Cand v) {
  #pragma unroll
  for (int o = 16; o; o >>= 1) {
    Cand w; w.c = __shfl_xor_sync(0xffffffffu, v.c, o); w.i = __shfl_xor_sync(0xffffffffu, v.i, o);
    v = cmin(v, w);
  }
  return v;
}

template <int L, bool HG>
__global__ void __launch_bounds__(BLKMAX) fast_k(const __grid_constant__ P p,
                                                 Scratch *sc, Out *out, float *cost_dbg,
                                                 const float *pre) {
  __shared__ Sh s;
  for (int j = threadIdx.x; j < MMAX; j += blockDim.x) {
    bool in = j < p.M;
    s.pt[j] = in ? make_float4(p.lx[j], p.ly[j], p.lh[j], 0.f) : make_float4(0.f, 0.f, 1e30f, 0.f);
    s.aux[j] = make_float4(p.lc[j], p.ls_[j], p.lpsi[j], p.lss[j]);
    s.vx[j] = fminf(p.lvx[j], p.vtgt);
  }
  if (threadIdx.x == 0) s.cnt = 0;
  if (!HG && threadIdx.x < NK) {
    int j = threadIdx.x;
    s.mc[j] = p.mc[j]; s.mw[j] = p.mw[j]; s.kc2w[j] = p.kc2w[j]; s.kw2c[j] = p.kw2c[j];
  }
  __syncthreads();
  if (p.prof == 1) { if (blockIdx.x == 0 && threadIdx.x == 0) out->best = 0.f; return; }

  const int g = blockIdx.x * blockDim.x + threadIdx.x;
  const int lane = g & (L - 1);
  const int kraw = g / L;
  const int k = min(kraw, p.K - 1);   // whole warps run the loop for the shuffles
  const float dt = p.dt;
  const float sd = (k == 0) ? 0.f : p.sig_d, sa = (k == 0) ? 0.f : p.sig_a;

  float prev = p.dprev, x = 0.f, y = 0.f, psi = p.psi0, v = p.v0, da = p.dprev;
  float cd = 0, ch = 0, cv = 0, cdel = 0, cdr = 0, ca = 0, ce = 0, cal = 0;
  float s0 = 0, s1 = 0, dl = 0, hl = 0;

  float4 gg = normals4(p, k, 0, pre);
  for (int i = 0; i < p.N; i += 2) {
    float4 gn = normals4(p, k, (i >> 1) + 1, pre);   // next pair, off the critical path
    #pragma unroll
    for (int h = 0; h < 2; ++h) {
      const int ii = i + h;
      if (ii >= p.N) break;
      float nd = sd * (h ? gg.z : gg.x), na = sa * (h ? gg.w : gg.y);
      float want = w2c<HG>(p.umd[ii] + nd, p, s);
      float lo = fmaxf(prev - p.drmax, -p.smax), hi = fminf(prev + p.drmax, p.smax);
      float c = fminf(fmaxf(want, lo), hi);
      float dd = c - prev; prev = c;
      float a = fminf(fmaxf(p.uma[ii] + na, p.amin), p.amax);

      da = fmaf(p.alag, c - da, da);
      float sp_, cp_; __sincosf(psi, &sp_, &cp_);
      float dv = dt * v;
      x = fmaf(dv, cp_, x); y = fmaf(dv, sp_, y);
      float sw, cw; __sincosf(c2w<HG>(da, p, s), &sw, &cw);
      psi = fmaf(v * p.dvL, __fdividef(sw, cw), psi);
      v = fminf(fmaxf(fmaf(dt, a, v), 0.f), p.vcap);
      { float ws, wc2; __sincosf(c2w<HG>(c, p, s), &ws, &wc2);
        float al = v * v * __fdividef(ws, wc2) * p.invL;
        float ax = fmaxf(0.f, fabsf(al) - (float)ALAT_MAX);
        cal = fmaf(ax, ax, cal); }

      /* nearest line point; four independent running minima so the compare
         chain is a quarter as long, merged with ties to the lowest index */
      Cand m[4] = {{INFINITY, 0}, {INFINITY, 1}, {INFINITY, 2}, {INFINITY, 3}};
      if (L == 1) {
        #pragma unroll
        for (int j = 0; j < MS; ++j) {        // line points as constant operands
          float key = fmaf(-y, p.ly[j], fmaf(-x, p.lx[j], p.lh[j]));
          if (key < m[j & 3].c) { m[j & 3].c = key; m[j & 3].i = j; }
        }
      } else {
        #pragma unroll
        for (int t = 0; t < (MS + L - 1) / L; ++t) {
          int j = lane + t * L;
          float4 q = s.pt[j];
          float key = fmaf(-x, q.x, fmaf(-y, q.y, q.z));
          if (key < m[t & 3].c) { m[t & 3].c = key; m[t & 3].i = j; }
        }
      }
      Cand bm = cmin(cmin(m[0], m[1]), cmin(m[2], m[3]));
      #pragma unroll
      for (int o = L / 2; o; o >>= 1) {
        Cand w; w.c = __shfl_xor_sync(0xffffffffu, bm.c, o); w.i = __shfl_xor_sync(0xffffffffu, bm.i, o);
        bm = cmin(bm, w);
      }
      const int jm = bm.i;

      float4 q = s.pt[jm], au = s.aux[jm];
      float ex = x - q.x, ey = y - q.y;
      float sv = fmaf(ex, au.x, fmaf(ey, au.y, au.w));
      float d = fmaf(-ex, au.y, ey * au.x);
      float pe = psi - au.z;
      pe = fmaf(-6.2831853f, rintf(pe * 0.15915494f), pe);
      float ve = v - s.vx[jm];
      float ee = fmaxf(0.f, fabsf(d) - (float)EDGE_INNER);
      cd = fmaf(d, d, cd); ch = fmaf(pe, pe, ch); cv = fmaf(ve, ve, cv);
      cdel = fmaf(c, c, cdel); cdr = fmaf(dd, dd, cdr); ca = fmaf(a, a, ca);
      ce = fmaf(ee, ee, ce);
      if (ii == 0) s0 = sv;
      s1 = sv; dl = d; hl = pe;
    }
    gg = gn;
  }
  float cost = 8.85f*cd + 2.97f*ch + 30.69f*cv + 0.05f*cdel + 0.12f*cdr + 2.f*ca
             + 1456.58f*ce - 6.23f*(s1 - s0) + 11.35f*hl*hl + (float)W_ALAT*cal;
  bool owner = lane == 0 && kraw < p.K;
  if (cost_dbg && owner) cost_dbg[kraw] = cost;

  /* Top-3 in two barrier-free levels. Each warp shuffles out its top-3 into
     shared memory; the last warp of the block to finish merges the block's
     candidates and publishes three; the last block's merging warp merges the
     grid's and rebuilds the elites. A warp retires the moment it is done. */
  const unsigned l32 = threadIdx.x & 31;
  const int nwb = blockDim.x >> 5, wib = threadIdx.x >> 5;
  Cand mine = owner ? Cand{cost, kraw} : Cand{INFINITY, 0x7fffffff};
  #pragma unroll
  for (int r = 0; r < NELITE; ++r) {
    Cand w = warp_argmin(mine);
    if (l32 == 0) s.wc[wib * NELITE + r] = w;
    if (mine.i == w.i) mine = Cand{INFINITY, 0x7fffffff};
  }
  __threadfence_block();
  unsigned done = 0;
  if (l32 == 0) done = atomicAdd(&s.cnt, 1u);
  if (__shfl_sync(0xffffffffu, done, 0) != (unsigned)nwb - 1) return;
  __threadfence_block();
  {
    Cand c{INFINITY, 0x7fffffff};
    if (l32 < nwb * NELITE) { c.c = ((volatile float *)&s.wc[l32].c)[0]; c.i = ((volatile int *)&s.wc[l32].i)[0]; }
    #pragma unroll
    for (int r = 0; r < NELITE; ++r) {
      Cand w = warp_argmin(c);
      if (l32 == 0 && p.prof != 2) sc->cand[blockIdx.x * NELITE + r] = w;
      if (c.i == w.i) c = Cand{INFINITY, 0x7fffffff};
    }
  }
  __threadfence();
  if (l32 == 0) done = atomicAdd(&sc->count, 1u);
  if (__shfl_sync(0xffffffffu, done, 0) != gridDim.x - 1) return;
  if (p.prof == 2) { if (l32 == 0) { sc->count = 0; __threadfence_system(); out->seq = p.seq; } return; }
  __threadfence();

  /* each lane keeps a sorted top-3 of its stride, then three warp argmins */
  const int C = gridDim.x * NELITE;
  Cand t0{INFINITY, 0x7fffffff}, t1 = t0, t2 = t0;
  for (int j = l32; j < C; j += 32) {
    Cand c; c.c = __ldcg(&sc->cand[j].c); c.i = __ldcg(&sc->cand[j].i);
    if (cmin(c, t2).i == c.i && c.i != t2.i) {
      if (cmin(c, t1).i == c.i) {
        t2 = t1;
        if (cmin(c, t0).i == c.i) { t1 = t0; t0 = c; } else t1 = c;
      } else t2 = c;
    }
  }
  int eid[NELITE]; float best = 0.f;
  #pragma unroll
  for (int r = 0; r < NELITE; ++r) {
    Cand w = warp_argmin(t0);
    if (t0.i == w.i) { t0 = t1; t1 = t2; t2 = Cand{INFINITY, 0x7fffffff}; }
    eid[r] = w.i;
    if (r == 0) best = w.c;
  }

  /* Rebuild the elites' controls instead of storing K x N of them. Everything
     but the slew clamp is per step, so it runs one lane per (elite, step);
     only the clamp's short scan is serial. */
  const int EN = NELITE * p.N;
  for (int t = l32; t < EN; t += 32) {
    int e = t / p.N, i = t - e * p.N, ke = e == 0 ? eid[0] : (e == 1 ? eid[1] : eid[2]);
    float4 q = normals4(p, ke, i >> 1, pre);
    float esd = ke == 0 ? 0.f : p.sig_d, esa = ke == 0 ? 0.f : p.sig_a;
    s.ew[e][i] = w2c<HG>(p.umd[i] + esd * ((i & 1) ? q.z : q.x), p, s);
    s.ea[e][i] = fminf(fmaxf(p.uma[i] + esa * ((i & 1) ? q.w : q.y), -3.f), 5.3f);
  }
  __syncwarp();
  if (l32 < NELITE) {
    float pv = 0.f;
    for (int i = 0; i < p.N; ++i) {
      float lo = fmaxf(pv - p.drmax, -p.smax), hi = fminf(pv + p.drmax, p.smax);
      pv = fminf(fmaxf(s.ew[l32][i], lo), hi);
      s.ew[l32][i] = pv;
    }
    s.ec0[l32] = s.ew[l32][0];
  }
  __syncwarp();
  for (int t = l32; t < EN; t += 32) {
    int e = t / p.N, i = t - e * p.N;
    s.ew[e][i] = c2w<HG>(s.ew[e][i], p, s);
  }
  __syncwarp();
  for (int i = l32; i < p.N; i += 32) {
    float w = 0.f, a = 0.f;
    #pragma unroll
    for (int e = 0; e < NELITE; ++e) { w += s.ew[e][i]; a += s.ea[e][i]; }
    out->ud[i] = w * (1.f / NELITE); out->ua[i] = a * (1.f / NELITE);
  }
  if (l32 == 0) {
    float c0 = 0.f;
    for (int e = 0; e < NELITE; ++e) { c0 += s.ec0[e]; out->elite[e] = eid[e]; }
    out->cmd0 = c0 * (1.f / NELITE); out->best = best; out->best_i = eid[0];
    sc->count = 0;
  }
  __syncwarp();
  if (l32 == 0) { __threadfence_system(); *(volatile unsigned *)&out->seq = p.seq; }
}


/* ---------------------------------------------------------------------- */
/* Elite mean over round(EL_FRAC * K) samples                              */
/* ---------------------------------------------------------------------- */
#define EL_FRAC 0.1
static inline int n_elite_for(int K) {
  int n = (int)(EL_FRAC * (double)K + 0.5);
  return n < 1 ? 1 : n;
}

/* One thread per sample. A selected sample replays its own noise and slew
   clamp (the clamp is serial in i, so it stays on one thread) and adds its
   sequence into the block's accumulator; each block then folds into the
   output once. Ties at the threshold are taken by a counter so the number
   averaged is exactly n_elite, matching argpartition. */
template <bool HG>
__global__ void __launch_bounds__(256) elite_k(const __grid_constant__ P p,
                                               const float *__restrict__ costs,
                                               float thresh, int n_take_eq,
                                               float *acc, unsigned *tie,
                                               const float *pre, Out *out,
                                               unsigned *gcount, int ne,
                                               unsigned seq) {
  __shared__ Sh s;
  __shared__ float sud[NMAX], sua[NMAX], sc0;
  if (!HG && threadIdx.x < NK) {
    int j = threadIdx.x;
    s.mc[j] = p.mc[j]; s.mw[j] = p.mw[j]; s.kc2w[j] = p.kc2w[j]; s.kw2c[j] = p.kw2c[j];
  }
  for (int i = threadIdx.x; i < p.N; i += blockDim.x) { sud[i] = 0.f; sua[i] = 0.f; }
  if (threadIdx.x == 0) sc0 = 0.f;
  __syncthreads();

  const int k = blockIdx.x * blockDim.x + threadIdx.x;
  bool sel = false;
  if (k < p.K) {
    float c = costs[k];
    if (c < thresh) sel = true;
    else if (c == thresh && n_take_eq > 0)
      sel = atomicAdd(tie, 1u) < (unsigned)n_take_eq;
  }
  if (sel) {
    const float sd = k == 0 ? 0.f : p.sig_d, sa = k == 0 ? 0.f : p.sig_a;
    float prev = p.dprev;
    for (int i = 0; i < p.N; ++i) {
      float4 q = normals4(p, k, i >> 1, pre);
      float nd = sd * ((i & 1) ? q.z : q.x), na = sa * ((i & 1) ? q.w : q.y);
      float want = w2c<HG>(p.umd[i] + nd, p, s);
      float lo = fmaxf(prev - p.drmax, -p.smax), hi = fminf(prev + p.drmax, p.smax);
      float c = fminf(fmaxf(want, lo), hi);
      prev = c;
      atomicAdd(&sud[i], c2w<HG>(c, p, s));
      atomicAdd(&sua[i], fminf(fmaxf(p.uma[i] + na, p.amin), p.amax));
      if (i == 0) atomicAdd(&sc0, c);
    }
  }
  __syncthreads();
  for (int i = threadIdx.x; i < p.N; i += blockDim.x) {
    atomicAdd(&acc[i], sud[i]);
    atomicAdd(&acc[NMAX + i], sua[i]);
  }
  if (threadIdx.x == 0) atomicAdd(&acc[2 * NMAX], sc0);
  if (!out) return;
  __threadfence();
  __shared__ bool last;
  if (threadIdx.x == 0) last = atomicAdd(gcount, 1u) == gridDim.x - 1;
  __syncthreads();
  if (!last) return;
  const float inv = 1.0f / (float)ne;
  for (int i = threadIdx.x; i < p.N; i += blockDim.x) {
    out->ud[i] = acc[i] * inv; out->ua[i] = acc[NMAX + i] * inv;
    acc[i] = 0.f; acc[NMAX + i] = 0.f;
  }
  __syncthreads();
  if (threadIdx.x == 0) {
    out->cmd0 = acc[2 * NMAX] * inv;
    acc[2 * NMAX] = 0.f; *tie = 0u; *gcount = 0u;
    __threadfence_system();
    *(volatile unsigned *)&out->seq = seq;
  }
}

/* threshold, tie count, best and argmin in one host pass over the costs */
struct Pick { float thresh; int n_take_eq; float best; int best_i; };
static Pick pick_elites(const float *costs, int K, int n_elite,
                        std::vector<float> &scratch) {
  scratch.assign(costs, costs + K);
  std::nth_element(scratch.begin(), scratch.begin() + (n_elite - 1), scratch.end());
  Pick r; r.thresh = scratch[n_elite - 1];
  int below = 0; r.best = costs[0]; r.best_i = 0;
  for (int k = 0; k < K; ++k) {
    if (costs[k] < r.thresh) ++below;
    if (costs[k] < r.best) { r.best = costs[k]; r.best_i = k; }
  }
  r.n_take_eq = n_elite - below;
  return r;
}

/* ------------------------------------------------------------------------ */
/* Host                                                                      */
/* ------------------------------------------------------------------------ */
static const int N = 40;
static const double DT = 0.05, WB = 1.05, VCAP = 12.0, VTGT = 6.0;
static const double SIG_D = 0.0873, SIG_A = 1.43;
static const double DRMAX = 33.7 * M_PI / 180.0 * 0.05, SMAX = 60 * M_PI / 180.0;
static const double ALAG = 0.05 / (0.17 + 0.05);
static const unsigned long long SEED = 12345ULL;

static double now_us() { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1e6 + t.tv_nsec * 1e-3; }
static void gap_until(double t0, double per) {
  double left = per - (now_us() - t0);
  if (left > 50.0) { timespec r{0, (long)((left - 30.0) * 1000.0)}; nanosleep(&r, nullptr); }
  while (now_us() - t0 < per) {}
}

static void setup_base() {
  Line ln; ln.m = WIN_M;
  for (int i = 0; i < WIN_M; ++i) { ln.x[i] = WIN_X[i]; ln.y[i] = WIN_Y[i]; ln.psi[i] = WIN_PSI[i]; ln.s[i] = WIN_S[i]; ln.vx[i] = WIN_VX[i]; }
  Map mp; mp.n = NK;
  for (int i = 0; i < NK; ++i) { mp.cmd[i] = MAP_CMD[i]; mp.wheel[i] = MAP_WHEEL[i]; }
  double ud[NMAX] = {0}, ua[NMAX] = {0};
  for (int i = 0; i < N; ++i) ud[i] = UMEAN_D[i];
  CK(cudaMemcpyToSymbol(c_line, &ln, sizeof(Line)));
  CK(cudaMemcpyToSymbol(c_map, &mp, sizeof(Map)));
  CK(cudaMemcpyToSymbol(c_umean_d, ud, sizeof(ud)));
  CK(cudaMemcpyToSymbol(c_umean_a, ua, sizeof(ua)));
}

static void fill_p(P &p, int K) {
  memset(&p, 0, sizeof(p));
  p.K = K; p.N = N; p.M = WIN_M; p.dt = DT; p.dvL = DT / WB; p.invL = 1.0f / WB; p.alag = ALAG;
  p.drmax = DRMAX; p.smax = SMAX; p.v0 = KART0[3]; p.vcap = VCAP; p.vtgt = VTGT;
  p.amin = -3.f; p.amax = 5.3f;
  p.sig_d = SIG_D; p.sig_a = SIG_A; p.seed_lo = (unsigned)SEED; p.seed_hi = (unsigned)(SEED >> 32);
  double x0 = KART0[0], y0 = KART0[1];
  p.psi0 = (float)KART0[2];
  for (int i = 0; i < N; ++i) { p.umd[i] = UMEAN_D[i]; p.uma[i] = 0.f; }
  for (int j = 0; j < WIN_M; ++j) {
    double rx = WIN_X[j] - x0, ry = WIN_Y[j] - y0;
    p.lx[j] = rx; p.ly[j] = ry; p.lh[j] = 0.5 * (rx * rx + ry * ry);
    p.lc[j] = cos(WIN_PSI[j]); p.ls_[j] = sin(WIN_PSI[j]); p.lpsi[j] = WIN_PSI[j];
    p.lss[j] = WIN_S[j]; p.lvx[j] = WIN_VX[j];
  }
  for (int j = WIN_M; j < MS; ++j) p.lh[j] = 1e30f;
  for (int j = 0; j < NK; ++j) { p.mc[j] = MAP_CMD[j]; p.mw[j] = MAP_WHEEL[j]; }
  auto hinge_of = [](Hinge &h, const double *xp, const double *fp) {
    h.xp0 = xp[0]; h.fp0 = fp[0]; h.xpN = xp[NK-1]; h.fpN = fp[NK-1];
    for (int j = 0; j + 1 < NK; ++j) {
      double w = xp[j+1] - xp[j];
      h.xp[j] = xp[j]; h.dy[j] = fp[j+1] - fp[j];
      h.iw[j] = w > 0 ? 1.0 / w : 0x1p100;      // repeated knot: a step
    }
  };
  hinge_of(p.hc2w, MAP_CMD, MAP_WHEEL); hinge_of(p.hw2c, MAP_WHEEL, MAP_CMD);
  for (int j = 0; j + 1 < NK; ++j) {
    double dc = MAP_CMD[j+1] - MAP_CMD[j], dw = MAP_WHEEL[j+1] - MAP_WHEEL[j];
    p.kc2w[j] = dc != 0 ? dw / dc : 0.0; p.kw2c[j] = dw != 0 ? dc / dw : 0.0;
  }
}

typedef void (*FastFn)(P, Scratch *, Out *, float *, const float *);
template <bool HG> static FastFn pick_t(int L) {
  switch (L) {
    case 1: return fast_k<1, HG>;  case 2: return fast_k<2, HG>;  case 4: return fast_k<4, HG>;
    case 8: return fast_k<8, HG>;  case 16: return fast_k<16, HG>; default: return fast_k<32, HG>;
  }
}
static FastFn pick(int L, bool hg) { return hg ? pick_t<true>(L) : pick_t<false>(L); }
/* Block size: 256 unless that leaves SMs idle. */
static int pick_blk(long threads) { int b = BLKMAX; while (b > 64 && threads / b < 16) b >>= 1; return b; }
/* Lanes per sample: enough threads to fill the 8 SMs, no more. */
/* One lane per sample: at K >= 1024 splitting the search across lanes costs
   more issue slots than the latency it hides (swept L = 1..32). */
static int auto_lanes(int) { return 1; }

static void report(const char *tag, int K, int extra, std::vector<double> &ts) {
  std::sort(ts.begin(), ts.end());
  size_t R = ts.size();
  printf("%s K=%-5d %-3d med %8.1f  p95 %8.1f  p99 %8.1f  max %8.1f us\n", tag, K, extra,
         ts[R / 2], ts[(size_t)(0.95 * R)], ts[(size_t)(0.99 * R)], ts[R - 1]);
}


/* ---------------------------------------------------------------------- */
/* C API                                                                   */
/* ---------------------------------------------------------------------- */
#ifdef LIBRARY
struct Ctx {
  int K, N, L, blk, grid; bool hg;
  P p; Scratch *sc; Out *out, *out_d;
  float *costs, *costs_d, *accd; unsigned *tie, *gcnt;
  cudaStream_t st; unsigned tick;
  std::vector<float> scratch;
};

extern "C" int mpc_cuda_available(void) {
  int n = 0;
  if (cudaGetDeviceCount(&n) != cudaSuccess || n < 1) return 0;
  void *q = nullptr;
  if (cudaMalloc(&q, 16) != cudaSuccess || !q) return 0;
  cudaFree(q);
  return 1;
}

/* map knots in radians, as mpc.py holds them */
extern "C" Ctx *mpc_cuda_init(int K, int Nh, const double *map_cmd,
                              const double *map_wheel, int nk,
                              double dt, double wheelbase, double tau,
                              double rate_max, double steer_max,
                              double sig_d, double sig_a,
                              double a_min, double a_max) {
  if (Nh > NMAX || nk != NK) return nullptr;
  Ctx *c = new Ctx();
  c->K = K; c->N = Nh; c->L = 1; c->hg = false;
  c->blk = pick_blk((long)K * c->L);
  c->grid = (int)(((long)K * c->L + c->blk - 1) / c->blk);
  /* prof=2 never touches Scratch.cand, so only the block counter bounds us */
  if (c->grid > 65535) { delete c; return nullptr; }
  cudaSetDeviceFlags(cudaDeviceScheduleSpin);
  cudaFree(0);
  cudaStreamCreate(&c->st);
  cudaMalloc(&c->sc, sizeof(Scratch)); cudaMemset(c->sc, 0, sizeof(Scratch));
  cudaHostAlloc(&c->out, sizeof(Out), cudaHostAllocMapped);
  cudaHostGetDevicePointer((void **)&c->out_d, c->out, 0);
  cudaHostAlloc(&c->costs, K * sizeof(float), cudaHostAllocMapped);
  cudaHostGetDevicePointer((void **)&c->costs_d, c->costs, 0);
  cudaMalloc(&c->accd, (2 * NMAX + 1) * sizeof(float));
  cudaMemset(c->accd, 0, (2 * NMAX + 1) * sizeof(float));
  cudaMalloc(&c->tie, sizeof(unsigned)); cudaMemset(c->tie, 0, sizeof(unsigned));
  cudaMalloc(&c->gcnt, sizeof(unsigned)); cudaMemset(c->gcnt, 0, sizeof(unsigned));
  c->out->seq = 0; c->tick = 0;
  memset(&c->p, 0, sizeof(P));
  P &p = c->p;
  p.K = K; p.N = Nh; p.dt = dt; p.dvL = dt / wheelbase; p.invL = 1.0 / wheelbase;
  p.alag = tau > 0.0 ? dt / (tau + dt) : 1.0;
  p.drmax = rate_max * dt; p.smax = steer_max;
  p.sig_d = sig_d; p.sig_a = sig_a; p.amin = a_min; p.amax = a_max;
  for (int j = 0; j < NK; ++j) { p.mc[j] = map_cmd[j]; p.mw[j] = map_wheel[j]; }
  for (int j = 0; j + 1 < NK; ++j) {
    double dc = map_cmd[j+1] - map_cmd[j], dw = map_wheel[j+1] - map_wheel[j];
    p.kc2w[j] = dc != 0 ? dw / dc : 0.0; p.kw2c[j] = dw != 0 ? dc / dw : 0.0;
  }
  return c;
}

extern "C" void mpc_cuda_free(Ctx *c) {
  if (!c) return;
  cudaFree(c->sc); cudaFreeHost(c->out); cudaFreeHost(c->costs);
  cudaFree(c->accd); cudaFree(c->tie); cudaFree(c->gcnt);
  cudaStreamDestroy(c->st); delete c;
}

/* One tick. Window arrays are absolute; the kernel works kart-relative, so the
   pose is subtracted here exactly as fill_p did. Returns 0 on success. */
extern "C" int mpc_cuda_solve(Ctx *c, double x0, double y0, double yaw0,
                              double v0, double dprev, double vcap, double vtgt,
                              const double *wx, const double *wy,
                              const double *wpsi, const double *ws,
                              const double *wvx, int M,
                              const double *umd, const double *uma,
                              unsigned long long seed, int n_elite,
                              double *u_out, double *best_cost, double *cmd0) {
  if (!c || M > MS) return -1;
  P &p = c->p;
  p.M = M; p.psi0 = (float)yaw0; p.v0 = (float)v0; p.dprev = (float)dprev;
  p.vcap = (float)vcap; p.vtgt = (float)vtgt;
  p.seed_lo = (unsigned)seed; p.seed_hi = (unsigned)(seed >> 32);
  for (int i = 0; i < c->N; ++i) { p.umd[i] = (float)umd[i]; p.uma[i] = (float)uma[i]; }
  for (int j = 0; j < M; ++j) {
    double rx = wx[j] - x0, ry = wy[j] - y0;
    p.lx[j] = rx; p.ly[j] = ry; p.lh[j] = 0.5 * (rx * rx + ry * ry);
    p.lc[j] = cos(wpsi[j]); p.ls_[j] = sin(wpsi[j]); p.lpsi[j] = wpsi[j];
    p.lss[j] = ws[j]; p.lvx[j] = wvx[j];
  }
  for (int j = M; j < MS; ++j) p.lh[j] = 1e30f;
  p.prof = 2; p.seq = ++c->tick;
  pick(c->L, c->hg)<<<c->grid, c->blk, 0, c->st>>>(p, c->sc, c->out_d, c->costs_d, nullptr);
  while (*(volatile unsigned *)&c->out->seq != c->tick) {}
  Pick pk = pick_elites(c->costs, c->K, n_elite, c->scratch);
  const unsigned s2 = c->tick | 0x80000000u;
  (c->hg ? elite_k<true> : elite_k<false>)<<<(c->K + 255) / 256, 256, 0, c->st>>>(
      p, c->costs_d, pk.thresh, pk.n_take_eq, c->accd, c->tie, nullptr,
      c->out_d, c->gcnt, n_elite, s2);
  while (*(volatile unsigned *)&c->out->seq != s2) {}
  for (int i = 0; i < c->N; ++i) { u_out[i] = c->out->ud[i]; u_out[NMAX + i] = c->out->ua[i]; }
  *best_cost = pk.best; *cmd0 = c->out->cmd0;
  return 0;
}
#endif

#ifndef LIBRARY
#ifndef VALIDATE
int main(int argc, char **argv) {
  if (argc < 3) { fprintf(stderr, "usage: bench base|fast K [period_us] [L]\n"); return 1; }
  bool fast = !strcmp(argv[1], "fast");
  int K = atoi(argv[2]);
  double period = argc > 3 ? atof(argv[3]) : 0.0;
  int L = argc > 4 && atoi(argv[4]) > 0 ? atoi(argv[4]) : auto_lanes(K);
  int prof = argc > 5 ? atoi(argv[5]) : 0;
  int Nh = argc > 6 ? atoi(argv[6]) : N;
  bool hg = getenv("INTERP") ? atoi(getenv("INTERP")) : K <= 2048;   // hinge wins while latency-bound
  int blk = getenv("BLK") ? atoi(getenv("BLK")) : pick_blk((long)K * L);
  int R = period > 0 ? 1200 : 3000, W = 200;
  CK(cudaSetDeviceFlags(cudaDeviceScheduleSpin));
  CK(cudaFree(0));
  cudaStream_t st; CK(cudaStreamCreate(&st));
  std::vector<double> ts;

  if (!fast) {
    setup_base();
    double *cost, *cmd, *dlt, *acc;
    CK(cudaMallocManaged(&cost, K * sizeof(double)));
    CK(cudaMallocManaged(&cmd, (size_t)K * N * sizeof(double)));
    CK(cudaMallocManaged(&dlt, (size_t)K * N * sizeof(double)));
    CK(cudaMallocManaged(&acc, (size_t)K * N * sizeof(double)));
    int blk = 128, grid = (K + blk - 1) / blk;
    double u[2][NMAX], sink = 0;
    for (int rep = -W; rep < R; ++rep) {
      double t0 = now_us();
      solve_k<<<grid, blk, 0, st>>>(K, N, DT, WB, ALAG, DRMAX, SMAX, KART0[0], KART0[1],
          KART0[2], KART0[3], VCAP, VTGT, SIG_D, SIG_A, SEED, cost, cmd, dlt, acc, nullptr, 0.0);
      CK(cudaStreamSynchronize(st));
      int e0 = 0, e1 = 1, e2 = 2;
      for (int k = 0; k < K; ++k) {
        if (cost[k] < cost[e0]) { e2 = e1; e1 = e0; e0 = k; }
        else if (cost[k] < cost[e1]) { e2 = e1; e1 = k; }
        else if (cost[k] < cost[e2]) { e2 = k; }
      }
      for (int i = 0; i < N; ++i) {
        u[0][i] = (dlt[e0*N+i] + dlt[e1*N+i] + dlt[e2*N+i]) / 3.0;
        u[1][i] = (acc[e0*N+i] + acc[e1*N+i] + acc[e2*N+i]) / 3.0;
      }
      sink += u[0][0];
      double dtus = now_us() - t0;
      if (rep >= 0) ts.push_back(dtus);
      if (period > 0) gap_until(t0, period);
    }
    report("base", K, grid, ts);
    if (sink == 1e300) printf("%f\n", sink);
  } else {
    P p; fill_p(p, K);
    int grid = (int)(((long)K * L + blk - 1) / blk);
    if (grid > 65535) { fprintf(stderr, "grid too large\n"); return 1; }
    Scratch *sc; Out *out;
    CK(cudaMalloc(&sc, sizeof(Scratch)));
    CK(cudaMemset(sc, 0, sizeof(Scratch)));
    CK(cudaHostAlloc(&out, sizeof(Out), cudaHostAllocMapped));
    Out *out_d; CK(cudaHostGetDevicePointer((void **)&out_d, out, 0));
    FastFn fn = pick(L, hg);
    unsigned tick = 0;
    const int ne = n_elite_for(K);
    float *costs; CK(cudaHostAlloc(&costs, K * sizeof(float), cudaHostAllocMapped));
    float *costs_d; CK(cudaHostGetDevicePointer((void **)&costs_d, costs, 0));
    /* [0,N) ud sums, [NMAX,NMAX+N) ua sums, [2*NMAX] cmd0 sum, then the tie counter */
    float *accd; CK(cudaMalloc(&accd, (2 * NMAX + 1) * sizeof(float)));
    unsigned *tie; CK(cudaMalloc(&tie, sizeof(unsigned)));
    unsigned *gcnt; CK(cudaMalloc(&gcnt, sizeof(unsigned)));
    CK(cudaMemset(accd, 0, (2 * NMAX + 1) * sizeof(float)));
    CK(cudaMemset(tie, 0, sizeof(unsigned)));
    CK(cudaMemset(gcnt, 0, sizeof(unsigned)));
    std::vector<float> cscratch;
    fprintf(stderr, "n_elite = %d of K = %d (frac %.2f)\n", ne, K, EL_FRAC);
    bool use_sync = getenv("SYNC") && atoi(getenv("SYNC"));
    out->seq = 0;
    float sink = 0;
    for (int rep = -W; rep < R; ++rep) {
      double t0 = now_us();
      fill_p(p, K);                       // a real tick rebuilds the launch block
      p.prof = prof; p.N = Nh; p.seq = ++tick;
      p.prof = 2;                       // last block publishes seq, no top-3
      fn<<<grid, blk, 0, st>>>(p, sc, out_d, costs_d, nullptr);
      if (use_sync) CK(cudaStreamSynchronize(st));
      else while (*(volatile unsigned *)&out->seq != tick) {}
      Pick pk = pick_elites(costs, K, ne, cscratch);
      const unsigned s2 = tick | 0x80000000u;
      (hg ? elite_k<true> : elite_k<false>)<<<(K + 255) / 256, 256, 0, st>>>(
          p, costs_d, pk.thresh, pk.n_take_eq, accd, tie, nullptr, out_d, gcnt, ne, s2);
      if (use_sync) CK(cudaStreamSynchronize(st));
      else while (*(volatile unsigned *)&out->seq != s2) {}
      out->best = pk.best; out->best_i = pk.best_i;
      sink += out->ud[0] + out->cmd0;
      double dtus = now_us() - t0;
      if (rep >= 0) ts.push_back(dtus);
      if (period > 0) gap_until(t0, period);
    }
    CK(cudaStreamSynchronize(st));
    CK(cudaGetLastError());
    report(hg ? "fast" : "fstB", K, L * 1000 + blk, ts);
    if (sink == 1e30f) printf("%f\n", sink);
  }
  return 0;
}
#else
/* Same gaussians into both kernels; compare what each hands back. */
int main(int argc, char **argv) {
  int K = argc > 1 ? atoi(argv[1]) : 4096;
  CK(cudaFree(0));
  setup_base();
  double *pre, *cost, *cmd, *dlt, *acc; float *pref, *costf;
  CK(cudaMallocManaged(&pre, (size_t)K * N * 2 * sizeof(double)));
  CK(cudaMallocManaged(&pref, (size_t)K * N * 2 * sizeof(float)));
  srand(7);
  for (size_t i = 0; i < (size_t)K * N; ++i) {
    double u1 = (rand() + 1.0) / (RAND_MAX + 2.0), u2 = rand() / (RAND_MAX + 1.0);
    double r = sqrt(-2 * log(u1));
    pre[2*i] = r * cos(2 * M_PI * u2); pre[2*i+1] = r * sin(2 * M_PI * u2);
    pref[2*i] = (float)pre[2*i]; pref[2*i+1] = (float)pre[2*i+1];
  }
  CK(cudaMallocManaged(&cost, K * sizeof(double)));
  CK(cudaMallocManaged(&costf, K * sizeof(float)));
  CK(cudaMallocManaged(&cmd, (size_t)K * N * sizeof(double)));
  CK(cudaMallocManaged(&dlt, (size_t)K * N * sizeof(double)));
  CK(cudaMallocManaged(&acc, (size_t)K * N * sizeof(double)));
  solve_k<<<(K + 127) / 128, 128>>>(K, N, DT, WB, ALAG, DRMAX, SMAX, KART0[0], KART0[1],
      KART0[2], KART0[3], VCAP, VTGT, SIG_D, SIG_A, SEED, cost, cmd, dlt, acc, pre, 0.0);
  CK(cudaDeviceSynchronize());

  /* reference: mean over the n_elite lowest fp64 costs, exactly as
     np.argpartition(cost, n_elite - 1)[:n_elite] then mean. */
  const int ne = n_elite_for(K);
  std::vector<int> idx(K); for (int k = 0; k < K; ++k) idx[k] = k;
  std::nth_element(idx.begin(), idx.begin() + (ne - 1), idx.end(),
                   [&](int a, int b) { return cost[a] < cost[b]; });
  double uref[2][NMAX] = {{0}}, cmd0ref = 0;
  for (int e = 0; e < ne; ++e) {
    int k = idx[e];
    for (int i = 0; i < N; ++i) { uref[0][i] += dlt[k*N+i]; uref[1][i] += acc[k*N+i]; }
    cmd0ref += cmd[k*N];
  }
  for (int i = 0; i < N; ++i) { uref[0][i] /= ne; uref[1][i] /= ne; }
  cmd0ref /= ne;
  double cmin = cost[idx[0]]; for (int e = 1; e < ne; ++e) cmin = std::min(cmin, cost[idx[e]]);

  unsigned *tie; CK(cudaMallocManaged(&tie, sizeof(unsigned)));
  std::vector<float> cscratch;
  for (int hg = 0; hg < 2; ++hg)
  for (int L : {1, 2, 4, 8}) {
    int blk = pick_blk((long)K * L);
    int grid = (int)(((long)K * L + blk - 1) / blk);
    if (grid > 3 * 1024) continue;
    P p; fill_p(p, K); p.prof = 2; p.seq = 1;
    Scratch *sc; Out *out;
    CK(cudaMalloc(&sc, sizeof(Scratch))); CK(cudaMemset(sc, 0, sizeof(Scratch)));
    CK(cudaMallocManaged(&out, sizeof(Out)));
    memset(out, 0, sizeof(Out));
    pick(L, hg)<<<grid, blk>>>(p, sc, out, costf, pref);
    CK(cudaDeviceSynchronize());
    Pick pk = pick_elites(costf, K, ne, cscratch);
    float *vacc; CK(cudaMallocManaged(&vacc, (2 * NMAX + 1) * sizeof(float)));
    memset(vacc, 0, (2 * NMAX + 1) * sizeof(float)); *tie = 0u;
    (hg ? elite_k<true> : elite_k<false>)<<<(K + 255) / 256, 256>>>(
        p, costf, pk.thresh, pk.n_take_eq, vacc, tie, pref, nullptr, nullptr, ne, 0u);
    CK(cudaDeviceSynchronize());
    for (int i = 0; i < N; ++i) { out->ud[i] = vacc[i]; out->ua[i] = vacc[NMAX + i]; }
    out->cmd0 = vacc[2 * NMAX];
    cudaFree(vacc);
    float inv = 1.0f / (float)ne;
    double du = 0, da = 0;
    for (int i = 0; i < N; ++i) {
      du = std::max(du, fabs(out->ud[i] * inv - uref[0][i]));
      da = std::max(da, fabs(out->ua[i] * inv - uref[1][i]));
    }
    double cerr = 0;
    for (int k = 0; k < K; ++k) cerr = std::max(cerr, fabs((double)costf[k] - cost[k]));
    printf("%s L=%-2d n_elite %d/%d  cost |err| %.3g  u_d err %.3g rad (%.2e deg)  "
           "u_a err %.3g  cmd0 err %.3g  best %.4f vs %.4f\n",
           hg ? "hinge" : "bisect", L, ne, K, cerr, du, du * 57.29578, da,
           fabs(out->cmd0 * inv - cmd0ref), pk.best, cmin);
    cudaFree(sc); cudaFree(out);
  }
  return 0;
}
#endif
#endif  /* LIBRARY */
