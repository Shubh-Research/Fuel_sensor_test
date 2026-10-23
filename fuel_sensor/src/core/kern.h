/* Fixed-point kernels of tracker v6.10 (from dev/esp32_port/c/kern.c, review round 2; arithmetic unchanged).
 * Pure C99, no platform headers. Integer arithmetic is exact and every float operation is a basic IEEE operation, so a host build
 * and an rv32imc / xtensa build give the same bits (compile with -ffp-contract=off).
 *   fs_env_median : TrackerV4._envelope      -- median of the exact I^2+Q^2 per bin, correctly rounded float32 square roots
 *   fs_tpl_pos    : TrackerV4._tpl_pos        -- int32 envelope (per-call power-of-two scale), int32 template, int64 dot/energy
 *   fs_su_vote    : TrackerV4._su_vote        -- Q24 pulse shapes, int32 frame sum, phase-free Gram matrix (Schur complement)
 * Mirrors dev/esp32_port/reference/level_v4_fixedpoint.py, which is bit-identical to these kernels. */
#ifndef FS_CORE_KERN_H
#define FS_CORE_KERN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* fb: [nfb][npts][2] int16 (I, Q) (frame order does not matter for a median); env: npts float32. nfb <= 32. */
void fs_env_median(const int16_t *fb, int nfb, int npts, float *env);

/* _tpl_pos(env, guess): the radar bins are d[i] = dstart + i * dstep, the fine grid f0 + k * 0.05 with f0 == d[0];
 * inc_q32 = round(0.05 / dstep * 2^32); tplq = the template's every dk-th point as int32 (|x| < 2^23).
 * u is caller scratch for the local fine-grid envelope (u_cap entries).
 * Returns the fine-grid index of the best position, -1 when Python returns `guess` unchanged, -2 when the scratch is too small. */
int fs_tpl_pos(const float *env, int npts, double dstart, int fine_n, double guess, double win, const int32_t *tplq, int L, int dk,
               int coarse, uint32_t inc_q32, int32_t *u, int u_cap);

/* Start-up check context: three pulse shapes (floor, surface, reverberation) in Q24 on the offset grid off0 + doff * i, the
 * fit-window bins on that grid in Q20 (dq), the quarter-wave Q30 sine table, and scratch for the interpolated shapes. */
typedef struct {
    const int32_t *wr[3], *wi[3];   /* real / imaginary shape of the floor (0), surface (1) and reverberation (2) echo, Q24 */
    int nw;                         /* shape points */
    double doff;                    /* offset grid step (mm) */
    const int32_t *dq;              /* window bins: floor((d - off0) / doff * 2^20 + 0.5), increasing */
    int nb;                         /* window bins */
    const int32_t *sintab;          /* sin(2 pi i / 16384) in Q30, i = 0..4096 */
    int32_t *w;                     /* scratch: 3 echoes x 2 parts x nb interpolated shape values */
} fs_su_ctx;

/* Work counters of the last call of each kernel (diagnostics and the ESP32 cost model; not used by the tracker logic). */
typedef struct {
    int tpl_nscore;                 /* template scores evaluated (fs_tpl_pos) */
    int su_nres;                    /* level residuals evaluated (fs_su_vote) */
    int su_nbin_acc;                /* shape-bin products accumulated (fs_su_vote) */
} fs_kern_stats_t;
extern fs_kern_stats_t fs_kern_stats;

/* One start-up vote, mirroring _su_vote: zr/zi = the int32 frame SUM over the window bins (nb). For each candidate cycle
 * j = -4 .. ceil(su_max_h (n-1) / step) at level h = (b_fine + j step - d0) / (n - 1) within [-2.5, su_max_h], the minimum fit
 * residual over the levels HS[m] in [h + win_lo, h + win_hi] (floored at 1e-12 * energy) goes to res_out[j + 4]; -1 marks a
 * skipped candidate. Returns the number of slots written (j + 5 of the last candidate tried, at most nj_max). */
int fs_su_vote(const fs_su_ctx *ctx, const int32_t *zr, const int32_t *zi, double d0, double n, double c0, double step,
               double b_fine, double su_max_h, double win_lo, double win_hi, const double *HS, int nhs, double *res_out, int nj_max);

#ifdef __cplusplus
}
#endif

#endif /* FS_CORE_KERN_H */
