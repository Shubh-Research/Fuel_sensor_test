/* Fixed-point kernels of tracker v6.10 -- see kern.h.  Adapted from dev/esp32_port/c/kern.c (review round 2, reviewer F); the
 * arithmetic of every kernel is unchanged (bit-identical to dev/esp32_port/reference/level_v4_fixedpoint.py).  Changes for the
 * firmware: scratch buffers come from the caller (no hidden globals), the start-up check takes one pulse shape per echo (floor,
 * surface, reverberation) and the sine table from flash, the level window is found by binary search (the same levels), and the
 * cost-only float32 helpers and the LDL^T variant were dropped. */
#include "kern.h"

#define FS_MEMCPY __builtin_memcpy

fs_kern_stats_t fs_kern_stats;

/* ------------------------------------------------------------------ envelope (TrackerV4._envelope) */
static float sqrt_u32_cr(uint32_t p)            /* correctly rounded float32 sqrt of an integer < 2^32 */
{
    if (p == 0) return 0.0f;
    int L = 32 - __builtin_clz(p);
    int s = 47 - L; if (s & 1) s++;              /* P = p << s in [2^46, 2^48): a 24-bit root */
    uint64_t P = (uint64_t)p << s;
    uint32_t hi = (uint32_t)(P >> 24), lo = (uint32_t)P & 0xFFFFFFu;   /* 24 + 24 bits */
    uint32_t rem = 0, root = 0;
    for (int i = 11; i >= 0; i--) {              /* digit-by-digit, 2 input bits per result bit */
        rem = (rem << 2) | ((hi >> (2 * i)) & 3u); uint32_t tr = (root << 2) | 1u; root <<= 1;
        if (rem >= tr) { rem -= tr; root |= 1u; }
    }
    for (int i = 11; i >= 0; i--) {
        rem = (rem << 2) | ((lo >> (2 * i)) & 3u); uint32_t tr = (root << 2) | 1u; root <<= 1;
        if (rem >= tr) { rem -= tr; root |= 1u; }
    }
    if (rem > root) root++;                      /* round to nearest (no ties for integers) */
    int e = -s / 2;
    if (root == (1u << 24)) { root >>= 1; e++; }
    uint32_t bits = ((uint32_t)(127 + 23 + e) << 23) | (root & 0x7FFFFFu);
    float f; FS_MEMCPY(&f, &bits, 4); return f;
}

void fs_env_median(const int16_t *fb, int nfb, int npts, float *env)
{
    uint32_t v[32];
    if (nfb > 32) nfb = 32;
    for (int b = 0; b < npts; b++) {
        for (int f = 0; f < nfb; f++) {          /* insertion sort of the exact powers (I^2 + Q^2 < 2^31) */
            int32_t I = fb[(f * npts + b) * 2], Q = fb[(f * npts + b) * 2 + 1];
            uint32_t x = (uint32_t)(I * I) + (uint32_t)(Q * Q);
            int j = f; while (j > 0 && v[j - 1] > x) { v[j] = v[j - 1]; j--; } v[j] = x;
        }
        int k = nfb / 2;                         /* np.partition: the middle value (odd) or the mean of the middle two (even) */
        if (nfb & 1) env[b] = sqrt_u32_cr(v[k]);
        else env[b] = (sqrt_u32_cr(v[k - 1]) + sqrt_u32_cr(v[k])) / 2.0f;
    }
}

/* ------------------------------------------------------------------ template search (TrackerV4._tpl_pos, fixed point) */
static double dfloor(double x) { long long i = (long long)x; return (double)(i - (x < (double)i)); }
static double dceil(double x) { long long i = (long long)x; return (double)(i + (x > (double)i)); }

#define FS_TPL_EMAX 256                          /* envelope bins spanned by one search window (about 30 with win = 10 mm) */

int fs_tpl_pos(const float *env, int npts, double dstart, int fine_n, double guess, double win, const int32_t *tplq, int L, int dk,
               int coarse, uint32_t inc_q32, int32_t *u, int u_cap)
{
    double f0 = dstart, fs = 0.05;
    int half = L / 2, nq = (L + dk - 1) / dk;
    long long ka = (long long)dfloor((guess - win - f0) / fs) - 1; if (ka < 0) ka = 0;
    long long kb = (long long)dceil((guess + win - f0) / fs) + 2; if (kb > fine_n) kb = fine_n;
    int i0 = -1, i1 = -1; long long k;           /* idx = k_all[(fk >= guess-win) & (fk <= guess+win)]: contiguous, ends found */
    for (k = ka; k < kb && f0 + (double)k * fs < guess - win; k++) ;                      /* from both sides (1-3 steps) */
    if (k < kb && f0 + (double)k * fs <= guess + win) {
        i0 = (int)k;
        for (k = kb - 1; k > i0 && f0 + (double)k * fs > guess + win; k--) ;
        i1 = (int)k;
    }
    fs_kern_stats.tpl_nscore = 0;
    if (i0 < 0) return -1;
    int lo = i0 - half; if (lo < 0) lo = 0;
    int hi = i1 - half + L; if (hi > fine_n) hi = fine_n;
    int nu = hi - lo; if (nu > u_cap) return -2;
    /* envelope bins spanned by the window -> int32 with a per-call power-of-two scale (max in [2^22, 2^23)) */
    uint64_t p0 = (uint64_t)lo * inc_q32, p1 = (uint64_t)(hi - 1) * inc_q32;
    int b0 = (int)(p0 >> 32), b1 = (int)(p1 >> 32) + 1; if (b1 > npts - 1) b1 = npts - 1;
    if (b1 - b0 + 1 > FS_TPL_EMAX) return -2;
    float mx = 0.0f; for (int b = b0; b <= b1; b++) if (env[b] > mx) mx = env[b];
    int ex = 0; { uint32_t bits; FS_MEMCPY(&bits, &mx, 4); ex = (int)((bits >> 23) & 0xFF) - 127; }   /* mx in [2^ex, 2^(ex+1)) */
    int sh = 22 - ex;                            /* mx * 2^sh in [2^22, 2^23) */
    int32_t E[FS_TPL_EMAX];
    for (int b = b0; b <= b1; b++) {
        float y = env[b]; uint32_t bits; FS_MEMCPY(&bits, &y, 4);
        if (y == 0.0f) { E[b - b0] = 0; continue; }
        int e2 = (int)((bits >> 23) & 0xFF) - 127 + sh;   /* y * 2^sh = m * 2^(e2-23), m the 24-bit mantissa */
        uint32_t m = (bits & 0x7FFFFFu) | 0x800000u;
        E[b - b0] = (e2 >= 23) ? (int32_t)(m << (e2 - 23)) : (e2 < 0 ? 0 : (int32_t)(m >> (23 - e2)));
    }
    uint64_t p = (uint64_t)lo * inc_q32;
    for (int kk = lo; kk < hi; kk++, p += inc_q32) {   /* u = interp(fine grid, d, env): Q24 weights from a Q32 position */
        int b = (int)(p >> 32);
        if (b >= npts - 1) { u[kk - lo] = E[npts - 1 - b0]; continue; }
        int32_t t = (int32_t)((uint32_t)p >> 8);  /* Q24 */
        int32_t e0 = E[b - b0], e1 = E[b + 1 - b0];
        u[kk - lo] = e0 + (int32_t)(((int64_t)(e1 - e0) * t + (1 << 23)) >> 24);
    }
    /* score(c): a = c - half - lo; seg = u[a::dk][:nq] - min; D = seg.tplq, Q = seg.seg; compare D^2/Q (double); first maximum */
    #define FS_SCORE(c, out) do { int a_ = (c) - half - lo; (out) = -1.0;                                     \
        if (!((c) - half < 0 || (c) - half + L > fine_n || a_ < 0 || a_ + L > nu)) {                        \
            const int32_t *s_ = u + a_; int32_t mn_ = s_[0];                                                     \
            for (int m_ = 1; m_ < nq; m_++) if (s_[m_ * dk] < mn_) mn_ = s_[m_ * dk];                            \
            int64_t D_ = 0, Q_ = 0;                                                                              \
            for (int m_ = 0; m_ < nq; m_++) { int32_t x_ = s_[m_ * dk] - mn_; D_ += (int64_t)x_ * tplq[m_]; Q_ += (int64_t)x_ * x_; } \
            fs_kern_stats.tpl_nscore++;                                                                          \
            (out) = Q_ > 0 ? ((double)D_ * (double)D_) / (double)Q_ : 0.0; } } while (0)
    double cbs = -1.0; int cb = -1;
    for (int c = i0; c <= i1; c += coarse) { double sc; FS_SCORE(c, sc); if (sc >= 0.0 && sc > cbs) { cbs = sc; cb = c; } }
    if (cb < 0) return -1;
    double bs = -1.0; int best = -1;
    int ca = cb - coarse > i0 ? cb - coarse : i0, ce = cb + coarse < i1 ? cb + coarse : i1;
    for (int c = ca; c <= ce; c++) { double sc; FS_SCORE(c, sc); if (sc >= 0.0 && sc > bs) { bs = sc; best = c; } }
    #undef FS_SCORE
    return best;
}

/* ------------------------------------------------------------------ start-up vote (TrackerV4._su_vote, fixed point) */
/* The surface and reverberation phases drop out (each has a free complex amplitude); the floor phase is applied to the three data
 * projections only, so the 5x5 Gram matrix is phase-free. */
static int32_t sin_q30(const int32_t *sintab, uint32_t ph)   /* ph: fraction of a turn in Q32 */
{
    uint32_t q = ph >> 30, r = ph & 0x3FFFFFFFu;
    uint32_t idx = r >> 18, fr = r & 0x3FFFFu;
    int32_t a, b;
    if (q & 1) { a = sintab[4096 - idx]; b = idx < 4096 ? sintab[4095 - idx] : sintab[0]; }
    else { a = sintab[idx]; b = sintab[idx + 1]; }
    int32_t v = a + (int32_t)(((int64_t)(b - a) * fr) >> 18);
    return (q & 2) ? -v : v;
}

typedef struct { int k0, k1; } span_t;
static span_t shape_span(const int32_t *dq, int nb, int32_t xq, int nw)
{   /* bins k with 0 <= dq[k] - xq < (nw-1) << 20 (dq increasing, nearly uniform): arithmetic guess + exact adjust */
    span_t s; int32_t lim = (int32_t)((nw - 1) << 20), dd = (dq[nb - 1] - dq[0]) / (nb - 1);
    int k = (int)((int64_t)(xq - dq[0]) / dd); if (k < 0) k = 0; if (k > nb) k = nb;
    while (k > 0 && dq[k - 1] - xq >= 0) k--;
    while (k < nb && dq[k] - xq < 0) k++;
    s.k0 = k;
    int k1 = k + (int)(lim / dd); if (k1 > nb) k1 = nb;
    while (k1 > k && dq[k1 - 1] - xq >= lim) k1--;
    while (k1 < nb && dq[k1] - xq < lim) k1++;
    s.k1 = k1; return s;
}

/* residual of one level; x* = position on the shape grid in Q20 (P / doff * 2^20) of the floor B, surface S and reverberation R echoes;
 * cph/sph = cos/sin(2 pi (B - c0) / step) in Q30; en = sum |z|^2.  Returns en - |projection|^2, or en when the floor amplitude < 0. */
static double su_level(const fs_su_ctx *ctx, const int32_t *zr, const int32_t *zi, int32_t xs, int32_t xb, int32_t xr,
                       int32_t cph, int32_t sph, double en)
{
    const int32_t *dq = ctx->dq; int nb = ctx->nb;
    int32_t xq[3] = { xb, xs, xr };              /* order: floor (b), surface (s), reverberation (r) */
    span_t sp[3]; int64_t Eaa[3], Yr_[3], Yi_[3], Cr[3], Ci[3];
    for (int a = 0; a < 3; a++) {
        const int32_t *Wr = ctx->wr[a], *Wi = ctx->wi[a];
        span_t s = shape_span(dq, nb, xq[a], ctx->nw); sp[a] = s;
        int64_t e = 0, yr = 0, yi = 0; int32_t *wr = ctx->w + (2 * a) * nb, *wi = ctx->w + (2 * a + 1) * nb, x = xq[a];
        for (int k = s.k0; k < s.k1; k++) {
            int32_t q = dq[k] - x; int i = q >> 20; int32_t t = q & 0xFFFFF;
            int32_t r = Wr[i] + (int32_t)(((int64_t)(Wr[i + 1] - Wr[i]) * t) >> 20), m = Wi[i] + (int32_t)(((int64_t)(Wi[i + 1] - Wi[i]) * t) >> 20);
            wr[k] = r; wi[k] = m;
            e += (int64_t)r * r + (int64_t)m * m;
            yr += (int64_t)r * zr[k] + (int64_t)m * zi[k];          /* conj(w) z */
            yi += (int64_t)r * zi[k] - (int64_t)m * zr[k];
        }
        Eaa[a] = e; Yr_[a] = yr; Yi_[a] = yi; fs_kern_stats.su_nbin_acc += s.k1 - s.k0;
    }
    static const int pa[3] = { 0, 0, 1 }, pb[3] = { 1, 2, 2 };
    for (int p = 0; p < 3; p++) {                                   /* conj(w_a) w_b over the overlap */
        int a = pa[p], b = pb[p]; int k0 = sp[a].k0 > sp[b].k0 ? sp[a].k0 : sp[b].k0, k1 = sp[a].k1 < sp[b].k1 ? sp[a].k1 : sp[b].k1;
        const int32_t *ar = ctx->w + (2 * a) * nb, *ai = ctx->w + (2 * a + 1) * nb, *br = ctx->w + (2 * b) * nb, *bi = ctx->w + (2 * b + 1) * nb;
        int64_t cr = 0, ci = 0;
        for (int k = k0; k < k1; k++) { cr += (int64_t)ar[k] * br[k] + (int64_t)ai[k] * bi[k]; ci += (int64_t)ar[k] * bi[k] - (int64_t)ai[k] * br[k]; }
        Cr[p] = cr; Ci[p] = ci;
    }
    /* The 5x5 least squares (real floor amplitude a, complex surface / reverberation amplitudes u; the 4x4 block of G is the real
     * form of the 2x2 Hermitian H = [[Ess, Csr], [conj(Csr), Err]]) solved by the Schur complement on a.  G in Q48, y in Q24 * z,
     * phasor Q30: the projection carries 2^60. */
    double Ebb = (double)Eaa[0], Ess = (double)Eaa[1], Err = (double)Eaa[2];
    double mxd = Ebb; if (Ess > mxd) mxd = Ess; if (Err > mxd) mxd = Err;
    double lam = 1e-9 * mxd; Ebb += lam; Ess += lam; Err += lam;
    double c = (double)cph, s = (double)sph, vr[3], vi[3];
    for (int a = 0; a < 3; a++) { double yr = (double)Yr_[a], yi = (double)Yi_[a]; vr[a] = c * yr + s * yi; vi[a] = c * yi - s * yr; }
    double hr = (double)Cr[2], hi = (double)Ci[2];                                   /* Csr */
    double g1r = (double)Cr[0], g1i = -(double)Ci[0], g2r = (double)Cr[1], g2i = -(double)Ci[1];   /* g = conj(Cbs), conj(Cbr) */
    double idet = 1.0 / (Ess * Err - (hr * hr + hi * hi));
    /* w = H^-1 v_(s,r):  w1 = (Err v1 - Csr v2)/det,  w2 = (Ess v2 - conj(Csr) v1)/det */
    double w1r = (Err * vr[1] - (hr * vr[2] - hi * vi[2])) * idet, w1i = (Err * vi[1] - (hr * vi[2] + hi * vr[2])) * idet;
    double w2r = (Ess * vr[2] - (hr * vr[1] + hi * vi[1])) * idet, w2i = (Ess * vi[2] - (hr * vi[1] - hi * vr[1])) * idet;
    double k1r = (Err * g1r - (hr * g2r - hi * g2i)) * idet, k1i = (Err * g1i - (hr * g2i + hi * g2r)) * idet;
    double k2r = (Ess * g2r - (hr * g1r + hi * g1i)) * idet, k2i = (Ess * g2i - (hr * g1i - hi * g1r)) * idet;
    double vHw = vr[1] * w1r + vi[1] * w1i + vr[2] * w2r + vi[2] * w2i;          /* Re(v^H H^-1 v) */
    double gHw = g1r * w1r + g1i * w1i + g2r * w2r + g2i * w2i;                  /* Re(g^H H^-1 v) */
    double gHk = g1r * k1r + g1i * k1i + g2r * k2r + g2i * k2i;                  /* g^H H^-1 g */
    double num = vr[0] - gHw, den = Ebb - gHk;
    fs_kern_stats.su_nres++;
    if (num < 0) return en;                                                     /* floor amplitude a = num/den < 0: no fit */
    double q = vHw + num * (num / den);
    q = q * (1.0 / (double)(1ULL << 60));
    return en - q;
}

int fs_su_vote(const fs_su_ctx *ctx, const int32_t *zr, const int32_t *zi, double d0, double n, double c0, double step,
               double b_fine, double su_max_h, double win_lo, double win_hi, const double *HS, int nhs, double *res_out, int nj_max)
{
    int nb = ctx->nb; double doff = ctx->doff;
    int64_t en_i = 0; for (int k = 0; k < nb; k++) en_i += (int64_t)zr[k] * zr[k] + (int64_t)zi[k] * zi[k];
    double en = (double)en_i;
    int jmax = (int)dceil(su_max_h * (n - 1) / step), nj = 0;
    fs_kern_stats.su_nres = 0; fs_kern_stats.su_nbin_acc = 0;
    for (int j = -4; j <= jmax && j + 4 < nj_max; j++) {
        res_out[j + 4] = -1.0; nj = j + 5;
        double h = (b_fine + j * step - d0) / (n - 1);
        if (h < -2.5 || h > su_max_h) continue;
        double lo = h + win_lo, hi = h + win_hi, best = 0.0;
        int found = 0;                           /* res[k:k+m].min(): a residual can round below 0 (no sentinel) */
        int m0 = 0, m1 = nhs;                    /* first level >= lo (HS increasing): the same levels as testing each one */
        while (m0 < m1) { int mid = (m0 + m1) >> 1; if (HS[mid] >= lo) m1 = mid; else m0 = mid + 1; }
        for (int m = m0; m < nhs && HS[m] <= hi; m++) {
            double x = HS[m];
            double S = d0 - x, B = d0 + (n - 1) * x, R = d0 + (2 * n - 1) * x;
            /* Q20 positions on the shape grid: floor(P / doff * 2^20 + 0.5) (doff = 0.5: exactly 2 P 2^20, as in esp32_port) */
            int32_t xs = (int32_t)dfloor(S / doff * 1048576.0 + 0.5), xb = (int32_t)dfloor(B / doff * 1048576.0 + 0.5), xr = (int32_t)dfloor(R / doff * 1048576.0 + 0.5);
            double cyc = (B - c0) / step; double fr = cyc - dfloor(cyc);
            uint32_t ph = (uint32_t)(uint64_t)(int64_t)dfloor(fr * 4294967296.0 + 0.5);
            int32_t sph = sin_q30(ctx->sintab, ph), cph = sin_q30(ctx->sintab, ph + 0x40000000u);
            double v = su_level(ctx, zr, zi, xs, xb, xr, cph, sph, en);
            if (!found || v < best) { best = v; found = 1; }
        }
        if (!found) continue;
        if (best < 1e-12 * en) best = 1e-12 * en;   /* max(min, 1e-12 en): also lifts a negative residual */
        res_out[j + 4] = best;
    }
    return nj;
}
