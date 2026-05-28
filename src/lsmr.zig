// LSQR (Paige & Saunders 1982): min ||A x - b||_2
// Both optimization problems in the slicer are linear LS, so LSQR solves them in one pass.
const std = @import("std");
const sp = @import("sparse.zig");

pub const Opts = struct {
    atol: f64 = 1e-8,
    max_iter: usize = 0, // 0 → 4*n
};

pub const Result = struct { iters: usize, norm_r: f64 };

// Solve via conjugate gradient on normal equations: (A^T A) x = A^T b
// Simpler than full LSQR bidiagonalization and sufficient for SPD normal eqs.
pub fn solve_normal_equations(
    jac: *const sp.Csr,
    b: []const f64,
    x: []f64,     // initial 0, solution on exit
    opts: Opts,
    allocator: std.mem.Allocator,
) !Result {
    const n = jac.ncols;
    const max_iter = if (opts.max_iter == 0) n * 10 else opts.max_iter;

    // Compute rhs = A^T b
    const rhs = try allocator.alloc(f64, n); defer allocator.free(rhs);
    jac.matvec_t(b, rhs);

    // CG: solve (A^T A) x = rhs
    // r = rhs - A^T A x  (initially x=0 → r = rhs)
    const r  = try allocator.alloc(f64, n); defer allocator.free(r);
    const p  = try allocator.alloc(f64, n); defer allocator.free(p);
    const Ap = try allocator.alloc(f64, n); defer allocator.free(Ap);
    const Atmp = try allocator.alloc(f64, jac.nrows); defer allocator.free(Atmp);

    @memcpy(r, rhs);
    @memcpy(p, r);
    @memset(x, 0);

    var rsold = sp.vec_dot(r, r);
    var norm_r = @sqrt(rsold);
    const tol = opts.atol * norm_r;

    var iter: usize = 0;
    while (iter < max_iter and norm_r > tol) : (iter += 1) {
        // Ap = A^T A p
        jac.matvec(p, Atmp);
        jac.matvec_t(Atmp, Ap);

        const pAp = sp.vec_dot(p, Ap);
        if (@abs(pAp) < 1e-15) break;
        const alpha = rsold / pAp;

        sp.vec_axpy(alpha, p, x);
        sp.vec_axpy(-alpha, Ap, r);

        const rsnew = sp.vec_dot(r, r);
        norm_r = @sqrt(rsnew);
        if (norm_r < tol) break;

        const beta = rsnew / rsold;
        for (p, r) |*pi, ri| pi.* = ri + beta * pi.*;
        rsold = rsnew;
    }

    return .{ .iters = iter, .norm_r = norm_r };
}
