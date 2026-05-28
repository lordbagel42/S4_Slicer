const std = @import("std");

pub const Vec3 = [3]f64;
pub const Mat3x3 = [9]f64; // row-major

pub inline fn dot3(a: Vec3, b: Vec3) f64 {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

pub inline fn norm3(v: Vec3) f64 {
    return @sqrt(dot3(v, v));
}

pub inline fn normalize3(v: Vec3) Vec3 {
    const n = norm3(v);
    if (n < 1e-12) return .{ 1, 0, 0 };
    return .{ v[0] / n, v[1] / n, v[2] / n };
}

pub inline fn cross3(a: Vec3, b: Vec3) Vec3 {
    return .{
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    };
}

pub inline fn add3(a: Vec3, b: Vec3) Vec3 {
    return .{ a[0] + b[0], a[1] + b[1], a[2] + b[2] };
}

pub inline fn sub3(a: Vec3, b: Vec3) Vec3 {
    return .{ a[0] - b[0], a[1] - b[1], a[2] - b[2] };
}

pub inline fn scale3(v: Vec3, s: f64) Vec3 {
    return .{ v[0] * s, v[1] * s, v[2] * s };
}

pub inline fn lerp3(a: Vec3, b: Vec3, t: f64) Vec3 {
    return add3(scale3(a, 1.0 - t), scale3(b, t));
}

// Matrix-vector multiply (3x3 row-major) * vec3
pub inline fn mat3_mul_vec3(m: Mat3x3, v: Vec3) Vec3 {
    return .{
        m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
        m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
        m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
    };
}

// Build rotation matrix from axis-angle (Rodrigues)
pub fn rotation_matrix_from_axis_angle(axis: Vec3, angle: f64) Mat3x3 {
    const a = normalize3(axis);
    const c = @cos(angle);
    const s = @sin(angle);
    const t = 1.0 - c;
    const x = a[0];
    const y = a[1];
    const z = a[2];
    return .{
        t * x * x + c,     t * x * y - s * z, t * x * z + s * y,
        t * x * y + s * z, t * y * y + c,     t * y * z - s * x,
        t * x * z - s * y, t * y * z + s * x, t * z * z + c,
    };
}

// Tet volume (positive)
pub fn tet_volume(p1: Vec3, p2: Vec3, p3: Vec3, p4: Vec3) f64 {
    const a = sub3(p2, p1);
    const b = sub3(p3, p1);
    const c = sub3(p4, p1);
    const bc = cross3(b, c);
    return @abs(dot3(a, bc)) / 6.0;
}

// Barycentric coordinates of point inside tetrahedron (a,b,c,d).
// Returns [lambda_a, lambda_b, lambda_c, lambda_d]; sum ~1 if inside.
pub fn barycentric(a: Vec3, b: Vec3, c: Vec3, d: Vec3, p: Vec3) [4]f64 {
    const total = tet_volume(a, b, c, d);
    if (total < 1e-15) return .{ 0.25, 0.25, 0.25, 0.25 };
    return .{
        tet_volume(p, b, c, d) / total,
        tet_volume(p, a, c, d) / total,
        tet_volume(p, a, b, d) / total,
        tet_volume(p, a, b, c) / total,
    };
}

// 2x2 SVD-based 2D rotation angle between two point sets (Kabsch on 2D projections)
// Returns rotation angle in radians.
pub fn kabsch_angle_2d(
    new_pts: []const [2]f64,
    old_pts: []const [2]f64,
) f64 {
    // covariance = new^T * old
    var h: [4]f64 = .{ 0, 0, 0, 0 }; // 2x2 row-major
    for (new_pts, old_pts) |n, o| {
        h[0] += n[0] * o[0];
        h[1] += n[0] * o[1];
        h[2] += n[1] * o[0];
        h[3] += n[1] * o[1];
    }
    // SVD of 2x2: H = U S V^T
    // For 2x2 Kabsch we just need U V^T which is the rotation
    const svd = svd2x2(h);
    // R = U * V^T
    const r00 = svd.u[0] * svd.vt[0] + svd.u[1] * svd.vt[2];
    const r10 = svd.u[2] * svd.vt[0] + svd.u[3] * svd.vt[2];
    var angle = -std.math.acos(std.math.clamp(r00, -1.0, 1.0));
    if (r10 < 0) angle = -angle;
    return angle;
}

const SVD2 = struct {
    u: [4]f64,  // 2x2 row-major
    s: [2]f64,
    vt: [4]f64, // 2x2 row-major (V transposed)
};

// Golub-Reinsch SVD for 2x2 matrix (sufficient for Kabsch angle extraction)
fn svd2x2(a: [4]f64) SVD2 {
    // Use the analytic 2x2 SVD
    const a11 = a[0]; const a12 = a[1];
    const a21 = a[2]; const a22 = a[3];

    const s1 = a11 + a22;
    const s2 = a21 - a12;
    const s3 = a11 - a22;
    const s4 = a21 + a12;

    const tau1 = @sqrt(s1 * s1 + s2 * s2);
    const tau2 = @sqrt(s3 * s3 + s4 * s4);

    const sig1 = (tau1 + tau2) / 2.0;
    const sig2 = @abs(tau1 - tau2) / 2.0;

    const theta: f64 = if (tau1 > 1e-12) std.math.atan2(s2, s1) else 0.0;
    const phi: f64   = if (tau2 > 1e-12) std.math.atan2(s4, s3) else 0.0;

    const c1 = @cos((theta - phi) / 2.0);
    const s1r = @sin((theta - phi) / 2.0);
    const c2 = @cos((theta + phi) / 2.0);
    const s2r = @sin((theta + phi) / 2.0);

    return .{
        .u = .{ c1, -s1r, s1r, c1 },
        .s = .{ sig1, sig2 },
        .vt = .{ c2, s2r, -s2r, c2 },
    };
}

// Project 3D points onto a 2D plane defined by two orthogonal unit vectors.
pub fn project_onto_plane(plane_x: Vec3, plane_y: Vec3, p: Vec3) [2]f64 {
    return .{ dot3(plane_x, p), dot3(plane_y, p) };
}

// Plane fit via PCA on a set of 3D points. Returns (centroid, normal).
// points: slice of [3]f64
pub fn plane_fit(points: []const Vec3, allocator: std.mem.Allocator) !struct { centroid: Vec3, normal: Vec3 } {
    const n = points.len;
    if (n < 3) return .{ .centroid = .{ 0, 0, 0 }, .normal = .{ 0, 0, 1 } };

    // centroid
    var c: Vec3 = .{ 0, 0, 0 };
    for (points) |p| { c[0] += p[0]; c[1] += p[1]; c[2] += p[2]; }
    c[0] /= @floatFromInt(n); c[1] /= @floatFromInt(n); c[2] /= @floatFromInt(n);

    // 3x3 covariance
    var cov: [9]f64 = .{0} ** 9;
    for (points) |p| {
        const d = sub3(p, c);
        inline for (0..3) |i| {
            inline for (0..3) |j| {
                cov[i * 3 + j] += d[i] * d[j];
            }
        }
    }

    _ = allocator;
    // Power iteration to find smallest eigenvector (normal to plane)
    // For correctness on any # of points, use Jacobi 3x3 SVD.
    const normal = jacobi3x3_min_eigenvector(cov);
    return .{ .centroid = c, .normal = normal };
}

// Jacobi iteration for symmetric 3x3 matrix, returns eigenvector of smallest eigenvalue.
fn jacobi3x3_min_eigenvector(m: [9]f64) Vec3 {
    var a = m;
    var v: [9]f64 = .{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }; // eigenvectors (columns)

    var iter: usize = 0;
    while (iter < 50) : (iter += 1) {
        // Find largest off-diagonal element
        var max_val: f64 = 0;
        var p: usize = 0;
        var q: usize = 1;
        const pairs = [_][2]usize{ .{ 0, 1 }, .{ 0, 2 }, .{ 1, 2 } };
        for (pairs) |pq| {
            const val = @abs(a[pq[0] * 3 + pq[1]]);
            if (val > max_val) { max_val = val; p = pq[0]; q = pq[1]; }
        }
        if (max_val < 1e-12) break;

        // Compute rotation angle
        const app = a[p * 3 + p];
        const aqq = a[q * 3 + q];
        const apq = a[p * 3 + q];
        const tau = (aqq - app) / (2.0 * apq);
        const t = if (tau >= 0) 1.0 / (tau + @sqrt(1.0 + tau * tau))
                  else 1.0 / (tau - @sqrt(1.0 + tau * tau));
        const c = 1.0 / @sqrt(1.0 + t * t);
        const s = t * c;

        // Apply Jacobi rotation
        a[p * 3 + p] = app - t * apq;
        a[q * 3 + q] = aqq + t * apq;
        a[p * 3 + q] = 0;
        a[q * 3 + p] = 0;
        for (0..3) |r| {
            if (r != p and r != q) {
                const arp = a[r * 3 + p];
                const arq = a[r * 3 + q];
                a[r * 3 + p] = c * arp - s * arq;
                a[p * 3 + r] = a[r * 3 + p];
                a[r * 3 + q] = s * arp + c * arq;
                a[q * 3 + r] = a[r * 3 + q];
            }
        }
        for (0..3) |r| {
            const vrp = v[r * 3 + p];
            const vrq = v[r * 3 + q];
            v[r * 3 + p] = c * vrp - s * vrq;
            v[r * 3 + q] = s * vrp + c * vrq;
        }
    }

    // Find index of smallest eigenvalue (diagonal of a after convergence)
    var min_idx: usize = 0;
    var min_val = a[0];
    for (0..3) |i| {
        if (a[i * 3 + i] < min_val) { min_val = a[i * 3 + i]; min_idx = i; }
    }
    return .{ v[0 * 3 + min_idx], v[1 * 3 + min_idx], v[2 * 3 + min_idx] };
}
