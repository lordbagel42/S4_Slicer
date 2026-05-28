// CSR sparse matrix (f64) and basic linear algebra needed by LSMR / TRF.
const std = @import("std");

pub const Csr = struct {
    nrows: usize,
    ncols: usize,
    data: []f64,     // non-zero values
    indices: []u32,  // column indices (same length as data)
    indptr: []u32,   // row pointers (length = nrows + 1)
    allocator: std.mem.Allocator,

    pub fn deinit(self: *Csr) void {
        self.allocator.free(self.data);
        self.allocator.free(self.indices);
        self.allocator.free(self.indptr);
    }

    // y = A * x
    pub fn matvec(self: Csr, x: []const f64, y: []f64) void {
        @memset(y, 0);
        for (0..self.nrows) |i| {
            var acc: f64 = 0;
            for (self.indptr[i]..self.indptr[i + 1]) |k| {
                acc += self.data[k] * x[self.indices[k]];
            }
            y[i] = acc;
        }
    }

    // y = A^T * x
    pub fn matvec_t(self: Csr, x: []const f64, y: []f64) void {
        @memset(y, 0);
        for (0..self.nrows) |i| {
            const xi = x[i];
            if (xi == 0) continue;
            for (self.indptr[i]..self.indptr[i + 1]) |k| {
                y[self.indices[k]] += self.data[k] * xi;
            }
        }
    }

    // Frobenius norm squared
    pub fn norm_sq(self: Csr) f64 {
        var s: f64 = 0;
        for (self.data) |v| s += v * v;
        return s;
    }
};

// CSR builder (COO → CSR)
pub const CsrBuilder = struct {
    nrows: usize,
    ncols: usize,
    entries: std.array_list.Managed(Entry),

    const Entry = struct { row: u32, col: u32, val: f64 };

    pub fn init(nrows: usize, ncols: usize, allocator: std.mem.Allocator) CsrBuilder {
        return .{
            .nrows = nrows,
            .ncols = ncols,
            .entries = std.array_list.Managed(Entry).init(allocator),
        };
    }

    pub fn set(self: *CsrBuilder, row: u32, col: u32, val: f64) !void {
        try self.entries.append(.{ .row = row, .col = col, .val = val });
    }

    pub fn build(self: *CsrBuilder, allocator: std.mem.Allocator) !Csr {
        const n = self.entries.items.len;
        // Sort by row, then col
        std.mem.sort(Entry, self.entries.items, {}, struct {
            fn lt(_: void, a: Entry, b: Entry) bool {
                if (a.row != b.row) return a.row < b.row;
                return a.col < b.col;
            }
        }.lt);

        // Deduplicate: sum values for same (row,col)
        var dedup = std.array_list.Managed(Entry).init(allocator);
        defer dedup.deinit();
        var i: usize = 0;
        while (i < n) {
            var j = i + 1;
            var val = self.entries.items[i].val;
            while (j < n and self.entries.items[j].row == self.entries.items[i].row and
                   self.entries.items[j].col == self.entries.items[i].col) : (j += 1)
            {
                val += self.entries.items[j].val;
            }
            try dedup.append(.{ .row = self.entries.items[i].row, .col = self.entries.items[i].col, .val = val });
            i = j;
        }

        const nnz = dedup.items.len;
        const data    = try allocator.alloc(f64, nnz);
        const indices = try allocator.alloc(u32, nnz);
        const indptr  = try allocator.alloc(u32, self.nrows + 1);
        @memset(indptr, 0);

        for (dedup.items) |e| indptr[e.row + 1] += 1;
        for (1..self.nrows + 1) |r| indptr[r] += indptr[r - 1];

        var row_offsets = try allocator.alloc(u32, self.nrows);
        defer allocator.free(row_offsets);
        @memcpy(row_offsets, indptr[0..self.nrows]);

        for (dedup.items) |e| {
            const pos = row_offsets[e.row];
            data[pos] = e.val;
            indices[pos] = e.col;
            row_offsets[e.row] += 1;
        }

        return .{
            .nrows = self.nrows,
            .ncols = self.ncols,
            .data = data,
            .indices = indices,
            .indptr = indptr,
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *CsrBuilder) void {
        self.entries.deinit();
    }
};

// Dense vector helpers
pub fn vec_dot(a: []const f64, b: []const f64) f64 {
    var s: f64 = 0;
    for (a, b) |x, y| s += x * y;
    return s;
}

pub fn vec_norm(a: []const f64) f64 {
    return @sqrt(vec_dot(a, a));
}

pub fn vec_axpy(alpha: f64, x: []const f64, y: []f64) void {
    for (x, y) |xi, *yi| yi.* += alpha * xi;
}

pub fn vec_scale(x: []f64, s: f64) void {
    for (x) |*xi| xi.* *= s;
}

pub fn vec_copy(dst: []f64, src: []const f64) void {
    @memcpy(dst, src);
}
