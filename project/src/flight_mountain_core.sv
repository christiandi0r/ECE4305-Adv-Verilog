
module flight_mountain_core
(
    input logic clk, reset,
    input logic [10:0] x, y,

    // FPro video slot interface
    input logic cs,
    input logic write,
    input logic [13:0] addr,
    input logic [31:0] wr_data,

    // Video interface
    input logic [11:0] si_rgb,
    output logic [11:0] so_rgb
);

// Register addresses
localparam BYPASS_REG = 14'h2000;
localparam X0_REG = 14'h2001;
localparam Y0_REG = 14'h2002;
localparam X1_REG = 14'h2003;
localparam Y1_REG = 14'h2004;
localparam X2_REG = 14'h2005;
localparam Y2_REG = 14'h2006;

// Mountain color
localparam MOUNTAIN_COLOR = 12'h687;

// Control registers
logic bypass_reg;

logic signed [15:0] x0_reg, y0_reg;
logic signed [15:0] x1_reg, y1_reg;
logic signed [15:0] x2_reg, y2_reg;

// Delayed pixel coordinates
logic [10:0] x_d1, x_d2;
logic [10:0] y_d1, y_d2;

// Stage 1: Edge vectors and pixel offsets
logic signed [15:0] s1_ex [0:2];
logic signed [15:0] s1_ey [0:2];
logic signed [15:0] s1_px [0:2];
logic signed [15:0] s1_py [0:2];

logic [11:0] s1_rgb;
logic s1_bypass;

// Stage 2: Edge multiplications
logic signed [31:0] s2_a [0:2];
logic signed [31:0] s2_b [0:2];

logic [11:0] s2_rgb;
logic s2_bypass;

// Stage 3: Edge-function results
logic signed [31:0] s3_edge [0:2];

logic [11:0] s3_rgb;
logic s3_bypass;

// Pixel classification
logic inside_triangle;

integer i;

// Control registers
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
    begin
        bypass_reg <= 1'b1;

        x0_reg <= 0;
        y0_reg <= 0;
        x1_reg <= 0;
        y1_reg <= 0;
        x2_reg <= 0;
        y2_reg <= 0;
    end

    else if (cs && write)
    begin
        case (addr)
            BYPASS_REG:
                bypass_reg <= wr_data[0];

            X0_REG:
                x0_reg <= $signed(wr_data[15:0]);

            Y0_REG:
                y0_reg <= $signed(wr_data[15:0]);

            X1_REG:
                x1_reg <= $signed(wr_data[15:0]);

            Y1_REG:
                y1_reg <= $signed(wr_data[15:0]);

            X2_REG:
                x2_reg <= $signed(wr_data[15:0]);

            Y2_REG:
                y2_reg <= $signed(wr_data[15:0]);

            default: ;
        endcase
    end
end

// Delay coordinates to match existing video timing
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
    begin
        x_d1 <= 0;
        x_d2 <= 0;
        y_d1 <= 0;
        y_d2 <= 0;
    end

    else
    begin
        x_d1 <= x;
        x_d2 <= x_d1;

        y_d1 <= y;
        y_d2 <= y_d1;
    end
end

// Stage 1: Calculate edge vectors and pixel offsets
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
    begin
        for (integer j = 0; j < 3; j++)
        begin
            s1_ex[j] <= 0;
            s1_ey[j] <= 0;
            s1_px[j] <= 0;
            s1_py[j] <= 0;
        end

        s1_rgb <= 0;
        s1_bypass <= 1'b1;
    end

    else
    begin
        // Edge 0: V0 -> V1
        s1_ex[0] <= x1_reg - x0_reg;
        s1_ey[0] <= y1_reg - y0_reg;
        s1_px[0] <= $signed({1'b0, x_d2}) - x0_reg;
        s1_py[0] <= $signed({1'b0, y_d2}) - y0_reg;

        // Edge 1: V1 -> V2
        s1_ex[1] <= x2_reg - x1_reg;
        s1_ey[1] <= y2_reg - y1_reg;
        s1_px[1] <= $signed({1'b0, x_d2}) - x1_reg;
        s1_py[1] <= $signed({1'b0, y_d2}) - y1_reg;

        // Edge 2: V2 -> V0
        s1_ex[2] <= x0_reg - x2_reg;
        s1_ey[2] <= y0_reg - y2_reg;
        s1_px[2] <= $signed({1'b0, x_d2}) - x2_reg;
        s1_py[2] <= $signed({1'b0, y_d2}) - y2_reg;

        s1_rgb <= si_rgb;
        s1_bypass <= bypass_reg;
    end
end

// Stage 2: Multiply edge-function terms
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
    begin
        for (integer j = 0; j < 3; j++)
        begin
            s2_a[j] <= 0;
            s2_b[j] <= 0;
        end

        s2_rgb <= 0;
        s2_bypass <= 1'b1;
    end

    else
    begin
        for (integer j = 0; j < 3; j++)
        begin
            s2_a[j] <= s1_ex[j] * s1_py[j];
            s2_b[j] <= s1_ey[j] * s1_px[j];
        end

        s2_rgb <= s1_rgb;
        s2_bypass <= s1_bypass;
    end
end

// Stage 3: Calculate signed edge functions
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
    begin
        for (integer j = 0; j < 3; j++)
            s3_edge[j] <= 0;

        s3_rgb <= 0;
        s3_bypass <= 1'b1;
    end

    else
    begin
        for (integer j = 0; j < 3; j++)
            s3_edge[j] <= s2_a[j] - s2_b[j];

        s3_rgb <= s2_rgb;
        s3_bypass <= s2_bypass;
    end
end

// All edges must have the same sign
always_comb
begin
    inside_triangle = 1'b0;

    if (((s3_edge[0] >= 0) &&
         (s3_edge[1] >= 0) &&
         (s3_edge[2] >= 0)) ||

        ((s3_edge[0] <= 0) &&
         (s3_edge[1] <= 0) &&
         (s3_edge[2] <= 0)))
    begin
        inside_triangle = 1'b1;
    end
end

// Stage 4: Select final pixel color
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
        so_rgb <= 0;

    else if (s3_bypass)
        so_rgb <= s3_rgb;

    else if (inside_triangle)
        so_rgb <= MOUNTAIN_COLOR;

    else
        so_rgb <= s3_rgb;
end

endmodule
