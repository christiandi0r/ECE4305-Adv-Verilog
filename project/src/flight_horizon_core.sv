`timescale 1ns / 1ps

module flight_horizon_core
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
localparam PITCH_REG  = 14'h2001;
localparam ROLL_REG   = 14'h2002;

// VGA colors (12-bit RGB)
localparam SKY_COLOR    = 12'h49C;
localparam GROUND_COLOR = 12'h964;

// Register declarations
logic bypass_reg;
logic signed [31:0] pitch_offset_reg;
logic signed [31:0] roll_slope_reg;

// Delayed pixel coordinates
logic [10:0] x_d1, x_d2;
logic [10:0] y_d1, y_d2;

// Horizon calculations
logic signed [31:0] pixel_x;
logic signed [31:0] pixel_y;
logic signed [63:0] slope_product;
logic signed [63:0] horizon_y;

// Control registers
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
    begin
        bypass_reg <= 1'b1;
        pitch_offset_reg <= 0;
        roll_slope_reg <= 0;
    end

    else if (cs && write)
    begin
        case (addr)
            BYPASS_REG:
                bypass_reg <= wr_data[0];

            PITCH_REG:
                pitch_offset_reg <= $signed(wr_data);

            ROLL_REG:
                roll_slope_reg <= $signed(wr_data);

            default: ;
        endcase
    end
end

// Delay pixel coordinates to align with video pipeline
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

// Horizon rendering
always_comb
begin
    // Center the coordinate system
    pixel_x = $signed({1'b0, x_d2}) - 32'sd320;
    pixel_y = $signed({1'b0, y_d2});

    // Calculate horizon slope using Q8 fixed-point
    slope_product = pixel_x * roll_slope_reg;

    // Calculate horizon position
    horizon_y = 64'sd240
              + pitch_offset_reg
              + (slope_product >>> 8);

    // Generate VGA colors
    if (bypass_reg)
        so_rgb = si_rgb;

    else if (pixel_y < horizon_y)
        so_rgb = SKY_COLOR;

    else
        so_rgb = GROUND_COLOR;
end

endmodule
