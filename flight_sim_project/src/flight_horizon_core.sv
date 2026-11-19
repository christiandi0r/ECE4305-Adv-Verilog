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
    output logic [11:0] so_rgb,

    // Attitude values exported to HUD
    output logic signed [15:0] hud_pitch_offset,
    output logic signed [15:0] hud_roll_slope
);

// Register addresses
localparam BYPASS_REG = 14'h2000;
localparam PITCH_REG  = 14'h2001;
localparam ROLL_REG   = 14'h2002;

// Horizon colors
localparam SKY_COLOR    = 12'h49C;
localparam GROUND_COLOR = 12'h964;

// Control registers
logic bypass_reg;
logic signed [15:0] pitch_offset_reg;
logic signed [15:0] roll_slope_reg;

// VGA coordinate alignment
logic [10:0] x_d1, x_d2;
logic [10:0] y_d1, y_d2;

// Relative horizontal coordinate
logic signed [11:0] relative_x;

// Stage 1: Roll multiplication
logic signed [27:0] s1_product;
logic signed [15:0] s1_pitch;
logic [10:0] s1_y;
logic [11:0] s1_rgb;
logic s1_bypass;

// Stage 2: Horizon position
logic signed [31:0] s2_horizon;
logic [10:0] s2_y;
logic [11:0] s2_rgb;
logic s2_bypass;

// Export attitude registers to HUD
assign hud_pitch_offset = pitch_offset_reg;
assign hud_roll_slope = roll_slope_reg;

// FPro register interface
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
                pitch_offset_reg <=
                    $signed(wr_data[15:0]);

            ROLL_REG:
                roll_slope_reg <=
                    $signed(wr_data[15:0]);

            default: ;
        endcase
    end
end

// Delay VGA coordinates
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

// Horizontal position relative to screen center
assign relative_x =
    $signed({1'b0, x_d2}) - 12'sd320;

// Stage 1: Calculate roll contribution
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
    begin
        s1_product <= 0;
        s1_pitch <= 0;
        s1_y <= 0;
        s1_rgb <= 0;
        s1_bypass <= 1'b1;
    end
    else
    begin
        s1_product <= relative_x * roll_slope_reg;

        s1_pitch <= pitch_offset_reg;
        s1_y <= y_d2;

        s1_rgb <= si_rgb;
        s1_bypass <= bypass_reg;
    end
end

// Stage 2: Calculate horizon position
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
    begin
        s2_horizon <=
            0;

        s2_y <= 0;
        s2_rgb <= 0;
        s2_bypass <= 1'b1;
    end
    else
    begin
        s2_horizon <=
            32'sd240 +
            $signed(s1_pitch) +
            ($signed(s1_product) >>> 8);

        s2_y <= s1_y;
        s2_rgb <= s1_rgb;
        s2_bypass <= s1_bypass;
    end
end

// Stage 3: Select sky or ground
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
    begin
        so_rgb <= 0;
    end
    else if (s2_bypass)
    begin
        so_rgb <= s2_rgb;
    end
    else if ($signed({1'b0, s2_y}) < s2_horizon)
    begin
        so_rgb <= SKY_COLOR;
    end
    else
    begin
        so_rgb <= GROUND_COLOR;
    end
end

endmodule
