
module flight_hud_core
(
    input logic clk, reset,
    input logic [10:0] x, y,

    // Existing artificial horizon controls
    input logic signed [15:0] pitch_offset,
    input logic signed [15:0] roll_slope,

    // Video interface
    input logic [11:0] si_rgb,
    output logic [11:0] so_rgb
);

// HUD color
localparam WHITE = 12'hFFF;

// VGA screen center
localparam XC = 320;
localparam YC = 240;

// VGA coordinate delay
logic [10:0] x_pipe [0:8];
logic [10:0] y_pipe [0:8];

// Fixed-point rotation coefficients (Q8)
logic signed [9:0] sin_q8;
logic signed [9:0] cos_q8;

logic signed [9:0] sin_reg;
logic signed [9:0] cos_reg;

logic signed [15:0] abs_roll;

// Relative screen coordinates
logic signed [15:0] dx;
logic signed [15:0] dy;

// Stage 1: Rotation products
logic signed [31:0] s1_dx_cos;
logic signed [31:0] s1_dx_sin;
logic signed [31:0] s1_dy_cos;
logic signed [31:0] s1_dy_sin;

logic signed [9:0] s1_sin;
logic signed [9:0] s1_cos;
logic signed [15:0] s1_abs_roll;

logic [10:0] s1_x, s1_y;
logic [11:0] s1_rgb;

// Stage 2: Rotated coordinates
logic signed [31:0] s2_horizontal;
logic signed [31:0] s2_vertical;

logic signed [12:0] s2_pointer_x;
logic signed [12:0] s2_pointer_y;

logic [10:0] s2_x, s2_y;
logic [11:0] s2_rgb;

// HUD drawing signals
logic signed [12:0] pointer_dx;
logic signed [12:0] pointer_dy;

logic signed [10:0] sin_magnitude;

logic pitch_mark;
logic roll_mark;
logic roll_pointer;
logic center_dot;

logic [11:0] hud_rgb;

// ------------------------------------------------------------
// VGA coordinate delay
// ------------------------------------------------------------
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
    begin
        for (integer i = 0; i < 9; i++)
        begin
            x_pipe[i] <= 0;
            y_pipe[i] <= 0;
        end
    end
    else
    begin
        x_pipe[0] <= x;
        y_pipe[0] <= y;

        for (integer i = 1; i < 9; i++)
        begin
            x_pipe[i] <= x_pipe[i-1];
            y_pipe[i] <= y_pipe[i-1];
        end
    end
end

// ------------------------------------------------------------
// Calculate roll magnitude
// ------------------------------------------------------------
always_comb
begin
    if (roll_slope < 0)
        abs_roll = -roll_slope;
    else
        abs_roll = roll_slope;
end

// ------------------------------------------------------------
// Q8 sine/cosine lookup table
// ------------------------------------------------------------
always_comb
begin
    sin_q8 = 10'sd0;
    cos_q8 = 10'sd256;

    if (abs_roll < 16'd4)
    begin
        sin_q8 = 10'sd0;
        cos_q8 = 10'sd256;
    end
    else if (abs_roll < 16'd13)
    begin
        sin_q8 = 10'sd9;
        cos_q8 = 10'sd256;
    end
    else if (abs_roll < 16'd22)
    begin
        sin_q8 = 10'sd18;
        cos_q8 = 10'sd255;
    end
    else if (abs_roll < 16'd31)
    begin
        sin_q8 = 10'sd27;
        cos_q8 = 10'sd255;
    end
    else if (abs_roll < 16'd41)
    begin
        sin_q8 = 10'sd36;
        cos_q8 = 10'sd254;
    end
    else if (abs_roll < 16'd50)
    begin
        sin_q8 = 10'sd44;
        cos_q8 = 10'sd252;
    end
    else if (abs_roll < 16'd59)
    begin
        sin_q8 = 10'sd53;
        cos_q8 = 10'sd250;
    end
    else if (abs_roll < 16'd69)
    begin
        sin_q8 = 10'sd62;
        cos_q8 = 10'sd248;
    end
    else if (abs_roll < 16'd78)
    begin
        sin_q8 = 10'sd71;
        cos_q8 = 10'sd246;
    end
    else if (abs_roll < 16'd88)
    begin
        sin_q8 = 10'sd79;
        cos_q8 = 10'sd243;
    end
    else if (abs_roll < 16'd98)
    begin
        sin_q8 = 10'sd88;
        cos_q8 = 10'sd241;
    end
    else if (abs_roll < 16'd109)
    begin
        sin_q8 = 10'sd96;
        cos_q8 = 10'sd237;
    end
    else if (abs_roll < 16'd119)
    begin
        sin_q8 = 10'sd104;
        cos_q8 = 10'sd234;
    end
    else if (abs_roll < 16'd130)
    begin
        sin_q8 = 10'sd112;
        cos_q8 = 10'sd230;
    end
    else if (abs_roll < 16'd142)
    begin
        sin_q8 = 10'sd120;
        cos_q8 = 10'sd226;
    end
    else if (abs_roll < 16'd154)
    begin
        sin_q8 = 10'sd128;
        cos_q8 = 10'sd222;
    end
    else if (abs_roll < 16'd166)
    begin
        sin_q8 = 10'sd136;
        cos_q8 = 10'sd217;
    end
    else if (abs_roll < 16'd179)
    begin
        sin_q8 = 10'sd143;
        cos_q8 = 10'sd212;
    end
    else
    begin
        sin_q8 = 10'sd150;
        cos_q8 = 10'sd207;
    end

    // Preserve roll direction
    if (roll_slope < 0)
        sin_q8 = -sin_q8;
end

// ------------------------------------------------------------
// Register rotation coefficients
// ------------------------------------------------------------
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
    begin
        sin_reg <= 0;
        cos_reg <= 10'sd256;
    end
    else
    begin
        sin_reg <= sin_q8;
        cos_reg <= cos_q8;
    end
end

// ------------------------------------------------------------
// Coordinates relative to aircraft center
// ------------------------------------------------------------
assign dx =
    $signed({1'b0, x_pipe[8]}) - 16'sd320;

assign dy =
    $signed({1'b0, y_pipe[8]}) -
    16'sd240 -
    pitch_offset;

// ------------------------------------------------------------
// Stage 1: Calculate rotation products
// ------------------------------------------------------------
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
    begin
        s1_dx_cos <= 0;
        s1_dx_sin <= 0;
        s1_dy_cos <= 0;
        s1_dy_sin <= 0;

        s1_sin <= 0;
        s1_cos <= 10'sd256;
        s1_abs_roll <= 0;

        s1_x <= 0;
        s1_y <= 0;
        s1_rgb <= 0;
    end
    else
    begin
        // Q8 rotation products
        s1_dx_cos <= dx * cos_reg;
        s1_dx_sin <= dx * sin_reg;

        s1_dy_cos <= dy * cos_reg;
        s1_dy_sin <= dy * sin_reg;

        s1_sin <= sin_reg;
        s1_cos <= cos_reg;
        s1_abs_roll <= abs_roll;

        s1_x <= x_pipe[8];
        s1_y <= y_pipe[8];

        s1_rgb <= si_rgb;
    end
end

// ------------------------------------------------------------
// Calculate sine magnitude for roll pointer
// ------------------------------------------------------------
always_comb
begin
    if (s1_sin < 0)
        sin_magnitude = -$signed(s1_sin);
    else
        sin_magnitude = $signed(s1_sin);
end

// ------------------------------------------------------------
// Stage 2: Rotated coordinates and roll pointer
// ------------------------------------------------------------
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
    begin
        s2_horizontal <= 0;
        s2_vertical <= 0;

        s2_pointer_x <= 13'sd320;
        s2_pointer_y <= 13'sd28;

        s2_x <= 0;
        s2_y <= 0;
        s2_rgb <= 0;
    end
    else
    begin
        // Convert Q8 rotation products to pixel coordinates
        s2_horizontal <=
            (s1_dx_cos + s1_dy_sin) >>> 8;

        s2_vertical <=
            (s1_dy_cos - s1_dx_sin) >>> 8;

        // Roll pointer limited to +/- 30 degrees
        if (s1_abs_roll >= 16'd148)
        begin
            if (s1_sin < 0)
                s2_pointer_x <= 13'sd444;
            else
                s2_pointer_x <= 13'sd196;

            s2_pointer_y <= 13'sd90;
        end
        else
        begin
            // Continuous pointer movement
            s2_pointer_x <=
                13'sd320 -
                ($signed(s1_sin) -
                 ($signed(s1_sin) >>> 5));

            s2_pointer_y <=
                13'sd28 +
                ((($signed(11'sd256) -
                    $signed(s1_cos)) * 11) >>> 3) +
                (sin_magnitude >>> 3);
        end

        s2_x <= s1_x;
        s2_y <= s1_y;
        s2_rgb <= s1_rgb;
    end
end

// ------------------------------------------------------------
// HUD drawing
// ------------------------------------------------------------
always_comb
begin
    pitch_mark = 1'b0;
    roll_mark = 1'b0;
    roll_pointer = 1'b0;
    center_dot = 1'b0;

    pointer_dx =
        $signed({1'b0, s2_x}) - s2_pointer_x;

    pointer_dy =
        $signed({1'b0, s2_y}) - s2_pointer_y;

    // ========================================================
    // PITCH LADDER
    // ========================================================
    // Pitch markings:
    // +/-5, +/-10, +/-15, +/-20 degrees
    //
    // Each 5 degrees corresponds to 25 pixels.
    //
    // Original solid white thickness:
    // approximately 3 pixels.
    //
    // Symmetric drawing region:
    // y = 100 to 380.

    if ((s2_x >= 120) && (s2_x <= 520) &&
        (s2_y >= 100) && (s2_y <= 380))
    begin
        // +/- 5 degrees - short lines
        if (((s2_vertical >= 24) &&
             (s2_vertical <= 26)) ||
            ((s2_vertical >= -26) &&
             (s2_vertical <= -24)))
        begin
            if ((s2_horizontal >= -20) &&
                (s2_horizontal <= 20))
                pitch_mark = 1'b1;
        end

        // +/- 10 degrees - long lines
        if (((s2_vertical >= 49) &&
             (s2_vertical <= 51)) ||
            ((s2_vertical >= -51) &&
             (s2_vertical <= -49)))
        begin
            if ((s2_horizontal >= -48) &&
                (s2_horizontal <= 48))
                pitch_mark = 1'b1;
        end

        // +/- 15 degrees - short lines
        if (((s2_vertical >= 74) &&
             (s2_vertical <= 76)) ||
            ((s2_vertical >= -76) &&
             (s2_vertical <= -74)))
        begin
            if ((s2_horizontal >= -20) &&
                (s2_horizontal <= 20))
                pitch_mark = 1'b1;
        end

        // +/- 20 degrees - long lines
        if (((s2_vertical >= 99) &&
             (s2_vertical <= 101)) ||
            ((s2_vertical >= -101) &&
             (s2_vertical <= -99)))
        begin
            if ((s2_horizontal >= -48) &&
                (s2_horizontal <= 48))
                pitch_mark = 1'b1;
        end
    end

    // ========================================================
    // FIXED ROLL SCALE
    // ========================================================

    // 0 degrees
    if ((s2_x >= 319) && (s2_x <= 321) &&
        (s2_y >= 42) && (s2_y <= 56))
        roll_mark = 1'b1;

    // +/- 10 degrees
    if ((((s2_x >= 277) && (s2_x <= 279)) ||
         ((s2_x >= 361) && (s2_x <= 363))) &&
        (s2_y >= 56) && (s2_y <= 69))
        roll_mark = 1'b1;

    // +/- 20 degrees
    if ((((s2_x >= 236) && (s2_x <= 238)) ||
         ((s2_x >= 402) && (s2_x <= 404))) &&
        (s2_y >= 76) && (s2_y <= 90))
        roll_mark = 1'b1;

    // +/- 30 degrees
    if ((((s2_x >= 195) && (s2_x <= 197)) ||
         ((s2_x >= 443) && (s2_x <= 445))) &&
        (s2_y >= 103) && (s2_y <= 121))
        roll_mark = 1'b1;

    // ========================================================
    // DOWNWARD-POINTING ROLL TRIANGLE
    // ========================================================
    if ((pointer_dy >= 0) &&
        (pointer_dy <= 12))
    begin
        if ((pointer_dx >=
             -(13'sd12 - pointer_dy)) &&
            (pointer_dx <=
              (13'sd12 - pointer_dy)))
        begin
            roll_pointer = 1'b1;
        end
    end

    // ========================================================
    // FIXED CENTER DOT
    // ========================================================
    // Uses screen coordinates, not rotated coordinates.

    if ((s2_x >= 318) && (s2_x <= 322) &&
        (s2_y >= 238) && (s2_y <= 242))
        center_dot = 1'b1;

    // ========================================================
    // TRANSPARENT HUD OVERLAY
    // ========================================================
    hud_rgb = s2_rgb;

    if (pitch_mark ||
        roll_mark ||
        roll_pointer ||
        center_dot)
    begin
        hud_rgb = WHITE;
    end
end

// ------------------------------------------------------------
// Stage 3: Final HUD output
// ------------------------------------------------------------
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
        so_rgb <= 12'h000;
    else
        so_rgb <= hud_rgb;
end

endmodule
