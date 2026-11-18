
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
localparam FACE_BASE  = 14'h2001;
localparam COMMIT_REG = 14'h2059;

// Two mountains, four triangular faces each
localparam FACE_COUNT = 8;
localparam FACE_STRIDE = 11;

// VGA coordinate delay
logic [10:0] x_d1, x_d2, x_d3, x_d4, x_d5;
logic [10:0] y_d1, y_d2, y_d3, y_d4, y_d5;

// Shadow registers written by MicroBlaze
logic signed [15:0] shadow_a [0:7][0:2];
logic signed [15:0] shadow_b [0:7][0:2];
logic signed [31:0] shadow_c [0:7][0:2];

logic [11:0] shadow_color [0:7];
logic shadow_enable [0:7];

// Active geometry used for rendering
logic signed [15:0] active_a [0:7][0:2];
logic signed [15:0] active_b [0:7][0:2];
logic signed [31:0] active_c [0:7][0:2];

logic [11:0] active_color [0:7];
logic active_enable [0:7];

logic bypass_reg;
logic commit_pending;

// Previous VGA coordinate
logic [10:0] prev_x, prev_y;

// Incremental edge values
logic signed [31:0] row_base [0:7][0:2];
logic signed [31:0] edge_state [0:7][0:2];

// Stage 1
logic signed [31:0] s1_edge [0:7][0:2];
logic [11:0] s1_color [0:7];
logic s1_enable [0:7];
logic [11:0] s1_rgb;
logic s1_bypass;

// Stage 2
logic signed [31:0] s2_edge [0:7][0:2];
logic [11:0] s2_color [0:7];
logic s2_enable [0:7];
logic [11:0] s2_rgb;
logic s2_bypass;

// Stage 3
logic [7:0] s3_inside;
logic [11:0] s3_color [0:7];
logic [11:0] s3_rgb;
logic s3_bypass;

// Final color selection
logic [11:0] selected_rgb;

// Delay VGA coordinates to align with horizon
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
    begin
        x_d1 <= 0;
        x_d2 <= 0;
        x_d3 <= 0;
        x_d4 <= 0;
        x_d5 <= 0;

        y_d1 <= 0;
        y_d2 <= 0;
        y_d3 <= 0;
        y_d4 <= 0;
        y_d5 <= 0;
    end
    else
    begin
        x_d1 <= x;
        x_d2 <= x_d1;
        x_d3 <= x_d2;
        x_d4 <= x_d3;
        x_d5 <= x_d4;

        y_d1 <= y;
        y_d2 <= y_d1;
        y_d3 <= y_d2;
        y_d4 <= y_d3;
        y_d5 <= y_d4;
    end
end

// Shadow register interface
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
    begin
        bypass_reg <= 1'b1;
        commit_pending <= 1'b0;

        for (integer f = 0; f < FACE_COUNT; f++)
        begin
            shadow_color[f] <= 0;
            shadow_enable[f] <= 1'b0;

            for (integer e = 0; e < 3; e++)
            begin
                shadow_a[f][e] <= 0;
                shadow_b[f][e] <= 0;
                shadow_c[f][e] <= 0;
            end
        end
    end
    else
    begin
        // Clear pending update at frame origin
        if ((x_d5 == 0) && (y_d5 == 0) &&
            commit_pending)
        begin
            commit_pending <= 1'b0;
        end

        if (cs && write)
        begin
            // Renderer bypass
            if (addr == BYPASS_REG)
                bypass_reg <= wr_data[0];

            // Request geometry update
            if (addr == COMMIT_REG)
                commit_pending <= 1'b1;

            // Write shadow geometry for all 8 faces
            for (integer f = 0; f < FACE_COUNT; f++)
            begin
                case (addr)

                    FACE_BASE + f * FACE_STRIDE + 0:
                        shadow_a[f][0] <=
                            $signed(wr_data[15:0]);

                    FACE_BASE + f * FACE_STRIDE + 1:
                        shadow_b[f][0] <=
                            $signed(wr_data[15:0]);

                    FACE_BASE + f * FACE_STRIDE + 2:
                        shadow_c[f][0] <=
                            $signed(wr_data);

                    FACE_BASE + f * FACE_STRIDE + 3:
                        shadow_a[f][1] <=
                            $signed(wr_data[15:0]);

                    FACE_BASE + f * FACE_STRIDE + 4:
                        shadow_b[f][1] <=
                            $signed(wr_data[15:0]);

                    FACE_BASE + f * FACE_STRIDE + 5:
                        shadow_c[f][1] <=
                            $signed(wr_data);

                    FACE_BASE + f * FACE_STRIDE + 6:
                        shadow_a[f][2] <=
                            $signed(wr_data[15:0]);

                    FACE_BASE + f * FACE_STRIDE + 7:
                        shadow_b[f][2] <=
                            $signed(wr_data[15:0]);

                    FACE_BASE + f * FACE_STRIDE + 8:
                        shadow_c[f][2] <=
                            $signed(wr_data);

                    FACE_BASE + f * FACE_STRIDE + 9:
                        shadow_color[f] <= wr_data[11:0];

                    FACE_BASE + f * FACE_STRIDE + 10:
                        shadow_enable[f] <= wr_data[0];

                    default: ;
                endcase
            end
        end
    end
end

// Stage 1: Update incremental edge functions
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
    begin
        prev_x <= 0;
        prev_y <= 0;

        s1_rgb <= 0;
        s1_bypass <= 1'b1;

        for (integer f = 0; f < FACE_COUNT; f++)
        begin
            active_color[f] <= 0;
            active_enable[f] <= 1'b0;

            s1_color[f] <= 0;
            s1_enable[f] <= 1'b0;

            for (integer e = 0; e < 3; e++)
            begin
                active_a[f][e] <= 0;
                active_b[f][e] <= 0;
                active_c[f][e] <= 0;

                row_base[f][e] <= 0;
                edge_state[f][e] <= 0;
                s1_edge[f][e] <= 0;
            end
        end
    end
    else
    begin
        prev_x <= x_d5;
        prev_y <= y_d5;

        s1_rgb <= si_rgb;
        s1_bypass <= bypass_reg;

        for (integer f = 0; f < FACE_COUNT; f++)
        begin
            s1_color[f] <= active_color[f];
            s1_enable[f] <= active_enable[f];

            // Activate shadow geometry at frame origin
            if ((x_d5 == 0) && (y_d5 == 0) &&
                commit_pending)
            begin
                active_color[f] <= shadow_color[f];
                active_enable[f] <= shadow_enable[f];

                s1_color[f] <= shadow_color[f];
                s1_enable[f] <= shadow_enable[f];
            end

            for (integer e = 0; e < 3; e++)
            begin

                // Start of a new VGA frame
                if ((x_d5 == 0) && (y_d5 == 0))
                begin
                    if (commit_pending)
                    begin
                        active_a[f][e] <= shadow_a[f][e];
                        active_b[f][e] <= shadow_b[f][e];
                        active_c[f][e] <= shadow_c[f][e];

                        row_base[f][e] <= shadow_c[f][e];
                        edge_state[f][e] <= shadow_c[f][e];
                        s1_edge[f][e] <= shadow_c[f][e];
                    end
                    else
                    begin
                        row_base[f][e] <= active_c[f][e];
                        edge_state[f][e] <= active_c[f][e];
                        s1_edge[f][e] <= active_c[f][e];
                    end
                end

                // Advance one VGA scanline
                else if (y_d5 != prev_y)
                begin
                    row_base[f][e] <=
                        row_base[f][e] + active_b[f][e];

                    edge_state[f][e] <=
                        row_base[f][e] + active_b[f][e];

                    s1_edge[f][e] <=
                        row_base[f][e] + active_b[f][e];
                end

                // Advance one horizontal pixel
                else if (x_d5 != prev_x)
                begin
                    edge_state[f][e] <=
                        edge_state[f][e] + active_a[f][e];

                    s1_edge[f][e] <=
                        edge_state[f][e] + active_a[f][e];
                end

                // Same pixel, retain edge value
                else
                begin
                    s1_edge[f][e] <= edge_state[f][e];
                end
            end
        end
    end
end

// Stage 2: Register edge values and RGB
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
    begin
        s2_rgb <= 0;
        s2_bypass <= 1'b1;

        for (integer f = 0; f < FACE_COUNT; f++)
        begin
            s2_color[f] <= 0;
            s2_enable[f] <= 1'b0;

            for (integer e = 0; e < 3; e++)
                s2_edge[f][e] <= 0;
        end
    end
    else
    begin
        s2_rgb <= s1_rgb;
        s2_bypass <= s1_bypass;

        for (integer f = 0; f < FACE_COUNT; f++)
        begin
            s2_color[f] <= s1_color[f];
            s2_enable[f] <= s1_enable[f];

            for (integer e = 0; e < 3; e++)
                s2_edge[f][e] <= s1_edge[f][e];
        end
    end
end

// Stage 3: Classify triangle pixels
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
    begin
        s3_inside <= 0;
        s3_rgb <= 0;
        s3_bypass <= 1'b1;

        for (integer f = 0; f < FACE_COUNT; f++)
            s3_color[f] <= 0;
    end
    else
    begin
        s3_rgb <= s2_rgb;
        s3_bypass <= s2_bypass;

        for (integer f = 0; f < FACE_COUNT; f++)
        begin
            s3_color[f] <= s2_color[f];

            // Visible faces use negative edge winding
            s3_inside[f] <= s2_enable[f] &&
                (s2_edge[f][0] <= 0) &&
                (s2_edge[f][1] <= 0) &&
                (s2_edge[f][2] <= 0);
        end
    end
end

// Face priority: lower face index drawn in front
always_comb
begin
    selected_rgb = s3_rgb;

    for (integer f = FACE_COUNT - 1; f >= 0; f--)
    begin
        if (s3_inside[f])
            selected_rgb = s3_color[f];
    end
end

// Stage 4: Final RGB output
always_ff @(posedge clk, posedge reset)
begin
    if (reset)
        so_rgb <= 0;

    else if (s3_bypass)
        so_rgb <= s3_rgb;

    else
        so_rgb <= selected_rgb;
end

endmodule
