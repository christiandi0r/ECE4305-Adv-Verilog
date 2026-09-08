`timescale 1ns / 1ps

module programmable_square_wave
(
    input logic [3:0] m, n,
    input logic clk, reset,
    output logic square_wave_out
);

logic [7:0] counter_q;
logic [3:0] selected_duration;
logic max_tick;
logic state_out;

assign square_wave_out = state_out;

programmable_counter Mod_Counter
(
    .in(selected_duration),
    .clk(clk),
    .reset(reset),
    .max_tick(max_tick),
    .q(counter_q)
);

m_n_mux Mux_2x1
(
    .m(m),
    .n(n),
    .select(state_out),
    .selected_duration(selected_duration)
);

square_wave_state State_Register
(
    .clk(clk),
    .reset(reset),
    .max_tick(max_tick),
    .state_out(state_out)
);

ila_0 ILA
(
    .clk(clk),
    .probe0(m),
    .probe1(n),
    .probe2(square_wave_out)
);

endmodule
