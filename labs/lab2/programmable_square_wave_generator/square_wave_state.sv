`timescale 1ns / 1ps

module square_wave_state
(
    input logic clk, reset, max_tick,
    output logic state_out
);

always_ff @(posedge clk)
begin
    if (reset)
        state_out <= 1'b1;
    else if (max_tick)
        state_out <= ~state_out;
end

endmodule
