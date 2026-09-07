`timescale 1ns / 1ps

module m_n_mux
(
    input logic [3:0] m, n,
    input logic select,
    output logic [3:0] selected_duration
);

logic [3:0] p0, p1; 

assign selected_duration = p0 | p1; 
assign p0 = m & {4{select}}; 
assign p1 = n & {4{~select}};

endmodule
