`timescale 1ns / 1ps

module programmable_counter 
(
    input logic [3:0] in, 
    input logic clk, reset,
    output logic max_tick,
    output logic [7:0] q
);

logic [7:0] r_reg;
logic [7:0] r_next;

always_ff @(posedge clk)
begin
    if (reset)
        r_reg <= 0;
    else
        r_reg <= r_next;
end

always_comb
begin 
    if (in == 0)
    begin
        r_next = 0;
        max_tick = 1;
    end
    
    else if (r_reg == ((in*10) - 1))
    begin
        r_next = 0;
        max_tick = 1;
    end
    
    else
    begin
        r_next = r_reg + 1;
        max_tick = 0;
    end      
end

assign q = r_reg;

endmodule
