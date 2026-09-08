`timescale 1ns / 1ps

module programmable_square_wave_TB();

// Time period
localparam T = 10;

// Declarations
logic [3:0] m, n;
logic clk, reset, square_wave_out;

programmable_square_wave uut(.*);

// clock (period = 10 ns)
always
begin
    clk = 1'b0;
    #(T / 2);
    clk = 1'b1;
    #(T / 2);
end

// initial reset
initial
begin
    reset = 1'b1;
    @(negedge clk)
    reset = 1'b0;
end

initial
begin
    m = 4'b0010;
    n = 4'b0011;
    
    #(100 * T); //1000 ns
    $stop;
end

endmodule
