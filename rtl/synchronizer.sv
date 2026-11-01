`timescale 1ns/1ps
//------------------------------------------------------------------------------
// 2-FF synchronizer (one flop chain per bit).
// WIDTH is the full bus width. Only feed it gray-coded / single-bit-change data.
//------------------------------------------------------------------------------
module synchronizer #(
  parameter int WIDTH = 4
) (
  input  logic             clk,
  input  logic             rst_n,
  input  logic [WIDTH-1:0] d_in,
  output logic [WIDTH-1:0] d_out
);

  (* ASYNC_REG = "TRUE" *) logic [WIDTH-1:0] q1;
  (* ASYNC_REG = "TRUE" *) logic [WIDTH-1:0] q2;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      q1 <= '0;
      q2 <= '0;
    end else begin
      q1 <= d_in;
      q2 <= q1;
    end
  end

  assign d_out = q2;

endmodule
