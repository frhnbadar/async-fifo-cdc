`timescale 1ns/1ps

module fifo_mem #(
  parameter int DEPTH      = 8,
  parameter int DATA_WIDTH = 8,
  parameter int PTR_WIDTH  = $clog2(DEPTH)
) (
  input  logic                   wclk,
  input  logic                   w_en,
  input  logic                   full,
  input  logic [PTR_WIDTH:0]     b_wptr,
  input  logic [PTR_WIDTH:0]     b_rptr,
  input  logic [DATA_WIDTH-1:0]  data_in,
  output logic [DATA_WIDTH-1:0]  data_out
);

  logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

  always_ff @(posedge wclk) begin
    if (w_en && !full)
      mem[b_wptr[PTR_WIDTH-1:0]] <= data_in;
  end

  assign data_out = mem[b_rptr[PTR_WIDTH-1:0]];

endmodule
