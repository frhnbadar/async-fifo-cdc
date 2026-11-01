`timescale 1ns/1ps
//------------------------------------------------------------------------------
// Asynchronous FIFO (dual clock) - gray-pointer, 2-FF synchronizer
//
//  DEPTH must be a power of 2 and >= 4.
//  Reset: each domain has its own active-low async reset. Assert both together.
//         (In a real SoC, drive each with a reset synchronizer: async assert,
//          sync de-assert in its own clock domain.)
//  Read:  first-word fall-through. data_out is valid whenever empty == 0;
//         pulse r_en for one cycle to pop it.
//  Flags: full / empty are registered and pessimistic (CDC latency of ~2-3
//         cycles), so no overflow / underflow is possible.
//------------------------------------------------------------------------------
module asynchronous_fifo #(
  parameter int DEPTH      = 8,
  parameter int DATA_WIDTH = 8
) (
  input  logic                   wclk,
  input  logic                   wrst_n,
  input  logic                   rclk,
  input  logic                   rrst_n,
  input  logic                   w_en,
  input  logic                   r_en,
  input  logic [DATA_WIDTH-1:0]  data_in,
  output logic [DATA_WIDTH-1:0]  data_out,
  output logic                   full,
  output logic                   empty
);

  localparam int PTR_WIDTH = $clog2(DEPTH);

  initial begin
    if (DEPTH < 4 || (DEPTH & (DEPTH - 1)) != 0)
      $fatal(1, "asynchronous_fifo: DEPTH (%0d) must be a power of 2 and >= 4", DEPTH);
  end

  logic [PTR_WIDTH:0] g_wptr_sync, g_rptr_sync;
  logic [PTR_WIDTH:0] b_wptr, b_rptr;
  logic [PTR_WIDTH:0] g_wptr, g_rptr;

  // write pointer -> read clock domain
  synchronizer #(.WIDTH(PTR_WIDTH+1)) sync_wptr (
    .clk   (rclk),
    .rst_n (rrst_n),
    .d_in  (g_wptr),
    .d_out (g_wptr_sync)
  );

  // read pointer -> write clock domain
  synchronizer #(.WIDTH(PTR_WIDTH+1)) sync_rptr (
    .clk   (wclk),
    .rst_n (wrst_n),
    .d_in  (g_rptr),
    .d_out (g_rptr_sync)
  );

  wptr_handler #(.PTR_WIDTH(PTR_WIDTH)) wptr_h (
    .wclk        (wclk),
    .wrst_n      (wrst_n),
    .w_en        (w_en),
    .g_rptr_sync (g_rptr_sync),
    .b_wptr      (b_wptr),
    .g_wptr      (g_wptr),
    .full        (full)
  );

  rptr_handler #(.PTR_WIDTH(PTR_WIDTH)) rptr_h (
    .rclk        (rclk),
    .rrst_n      (rrst_n),
    .r_en        (r_en),
    .g_wptr_sync (g_wptr_sync),
    .b_rptr      (b_rptr),
    .g_rptr      (g_rptr),
    .empty       (empty)
  );

  fifo_mem #(.DEPTH(DEPTH), .DATA_WIDTH(DATA_WIDTH), .PTR_WIDTH(PTR_WIDTH)) fifom (
    .wclk     (wclk),
    .w_en     (w_en),
    .full     (full),
    .b_wptr   (b_wptr),
    .b_rptr   (b_rptr),
    .data_in  (data_in),
    .data_out (data_out)
  );

endmodule
