`timescale 1ns/1ps
//------------------------------------------------------------------------------
// Read pointer handler (read clock domain)
//  - empty is registered and computed from the NEXT gray pointer (pessimistic:
//    may stay high a few cycles after a write, never low when the FIFO is empty)
//------------------------------------------------------------------------------
module rptr_handler #(
  parameter int PTR_WIDTH = 3
) (
  input  logic                 rclk,
  input  logic                 rrst_n,
  input  logic                 r_en,
  input  logic [PTR_WIDTH:0]   g_wptr_sync,   // write gray pointer, synced into rclk
  output logic [PTR_WIDTH:0]   b_rptr,
  output logic [PTR_WIDTH:0]   g_rptr,
  output logic                 empty
);

  logic [PTR_WIDTH:0] b_rptr_next;
  logic [PTR_WIDTH:0] g_rptr_next;
  logic               rempty;

  assign b_rptr_next = b_rptr + {{PTR_WIDTH{1'b0}}, (r_en & ~empty)};
  assign g_rptr_next = (b_rptr_next >> 1) ^ b_rptr_next;

  assign rempty = (g_wptr_sync == g_rptr_next);

  always_ff @(posedge rclk or negedge rrst_n) begin
    if (!rrst_n) begin
      b_rptr <= '0;
      g_rptr <= '0;
      empty  <= 1'b1;
    end else begin
      b_rptr <= b_rptr_next;
      g_rptr <= g_rptr_next;
      empty  <= rempty;
    end
  end

endmodule
