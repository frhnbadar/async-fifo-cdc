`timescale 1ns/1ps

module wptr_handler #(
  parameter int PTR_WIDTH = 3            // address bits; pointers are PTR_WIDTH+1 bits
) (
  input  logic                 wclk,
  input  logic                 wrst_n,
  input  logic                 w_en,
  input  logic [PTR_WIDTH:0]   g_rptr_sync,   // read gray pointer, synced into wclk
  output logic [PTR_WIDTH:0]   b_wptr,
  output logic [PTR_WIDTH:0]   g_wptr,
  output logic                 full
);

  logic [PTR_WIDTH:0] b_wptr_next;
  logic [PTR_WIDTH:0] g_wptr_next;
  logic               wfull;

  assign b_wptr_next = b_wptr + {{PTR_WIDTH{1'b0}}, (w_en & ~full)};
  assign g_wptr_next = (b_wptr_next >> 1) ^ b_wptr_next;

  // full: next write gray pointer == read gray pointer with the top 2 bits inverted
  assign wfull = (g_wptr_next == {~g_rptr_sync[PTR_WIDTH:PTR_WIDTH-1],
                                   g_rptr_sync[PTR_WIDTH-2:0]});

  always_ff @(posedge wclk or negedge wrst_n) begin
    if (!wrst_n) begin
      b_wptr <= '0;
      g_wptr <= '0;
      full   <= 1'b0;
    end else begin
      b_wptr <= b_wptr_next;
      g_wptr <= g_wptr_next;
      full   <= wfull;
    end
  end

endmodule
