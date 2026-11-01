`timescale 1ns/1ps

`ifndef FIFO_DEPTH
  `define FIFO_DEPTH 8
`endif
`ifndef FIFO_DW
  `define FIFO_DW 8
`endif

//------------------------------------------------------------------------------
// Interface
//------------------------------------------------------------------------------
interface async_fifo_if #(parameter int DW) (input logic wclk, input logic rclk);

  logic          wrst_n, rrst_n;
  logic          w_en, r_en;
  logic [DW-1:0] data_in, data_out;
  logic          full, empty;

  initial begin
    wrst_n  = 1'b0;
    rrst_n  = 1'b0;
    w_en    = 1'b0;
    r_en    = 1'b0;
    data_in = '0;
  end

  // Drivers drive with non-blocking assignments at posedge; monitors sample at the
  // same posedge in the active region, so they always see pre-edge (race-free) values.
  modport DRV_W (input wclk, output w_en, data_in);
  modport DRV_R (input rclk, output r_en);
  modport MON_W (input wclk, wrst_n, w_en, data_in, full);
  modport MON_R (input rclk, rrst_n, r_en, data_out, empty);

  // Reset helpers: asynchronous assert, synchronous (per-domain) release
  task automatic assert_reset();
    wrst_n = 1'b0;
    rrst_n = 1'b0;
  endtask

  task automatic release_reset(int wcycles = 4, int rcycles = 6);
    fork
      begin repeat (wcycles) @(posedge wclk); wrst_n <= 1'b1; end
      begin repeat (rcycles) @(posedge rclk); rrst_n <= 1'b1; end
    join
  endtask

endinterface

//------------------------------------------------------------------------------
// Package: transaction, generator, drivers, monitors, scoreboard, coverage, env
//------------------------------------------------------------------------------
package async_fifo_pkg;

  parameter int DEPTH = `FIFO_DEPTH;
  parameter int DW    = `FIFO_DW;

  localparam bit [DW-1:0] ALL1 = '1;

  typedef virtual async_fifo_if #(DW) vif_t;

  typedef enum int {WFAST, RFAST, NEAR_EQ, EQUAL} clk_mode_e;

  //--------------------------------------------------------------------------
  // Transaction: one clock cycle of stimulus (used for both write and read side)
  //--------------------------------------------------------------------------
  class fifo_txn;
    rand bit [DW-1:0] data;
    bit               en;
    int unsigned      pct = 50;    // probability (%) that en == 1

    // corner values (all-0 / all-1) get extra weight
    constraint c_data { data dist {{DW{1'b0}} := 5, ALL1 := 5, [1:ALL1-1] :/ 90}; }

    function void post_randomize();
      en = ($urandom_range(99, 0) < pct);   // activity probability (portable across simulators)
    endfunction
  endclass

  //--------------------------------------------------------------------------
  // Generator
  //--------------------------------------------------------------------------
  class fifo_gen;
    mailbox #(fifo_txn) mbx;

    function new(mailbox #(fifo_txn) m);
      mbx = m;
    endfunction

    task run(int unsigned n, int unsigned pct);
      fifo_txn t;
      repeat (n) begin
        t     = new();
        t.pct = pct;
        if (!t.randomize()) $fatal(1, "fifo_gen: randomize() failed");
        mbx.put(t);
      end
    endtask
  endclass

  //--------------------------------------------------------------------------
  // Coverage
  //--------------------------------------------------------------------------
  class fifo_cov;
    clk_mode_e   mode = WFAST;
    bit          w_en_s, w_full_s;
    bit          r_en_s, r_empty_s;
    int unsigned w_occ, r_occ;

`ifndef VERILATOR   // Verilator has no covergroup support; xsim/Questa/VCS do
    covergroup cg_wr;
      option.per_instance = 1;
      cp_en   : coverpoint w_en_s;
      cp_full : coverpoint w_full_s;
      cp_mode : coverpoint mode;
      cp_occ  : coverpoint w_occ {
        bins empty = {0};
        bins low   = {[1 : DEPTH/2 - 1]};
        bins high  = {[DEPTH/2 : DEPTH - 1]};
        bins full  = {DEPTH};
      }
      x_en_full : cross cp_en, cp_full;     // incl. write attempted while full
      x_en_occ  : cross cp_en, cp_occ;
      x_mode_full : cross cp_mode, cp_full;
    endgroup

    covergroup cg_rd;
      option.per_instance = 1;
      cp_en    : coverpoint r_en_s;
      cp_empty : coverpoint r_empty_s;
      cp_mode  : coverpoint mode;
      cp_occ   : coverpoint r_occ {
        bins empty = {0};
        bins low   = {[1 : DEPTH/2 - 1]};
        bins high  = {[DEPTH/2 : DEPTH - 1]};
        bins full  = {DEPTH};
      }
      x_en_empty : cross cp_en, cp_empty;   // incl. read attempted while empty
      x_en_occ   : cross cp_en, cp_occ;
      x_mode_empty : cross cp_mode, cp_empty;
    endgroup

    function new();
      cg_wr = new();
      cg_rd = new();
    endfunction

    function void sample_w(bit en, bit full, int unsigned occ);
      w_en_s = en; w_full_s = full; w_occ = occ;
      cg_wr.sample();
    endfunction

    function void sample_r(bit en, bit empty, int unsigned occ);
      r_en_s = en; r_empty_s = empty; r_occ = occ;
      cg_rd.sample();
    endfunction

    function real wr_cov(); return cg_wr.get_inst_coverage(); endfunction
    function real rd_cov(); return cg_rd.get_inst_coverage(); endfunction
`else
    function void sample_w(bit en, bit full, int unsigned occ); endfunction
    function void sample_r(bit en, bit empty, int unsigned occ); endfunction
    function real wr_cov(); return 0.0; endfunction
    function real rd_cov(); return 0.0; endfunction
`endif
    function real total();
      return (wr_cov() + rd_cov()) / 2.0;
    endfunction
  endclass

  //--------------------------------------------------------------------------
  // Scoreboard (reference model = queue)
  //--------------------------------------------------------------------------
  class scoreboard;
    bit [DW-1:0] q[$];
    int unsigned errors = 0;
    int unsigned n_wr = 0, n_rd = 0, n_wr_blocked = 0, n_rd_blocked = 0;
    int unsigned max_occ = 0;
    bit          verbose = 0;
    fifo_cov     cov;

    function new(fifo_cov c);
      cov = c;
    endfunction

    function void flush();
      q.delete();
    endfunction

    // called every wclk (out of reset) with pre-edge sampled values
    function void on_write(bit en, bit full, bit [DW-1:0] d);
      cov.sample_w(en, full, q.size());
      if (en && full) begin
        n_wr_blocked++;                       // DUT must ignore this write
      end else if (en) begin
        if (q.size() >= DEPTH) begin
          errors++;
          $error("[SB] %0t OVERFLOW: write accepted but FIFO already holds %0d words", $time, q.size());
        end else begin
          q.push_back(d);
          n_wr++;
          if (q.size() > max_occ) max_occ = q.size();
        end
      end
    endfunction

    // called every rclk (out of reset) with pre-edge sampled values
    function void on_read(bit en, bit empty, bit [DW-1:0] d);
      bit [DW-1:0] exp;
      cov.sample_r(en, empty, q.size());
      if (!empty && q.size() == 0) begin
        errors++;
        $error("[SB] %0t FALSE NON-EMPTY: empty=0 but model FIFO has no data", $time);
      end
      if (en && empty) begin
        n_rd_blocked++;                       // DUT must ignore this read
      end else if (en) begin
        if (q.size() == 0) begin
          errors++;
          $error("[SB] %0t UNDERFLOW: read accepted on empty model", $time);
        end else begin
          exp = q.pop_front();
          n_rd++;
          if (d !== exp) begin
            errors++;
            $error("[SB] %0t MISMATCH: expected %h, got %h", $time, exp, d);
          end else if (verbose) begin
            $display("[SB] %0t PASS: %h", $time, d);
          end
        end
      end
    endfunction
  endclass

  //--------------------------------------------------------------------------
  // Drivers
  //--------------------------------------------------------------------------
  class wr_driver;
    vif_t vif;
    mailbox #(fifo_txn)   mbx;

    function new(vif_t v, mailbox #(fifo_txn) m);
      vif = v; mbx = m;
    endfunction

    task run();
      fifo_txn t;
      forever begin
        @(posedge vif.wclk);
        if (mbx.try_get(t)) begin
          vif.w_en    <= t.en;
          vif.data_in <= t.data;
        end else begin
          vif.w_en    <= 1'b0;
        end
      end
    endtask
  endclass

  class rd_driver;
    vif_t vif;
    mailbox #(fifo_txn)   mbx;
    bit                   force_read = 0;   // used to drain the FIFO

    function new(vif_t v, mailbox #(fifo_txn) m);
      vif = v; mbx = m;
    endfunction

    task run();
      fifo_txn t;
      forever begin
        @(posedge vif.rclk);
        if (force_read)
          vif.r_en <= 1'b1;
        else if (mbx.try_get(t))
          vif.r_en <= t.en;
        else
          vif.r_en <= 1'b0;
      end
    endtask
  endclass

  //--------------------------------------------------------------------------
  // Monitors
  //--------------------------------------------------------------------------
  class wr_monitor;
    vif_t vif;
    scoreboard            sb;

    function new(vif_t v, scoreboard s);
      vif = v; sb = s;
    endfunction

    task run();
      forever begin
        @(posedge vif.wclk);
        if (vif.wrst_n === 1'b1)
          sb.on_write(vif.w_en, vif.full, vif.data_in);
      end
    endtask
  endclass

  class rd_monitor;
    vif_t vif;
    scoreboard            sb;

    function new(vif_t v, scoreboard s);
      vif = v; sb = s;
    endfunction

    task run();
      forever begin
        @(posedge vif.rclk);
        if (vif.rrst_n === 1'b1)
          sb.on_read(vif.r_en, vif.empty, vif.data_out);
      end
    endtask
  endclass

  //--------------------------------------------------------------------------
  // Environment
  //--------------------------------------------------------------------------
  class fifo_env;
    vif_t vif;

    mailbox #(fifo_txn) wmbx, rmbx;
    fifo_gen            wgen, rgen;
    wr_driver           wdrv;
    rd_driver           rdrv;
    wr_monitor          wmon;
    rd_monitor          rmon;
    scoreboard          sb;
    fifo_cov            cov;

    int unsigned        env_errors = 0;

    function new(vif_t v, bit verbose = 0);
      vif  = v;
      wmbx = new();
      rmbx = new();
      wgen = new(wmbx);
      rgen = new(rmbx);
      cov  = new();
      sb   = new(cov);
      sb.verbose = verbose;
      wdrv = new(vif, wmbx);
      rdrv = new(vif, rmbx);
      wmon = new(vif, sb);
      rmon = new(vif, sb);
    endfunction

    task start();
      fork
        wdrv.run();
        rdrv.run();
        wmon.run();
        rmon.run();
      join_none
    endtask

    // full reset sequence: assert, flush the model while both domains are in
    // reset, then release each domain on its own clock
    task do_reset(int wc = 4, int rc = 6);
      vif.assert_reset();
      repeat (2) @(posedge vif.wclk);
      repeat (2) @(posedge vif.rclk);
      check_reset_state();          // flags must be at reset values while reset is held
      sb.flush();
      vif.release_reset(wc, rc);
    endtask

    task check_reset_state();
      if (vif.empty !== 1'b1 || vif.full !== 1'b0) begin
        env_errors++;
        $error("[ENV] after reset expected empty=1 full=0, got empty=%b full=%b", vif.empty, vif.full);
      end
    endtask

    // push n cycles of stimulus on each side and wait until the drivers used it
    task traffic(string name, int unsigned wn, int unsigned wpct,
                              int unsigned rn, int unsigned rpct);
      $display("[ENV] %0t phase '%s': %0d wr cycles @%0d%%, %0d rd cycles @%0d%%",
               $time, name, wn, wpct, rn, rpct);
      fork
        wgen.run(wn, wpct);
        rgen.run(rn, rpct);
      join
      while (wmbx.num() > 0 || rmbx.num() > 0) @(posedge vif.wclk);
      repeat (4) @(posedge vif.wclk);
    endtask

    // read until the model is empty, then check the empty flag settles
    task drain(string name = "drain");
      int unsigned guard = 0;
      rdrv.force_read = 1;
      while (sb.q.size() > 0 && guard < 5000) begin
        @(posedge vif.rclk);
        guard++;
      end
      if (sb.q.size() > 0) begin
        env_errors++;
        $error("[ENV] %s: timeout, %0d words stuck in FIFO (false empty?)", name, sb.q.size());
      end
      rdrv.force_read = 0;
      repeat (8) @(posedge vif.rclk);
      if (vif.empty !== 1'b1) begin
        env_errors++;
        $error("[ENV] %s: empty flag not asserted after drain", name);
      end
    endtask

    function int unsigned total_errors();
      return sb.errors + env_errors;
    endfunction

    function void report();
      $display("");
      $display("================ ASYNC FIFO TB SUMMARY ================");
      $display(" DEPTH=%0d DATA_WIDTH=%0d", DEPTH, DW);
      $display(" writes accepted : %0d   (blocked while full  : %0d)", sb.n_wr, sb.n_wr_blocked);
      $display(" reads  accepted : %0d   (blocked while empty : %0d)", sb.n_rd, sb.n_rd_blocked);
      $display(" max occupancy   : %0d / %0d", sb.max_occ, DEPTH);
      $display(" functional cov  : %0.2f %%   (wr %0.2f %%, rd %0.2f %%)",
               cov.total(), cov.wr_cov(), cov.rd_cov());
      $display("=======================================================");
    endfunction
  endclass

endpackage

//------------------------------------------------------------------------------
// Top
//------------------------------------------------------------------------------
module async_fifo_tb;
  import async_fifo_pkg::*;

  logic wclk = 1'b0;
  logic rclk = 1'b0;
  real  wclk_half = 5.0;     // 100 MHz
  real  rclk_half = 17.5;    // ~28.6 MHz (same as the original TB: 10ns / 35ns period)

  always #(wclk_half) wclk = ~wclk;
  always #(rclk_half) rclk = ~rclk;

  async_fifo_if #(.DW(DW)) fif (.wclk(wclk), .rclk(rclk));

  asynchronous_fifo #(.DEPTH(DEPTH), .DATA_WIDTH(DW)) dut (
    .wclk     (wclk),
    .wrst_n   (fif.wrst_n),
    .rclk     (rclk),
    .rrst_n   (fif.rrst_n),
    .w_en     (fif.w_en),
    .r_en     (fif.r_en),
    .data_in  (fif.data_in),
    .data_out (fif.data_out),
    .full     (fif.full),
    .empty    (fif.empty)
  );

  //--------------------------------------------------------------------------
  // Assertions
  //--------------------------------------------------------------------------
  int unsigned assert_errors = 0;

  function automatic void sva_fail(string msg);
    assert_errors++;
    $error("%s", msg);
  endfunction

  // gray-coded pointers must change by at most 1 bit per own-clock cycle
  a_gray_w : assert property (@(posedge wclk) disable iff (!fif.wrst_n)
      $countones(dut.g_wptr ^ $past(dut.g_wptr)) <= 1)
    else sva_fail("[SVA] g_wptr changed by more than 1 bit");

  a_gray_r : assert property (@(posedge rclk) disable iff (!fif.rrst_n)
      $countones(dut.g_rptr ^ $past(dut.g_rptr)) <= 1)
    else sva_fail("[SVA] g_rptr changed by more than 1 bit");

  // writes while full / reads while empty must not move the pointers
  a_no_wr_when_full : assert property (@(posedge wclk) disable iff (!fif.wrst_n)
      (fif.w_en && fif.full) |=> $stable(dut.b_wptr))
    else sva_fail("[SVA] write pointer moved while full");

  a_no_rd_when_empty : assert property (@(posedge rclk) disable iff (!fif.rrst_n)
      (fif.r_en && fif.empty) |=> $stable(dut.b_rptr))
    else sva_fail("[SVA] read pointer moved while empty");

  // reset values
  a_wrst : assert property (@(posedge wclk)
      !fif.wrst_n |=> (!fif.full && dut.b_wptr == '0))
    else sva_fail("[SVA] write side not reset correctly");

  a_rrst : assert property (@(posedge rclk)
      !fif.rrst_n |=> (fif.empty && dut.b_rptr == '0))
    else sva_fail("[SVA] read side not reset correctly");

  // flags / data never X once out of reset
  a_full_known : assert property (@(posedge wclk) disable iff (!fif.wrst_n)
      !$isunknown(fif.full))
    else sva_fail("[SVA] full is X");

  a_empty_known : assert property (@(posedge rclk) disable iff (!fif.rrst_n)
      !$isunknown(fif.empty))
    else sva_fail("[SVA] empty is X");

  a_data_known : assert property (@(posedge rclk) disable iff (!fif.rrst_n)
      !fif.empty |-> !$isunknown(fif.data_out))
    else sva_fail("[SVA] data_out is X while !empty");

  //--------------------------------------------------------------------------
  // Test
  //--------------------------------------------------------------------------
  fifo_env env;

  task automatic set_clocks(real wp, real rp, clk_mode_e m);
    wclk_half    = wp / 2.0;
    rclk_half    = rp / 2.0;
    env.cov.mode = m;
    $display("[TB ] %0t clocks: wclk=%0.2f ns, rclk=%0.2f ns (%s)", $time, wp, rp, m.name());
  endtask

  task automatic run_mode(clk_mode_e m, real wp, real rp);
    set_clocks(wp, rp, m);
    env.traffic($sformatf("%s write-heavy", m.name()), 400, 90, 400, 30);
    env.drain();
    env.traffic($sformatf("%s read-heavy",  m.name()), 400, 30, 400, 90);
    env.drain();
    env.traffic($sformatf("%s balanced",    m.name()), 600, 50, 600, 50);
    env.drain();
  endtask

  initial begin
    // XSim compatibility: omit the explicit $urandom(seed) call.
    // Random stimulus still uses randomize() and $urandom_range() below.
    env = new(fif, $test$plusargs("VERBOSE"));
    env.start();

    // ---- 1. reset ---------------------------------------------------------
    env.do_reset(6, 8);

    // ---- 2. single write / read -------------------------------------------
    set_clocks(10.0, 35.0, WFAST);
    env.traffic("single word", 1, 100, 0, 0);
    env.drain("single word");

    // ---- 3. fill to full, overflow attempts, drain, underflow attempts ----
    env.traffic("fill + overflow attempts", DEPTH + 4, 100, 0, 0);
    if (fif.full !== 1'b1 || env.sb.q.size() != DEPTH) begin
      env.env_errors++;
      $error("[TB ] expected full with %0d words, got full=%b words=%0d",
             DEPTH, fif.full, env.sb.q.size());
    end
    env.traffic("underflow attempts while draining", 0, 0, DEPTH + 4, 100);
    env.drain("fill/drain");

    // ---- 4. clock-ratio sweep ---------------------------------------------
    run_mode(WFAST,   10.0, 35.0);   // write clock much faster
    run_mode(RFAST,   35.0, 10.0);   // read clock much faster
    run_mode(NEAR_EQ, 10.0, 10.7);   // nearly equal, phase drifts
    run_mode(EQUAL,   10.0, 10.0);   // same frequency, fixed phase

    // ---- 5. back-to-back full throughput ----------------------------------
    set_clocks(10.0, 10.0, EQUAL);
    env.traffic("back-to-back", 500, 100, 500, 100);
    env.drain("back-to-back");

    // ---- 6. reset while traffic is in flight ------------------------------
    set_clocks(10.0, 25.0, WFAST);
    fork
      env.wgen.run(300, 80);
      env.rgen.run(300, 60);
    join_none
    repeat (40) @(posedge wclk);
    $display("[ENV] %0t mid-traffic reset", $time);
    env.do_reset(4, 6);
    while (env.wmbx.num() > 0 || env.rmbx.num() > 0) @(posedge wclk);
    repeat (4) @(posedge wclk);
    env.drain("post-reset leftover");

    // ---- 7. random sanity after reset -------------------------------------
    env.traffic("post-reset random", 500, 60, 500, 60);
    env.drain("post-reset random");

    // ---- report -----------------------------------------------------------
    repeat (10) @(posedge rclk);
    env.report();
    if (env.total_errors() == 0 && assert_errors == 0)
      $display("\n*** TEST PASSED ***\n");
    else
      $display("\n*** TEST FAILED *** (scoreboard/env errors: %0d, assertion errors: %0d)\n",
               env.total_errors(), assert_errors);
    $finish;
  end

  // watchdog
  initial begin
    #20ms;
    $error("[TB ] watchdog timeout");
    $display("\n*** TEST FAILED *** (timeout)\n");
    $finish;
  end

  // waves
  initial begin
    $dumpfile("async_fifo.vcd");
    $dumpvars(0, async_fifo_tb);
  end

endmodule
