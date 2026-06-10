//
// cic_interp - Complex (I/Q) Cascaded Integrator-Comb INTERPOLATION filter,
//              order 5, with a RUNTIME-SELECTABLE interpolation factor.
//
// Ported from openHPSDR / orion2 Polyphase_FIR/CicInterpM5.v (Phil Harman
// VK6PH, GPLv2) -- structure preserved verbatim, with two changes for the
// bladeRF HPSDR DUC:
//
//   1. The interpolation ratio RRRR becomes a runtime input `interpolation`
//      (compare cic.v's runtime `decimation`), so one instance covers the
//      whole 48k..1536k DUC rate set without re-synthesis.
//
//   2. The fixed output bit-window (orion2 selects y5[CBITS-1 -: OBITS] for
//      its single RRRR) becomes a runtime window whose MSB tracks the CIC
//      interpolation gain.  A 5-stage interpolator with differential delay
//      M=1 has DC gain R^(N-1) = R^4, i.e. it occupies 4*log2(R) bits above
//      the input word.  To hold ~unity gain across rates we slide the OBITS
//      window so its top bit sits at (IBITS-1) + 4*log2(R):
//
//         R    4*log2(R)   msb = (IBITS-1)+4*log2(R)
//          8       12              23+12 = 35
//         16       16              23+16 = 39
//         32       20              23+20 = 43
//         64       24              23+24 = 47   (192 kHz, spec default)
//        128       28              23+28 = 51
//        256       32              23+32 = 55   = CBITS-1
//
//      GBITS is sized for the largest supported R (256 -> 32), so at R=256
//      the window is the top OBITS bits, exactly as orion2 does for its
//      fixed ratio.
//
// Like the decimating cic.v this block has NO functional reset on the
// integrator/comb state -- it relies on Cyclone V config-time zero init and
// on the upstream FIFO being cleared (aclr) whenever the DUC is idle (not
// PTT), so any stale accumulator state is flushed between transmissions.
//
// Timing model (identical to CicInterpM5): runs on `clock`, advances one
// output per `clock_en`.  Every `interpolation` outputs it pulses `req` for
// one enabled cycle and latches a fresh (x_real, x_imag) input; the caller
// presents the next input on the cycle after req (show-ahead FIFO).
//
module cic_interp(clock, clock_en, interpolation, req,
                  x_real, x_imag, y_real, y_imag);

  // design parameters
  parameter IBITS       = 24;   // input I/Q sample width (HPSDR DUC = 24)
  parameter OBITS       = 16;   // output sample width (AD9361 DAC field = 16)
  parameter STAGES      = 5;    // CIC order (comb + integrator sections)
  parameter MAX_INTERP  = 256;  // 12.288 MHz / 48 kHz
  parameter MIN_INTERP  = 8;    // 12.288 MHz / 1536 kHz

  // growth bits: (STAGES-1)*log2(MAX_INTERP); $clog2 rounds up as required.
  parameter GBITS       = (STAGES-1) * $clog2(MAX_INTERP);
  localparam CBITS      = IBITS + GBITS;             // accumulator width

  // counter must reach MAX_INTERP-1.
  localparam CNT_W      = $clog2(MAX_INTERP);

  input clock;
  input clock_en;                                    // enable an output sample
  input [CNT_W:0] interpolation;                     // runtime ratio R
  output reg req;                                    // request next input
  input signed [IBITS-1:0] x_real;
  input signed [IBITS-1:0] x_imag;
  output reg signed [OBITS-1:0] y_real;
  output reg signed [OBITS-1:0] y_imag;

  reg [CNT_W-1:0] counter;

  // comb (low-rate) + integrator (high-rate) state, real and imaginary.
  reg signed [CBITS-1:0] x0, x1, x2, x3, x4, x5, dx0, dx1, dx2, dx3, dx4;
  reg signed [CBITS-1:0] y1, y2, y3, y4, y5;
  reg signed [CBITS-1:0] q0, q1, q2, q3, q4, q5, dq0, dq1, dq2, dq3, dq4;
  reg signed [CBITS-1:0] s1, s2, s3, s4, s5;

  wire signed [CBITS-1:0] sxtxr, sxtxi;
  assign sxtxr = {{(CBITS - IBITS){x_real[IBITS-1]}}, x_real};   // sign extended
  assign sxtxi = {{(CBITS - IBITS){x_imag[IBITS-1]}}, x_imag};

  // Runtime output-window MSB = (IBITS-1) + (STAGES-1)*log2(interpolation).
  // log2() of the (power-of-two) ratio resolved by a small case so the
  // part-select base is a clean integer; everything outside the supported
  // set falls back to the 192 kHz window (R=64).
  reg [7:0] msb;
  always @(*) begin
    case (interpolation)
      'd8:   msb = (IBITS-1) + (STAGES-1)*3;   // 1536 kHz
      'd16:  msb = (IBITS-1) + (STAGES-1)*4;   //  768 kHz
      'd32:  msb = (IBITS-1) + (STAGES-1)*5;   //  384 kHz
      'd64:  msb = (IBITS-1) + (STAGES-1)*6;   //  192 kHz (default)
      'd128: msb = (IBITS-1) + (STAGES-1)*7;   //   96 kHz
      'd256: msb = (IBITS-1) + (STAGES-1)*8;   //   48 kHz
      default: msb = (IBITS-1) + (STAGES-1)*6; //  fall back to 192 kHz
    endcase
  end

  initial begin
    counter = 0;
    req     = 0;
    y_real  = 0;
    y_imag  = 0;
  end

  always @(posedge clock) begin
    if (clock_en) begin
      // (x0,q0) -> comb -> (x5,q5) -> zero-stuff/interpolate -> integrate
      if (counter == (interpolation - 1'b1)) begin
        counter <= 0;
        x0 <= sxtxr;
        q0 <= sxtxi;
        req <= 1'b1;
        // comb, real
        x1 <= x0 - dx0;
        x2 <= x1 - dx1;
        x3 <= x2 - dx2;
        x4 <= x3 - dx3;
        x5 <= x4 - dx4;
        dx0 <= x0; dx1 <= x1; dx2 <= x2; dx3 <= x3; dx4 <= x4;
        // comb, imaginary
        q1 <= q0 - dq0;
        q2 <= q1 - dq1;
        q3 <= q2 - dq2;
        q4 <= q3 - dq3;
        q5 <= q4 - dq4;
        dq0 <= q0; dq1 <= q1; dq2 <= q2; dq3 <= q3; dq4 <= q4;
      end
      else begin
        counter <= counter + 1'b1;
        x5 <= 0;                 // stuff a zero between comb samples
        q5 <= 0;
        req <= 1'b0;
      end
      // integrators (high rate), real then imaginary
      y1 <= y1 + x5;
      y2 <= y2 + y1;
      y3 <= y3 + y2;
      y4 <= y4 + y3;
      y5 <= y5 + y4;
      s1 <= s1 + q5;
      s2 <= s2 + s1;
      s3 <= s3 + s2;
      s4 <= s4 + s3;
      s5 <= s5 + s4;
      // registered output window (rounded), tracks the interpolation gain.
      y_real <= y5[msb -: OBITS] + y5[msb - OBITS];
      y_imag <= s5[msb -: OBITS] + s5[msb - OBITS];
    end
    else begin
      req <= 1'b0;
    end
  end
endmodule
