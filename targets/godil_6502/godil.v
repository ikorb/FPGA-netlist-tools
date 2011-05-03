// Top-level module for GODIL40_XC3S500E board

`define CLOCK_SHIFTER_LEN  5000
`define CLOCK_TAP_INITIAL  19
`define CLOCK_TAP_OFFSET   3550

module godil40_xc3s500e(
  input clk_49152mhz,
// start of 6502 pins on DIL40 connector
  output [15:0] ab,
  inout [7:0] db,
  input res,
  output rw,
  output sync,
  input so,
  input clk0,
  output clk1out,
  output clk2out,
  input rdy,
  input nmi,
  input irq,
// end of 6502 pins on DIL40 connector
  output [1:0] led,

  output pin_c1,
  output pin_a2,
  input  sw1,
  input  sw2,
  output tst_rclk,
  output tst_sclk,
  output tst_ser
);

// synchronized signals
   reg [7:0]  syn_db;
   reg        syn_res;
   reg        syn_so;
   reg        syn_clk0;
   reg        syn_rdy;
   reg        syn_nmi;
   reg        syn_irq;

// debounced buttons
   wire       btn1;
   wire       btn2;

// handle three-state data bus

  wire [7:0] db_i;
  wire [7:0] db_o;
  wire [7:0] db_t;  // not yet properly set by the 6502 model; instead use rw for the three-state enable for all db pins

  assign db_i = syn_db;
  assign db = rw ? 8'bz : db_o;

// create an emulation clock from clk_49152mhz

  wire eclk, ereset;

  clock_and_reset _clk(clk_49152mhz, eclk, ereset);

// synchronize external input signals
   always @(posedge eclk) begin
      syn_db   = db;
      syn_res  = res;
      syn_so   = so;
      syn_clk0 = clk0;
      syn_rdy  = rdy;
      syn_nmi  = nmi;
      syn_irq  = irq;
   end

// phi0 delay
   reg [`CLOCK_SHIFTER_LEN-1:0] clk0delay = 0;

   always @(posedge eclk)
     clk0delay <= {clk0delay[`CLOCK_SHIFTER_LEN-2:0], syn_clk0};
   
// debounce buttons
   debouncer _deb1(eclk,  sw1, btn1);
   debouncer _deb2(eclk, !sw2, btn2);

// blink an LED using eclk
  blink #(26) _blink0(eclk, led[0]);

  assign led[1] = !res;

// modify clock_tap via buttons
   reg [7:0]                    clock_tap = `CLOCK_TAP_INITIAL;
   reg                          button_lock = 0;

   always @(posedge eclk) begin
      if (!btn1 && !btn2)
        button_lock <= 0;
      else begin
         if (!button_lock && btn1) begin
            clock_tap <= clock_tap + 1;

            button_lock <= 1;
         end

         if (!button_lock && btn2) begin
            clock_tap <= clock_tap - 1;

            button_lock <= 1;
         end
      end // else: !if(!btn1 && !btn2)
   end // always @ (posedge eclk)
   
// calculate and display the difference between phi0 and phi2
   wire [15:0] diffticks;
   
   clock_difference _cdiff(eclk, syn_clk0, clk2out, diffticks, pin_c1, pin_a2);
   dy1 _display(eclk,
                (diffticks >> 8) & 15,(diffticks >> 4) & 15,diffticks & 15,
                (clock_tap >> 8) & 15,(clock_tap >> 4) & 15,clock_tap & 15,
                tst_rclk, tst_sclk, tst_ser);

// instantiate the 6502 model
   
  chip_6502 _chip_6502(eclk, ereset,
    ab[0], ab[1], ab[2], ab[3], ab[4], ab[5], ab[6], ab[7], ab[8], ab[9], ab[10], ab[11], ab[12], ab[13], ab[14], ab[15],
    db_i[0], db_o[0], db_t[0], db_i[1], db_o[1], db_t[1], db_i[2], db_o[2], db_t[2], db_i[3], db_o[3], db_t[3], 
    db_i[4], db_o[4], db_t[4], db_i[5], db_o[5], db_t[5], db_i[6], db_o[6], db_t[6], db_i[7], db_o[7], db_t[7], 
    syn_res, rw, sync, syn_so, clk0delay[clock_tap + `CLOCK_TAP_OFFSET], clk1out, clk2out, syn_rdy, syn_nmi, syn_irq);

endmodule

//
// Make emulation clock from on-board 49.152 MHz oscillator
//

module clock_and_reset(
  input clk_in,
  output eclk,
  output ereset
);

  wire clk_56mhz;
  dcm_mult #(8,7) _dcm0(clk_in, clk_56mhz);
  BUFG b0(.I(clk_56mhz), .O(eclk));

  reg [7:0] r = 8'd0;

  always @(posedge eclk)
    r <= {r[6:0], 1'b1};

  assign ereset = ~r[7];

endmodule

module dcm_mult(
  input clk_in,
  output clk_out
);

  parameter N = 2;
  parameter D = 2;

  wire clk_m;

  DCM_SP #(
   .CLKFX_DIVIDE(D),    // Can be any integer from 1 to 32
   .CLKFX_MULTIPLY(N), // Can be any integer from 2 to 32
   .STARTUP_WAIT("TRUE")    // Delay configuration DONE until DCM LOCK, TRUE/FALSE
) DCM_SP_inst (
   .CLKFX(clk_out),     // DCM CLK synthesis out (M/D)
   .CLKIN(clk_in)    // Clock input (from IBUFG, BUFG or DCM)
);

endmodule

module blink(
  input clk,
  output led
);

  parameter W = 8;

  reg [W-1:0] c;

  always @(posedge clk)
    c <= c + 1;

  assign led = c[W-1];

endmodule


//
// PLL to adjust the clock delay
// (unfinished)
//

// phase difference calculation
module clock_difference(
                 input eclk,
                 input phi0,
                 input phi2,
                 output integer diffticks,
                 output l0,
                 output l1
                 );

   parameter [2:0]
     STATE_IDLE  = 2'b00,
     STATE_WAIT0 = 2'b01,
     STATE_WAIT2 = 2'b10;

   integer                      curticks;
   reg [1:0]                    state = STATE_IDLE;

                          
   wire                         phi0long, phi2long;

   long_clock_finder _clk0long(phi0, eclk, phi0long);
   long_clock_finder _clk2long(phi2, eclk, phi2long);

   assign l0 = phi0long;
   assign l1 = phi2long;
   
   always @(posedge eclk) begin
      if (phi0long || phi2long) begin
         // something happened
         if (phi0long && phi2long) begin
            // both at the same time
            diffticks <= 0;
            state <= STATE_IDLE;
         end else begin
            // single event
            case (state)
              STATE_IDLE: begin
                 if (phi0long) begin
                    // first event on phi0
                    state <= STATE_WAIT2;
                    curticks <= 0;
                 end else begin
                    // first event on phi2
                    state <= STATE_WAIT0;
                    curticks <= 0;
                 end
              end // case: STATE_IDLE
              
              STATE_WAIT0: begin
                 if (phi0long) begin
                    // end event found
                    state <= STATE_IDLE;
                    diffticks <= curticks;
                 end else begin
                    // double start event found
                    curticks <= 0;
                 end
              end

              STATE_WAIT2: begin
                 if (phi0long) begin
                    // double start event found
                    curticks <= 0;
                 end else begin
                    // end event found
                    state <= STATE_IDLE;
                    diffticks <= -curticks;
                 end
              end

              default: state <= STATE_IDLE;
            endcase // case (state)
         end // else: !if(phi0long && phi2long)
      end else begin // if (phi0long || phi2long)
         if (state != STATE_IDLE)
           curticks <= curticks + 1;
      end // else: !if(phi0long || phi2long)
   end // always @ (posedge eclk)

endmodule // clock_pll


// outputs a single-clock signal when the falling edge of a long clock pulse is detected
module long_clock_finder(
                    input  clock,
                    input  eclk,
                    output reg longclock
                    );
   parameter LONGCOUNT = 30;
   
   reg [7:0]               ticks;
   reg                     prevstate;
      
   always @(posedge eclk) begin
      if (clock != prevstate) begin
         // clock edge
         if (clock) begin
            // clock is now high, reset counter
            ticks <= 8'd0;
         end else begin
            // clock is now low, check number of ticks
            if (ticks > LONGCOUNT) begin
               longclock <= 1;
            end
         end
      end else begin // if (clock != prevstate)
         longclock <= 0;
         if (clock) begin
            // clock is high, increment ticks
            ticks <= ticks + 1;
         end
      end // else: !if(clock != prevstate)
      prevstate = clock;
   end // always @ (posedge eclk)

endmodule // long_clock_finder


/* ---- 7-Segment-Display ---- */

module dy1(
           input eclk,
           input [3:0] digit1,
           input [3:0] digit2,
           input [3:0] digit3,
           input [3:0] digit4,
           input [3:0] digit5,
           input [3:0] digit6,
           output rclk,
           output sclk,
           output ser
           );

   wire [47:0]    segments;
   
   hex2segments seg3(digit1, segments[7:0]);
   hex2segments seg2(digit2, segments[15:8]);
   hex2segments seg1(digit3, segments[23:16]);

   hex2segments seg6(digit4, segments[31:24]);
   hex2segments seg5(digit5, segments[39:32]);
   hex2segments seg4(digit6, segments[47:40]);

   segmentshifter shifter(eclk, segments, rclk, sclk, ser);
  
endmodule // dy1
                         

module segmentshifter(
                      input eclk,
                      input [47:0] segments,
                      output reg rclk,
                      output reg sclk,
                      output reg ser
                      );
   parameter CLOCKDIV = 32;

   integer                   clkcount = 0;
   integer                   curseg = 0;
   
   always @(posedge eclk) begin
      if (clkcount < CLOCKDIV) begin
         if (clkcount == CLOCKDIV/2) begin
            // toggle SCLK in the middle
            sclk <= 1;
         end
            
         clkcount <= clkcount + 1;
      end else begin
         clkcount <= 0;

         // output current segment
         ser  <= !segments[curseg];
         sclk <= 0;

         if (curseg == 47) begin
            // last segment is now shifted in, copy to output
            rclk <= 1;
         end else begin
            // set rclk low to enable outputs
            rclk <= 0;
         end
         
         if (curseg > 0) begin
            curseg <= curseg - 1;
         end else begin
            // wrap
            curseg <= 47;
         end

      end // else: !if(clkcount < CLOCKDIV)
   end // always @ (posedge eclk)
   
endmodule // segmentshifter

module hex2segments(
                    input  [3:0] hexvalue,
                    output [7:0] segments
                    );

   /* rotate the contents of a 7-segment digit by 180 degrees */
   function [7:0] Rotate;
      input [7:0]                s;
      begin
         Rotate = {s[1],s[4],s[5],s[6],s[0],s[2],s[7],s[3]};
      end
   endfunction // case

   function [7:0] Decoder;
      input [3:0]                hexvalue;
      begin
         case (hexvalue)
            0: Decoder = 8'b11011011;
            1: Decoder = 8'b00001010;
            2: Decoder = 8'b11110010;
            3: Decoder = 8'b01111010;

            4: Decoder = 8'b00101011;
            5: Decoder = 8'b01111001;
            6: Decoder = 8'b11111001;
            7: Decoder = 8'b00011010;

            8: Decoder = 8'b11111011;
            9: Decoder = 8'b01111011;
           10: Decoder = 8'b10111011;
           11: Decoder = 8'b11101001;
     
           12: Decoder = 8'b11010001;
           13: Decoder = 8'b11101010;
           14: Decoder = 8'b11110001;
           15: Decoder = 8'b10110001;
         endcase // case (hexvalue)
      end
   endfunction // case

   assign segments = Rotate(Decoder(hexvalue));
      
endmodule // hex2segments

module debouncer(
                 input eclk,
                 input button,
                 output reg btn_debounce
                 );
   parameter REG_BITS = 8;

   reg [REG_BITS-1:0]       counter = 0;
   reg                      oldstate;

   always @(posedge eclk) begin
      if (button != oldstate) begin
         counter <= 0;
      end else begin
         if (counter < (1 << REG_BITS)-1)
           counter <= counter + 1;
         else begin
            btn_debounce <= button;
         end
      end
      oldstate <= button;
   end // always @ (posedge eclk)

endmodule // debouncer
