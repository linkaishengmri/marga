// rx_gain_ctrl.sv
// -----------------------------------------------------------------------------
// Simple SPI-like serial driver for an attenuator/gain device.
// - Inputs:
//     clk             : system clock
//     rx_gain_write   : write strobe (rising-edge starts a 16-bit transfer)
//     rx_gain[5:0]    : payload (either address/sel or value bits)
//     rx_gain_sel     : 1 => address mode, 0 => value mode
// - Outputs:
//     gain_si_o       : serial data out (SI)  -- driven during transfer
//     gain_clk_o      : serial clock (CLK)    -- 50% duty, generated only during transfer
//     gain_le_o       : latch enable (LE)     -- pulsed at end of transfer
//     gain_sel_o[2:0] : latched select bits (from rx_gain[5:3] when rx_gain_sel==1)
//
// Internal registers:
//   reg [2:0] rx_gain_addr
//   reg [7:0] rx_gain_value
//
// Transfer word (16 bits, LSB-first):
//   {5'b00000, rx_gain_addr[2:0], rx_gain_value[7:0]}
// Behavior modeled on provided Arduino attu_write().
// -----------------------------------------------------------------------------
`ifndef _RX_GAIN_CTRL_
 `define _RX_GAIN_CTRL_

 `timescale 1ns / 1ns

module rx_gain_ctrl #(
    // ------------------------------------------------------------------------
    // Parameters - adjust timing here
    // DIV must be even for clean 50% duty (we toggle clock every DIV cycles).
    // When a transfer is active, clk will toggle every DIV system-clocks.
    // ------------------------------------------------------------------------
    parameter integer DIV = 32,
    parameter integer LE_PULSE_CYCLES = 32
) (
    input  wire        clk,
    input  wire        rx_gain_write,
    input  wire [5:0]  rx_gain,
    input  wire        rx_gain_sel,
    output reg         gain_si_o,
    output reg         gain_clk_o,
    output reg         gain_le_o,
    output reg [2:0]   gain_sel_o
);
    // ------------------------------------------------------------------------
    // Internal latches for address/value (no reset used per request)
    // ------------------------------------------------------------------------
    reg [2:0] rx_gain_addr;
    reg [7:0] rx_gain_value;

    // previous-cycle sample of rx_gain_write (one-clock delay), keep the delayed
    // implementation you liked.
    reg rx_gain_write_r;

    // combinational 16-bit word to write
    wire [15:0] rx_gain_to_write;
    assign rx_gain_to_write = {5'b0, rx_gain_addr, rx_gain_value};

    // shift register loaded at start of transfer
    reg [15:0] shift_reg;

    // state machine
    reg [1:0] state;
    parameter IDLE      = 2'b00;
    parameter SHIFT     = 2'b01;
    parameter LE_PULSE  = 2'b10;

    // bit index 0..15
    reg [3:0] bit_idx;

    // clock generator during SHIFT
    reg [($clog2(DIV)-1):0] div_cnt; // counter for generating gated clock
    // we also keep previous value of gain_clk_o for edge detection
    reg gain_clk_prev;

    // LE pulse counter
    reg [15:0] le_cnt; // wide enough for LE_PULSE_CYCLES

    // start pulse detection (combinational)
    wire start_pulse;
    assign start_pulse = rx_gain_write & ~rx_gain_write_r;

    // ---------------------------
    // Latch rx_gain inputs (no reset)
    // - if rx_gain_sel == 1: address mode -> latch addr and sel
    // - else: value mode -> latch rx_gain_value <= {1'b0, rx_gain, 1'b0}
    // This latching happens every clock edge (synchronous).
    // ---------------------------
    always @(posedge clk) begin
        rx_gain_write_r <= rx_gain_write; // one-cycle delayed copy (keep this)
        if (rx_gain_sel) begin
            rx_gain_addr <= rx_gain[2:0];
            gain_sel_o   <= rx_gain[5:3];
        end else begin
            rx_gain_value <= {1'b0, rx_gain, 1'b0};
        end
    end

    // ---------------------------
    // Clock generation (gated): only active when state == SHIFT.
    // - div_cnt counts 0 .. DIV-1; when reached, toggle gain_clk_o.
    // - when not in SHIFT, ensure gain_clk_o is low and div_cnt cleared.
    // ---------------------------
    always @(posedge clk) begin
        gain_clk_prev <= gain_clk_o; // capture previous value (for edge detect)

        if (state == SHIFT) begin
            if (div_cnt == DIV - 1) begin
                div_cnt <= 0;
                gain_clk_o <= ~gain_clk_o;
            end else begin
                div_cnt <= div_cnt + 1;
            end
        end else begin
            div_cnt <= 0;
            gain_clk_o <= 1'b0;
        end
    end

    // ---------------------------
    // Main FSM: IDLE -> SHIFT -> LE_PULSE -> IDLE
    // Transmission flow:
    //  - On start_pulse in IDLE: load shift_reg with rx_gain_to_write,
    //    present shift_reg[0] on SI, clear bit_idx=0, enter SHIFT.
    //  - In SHIFT, we mimic Arduino: set SI before raising CLK.
    //    We use the clock edges produced above:
    //      * When gain_clk falls (1->0): prepare next SI (next bit).
    //      * When gain_clk rises (0->1): external device samples SI.
    //    After the rising edge that samples bit[15], on subsequent falling we
    //    detect completion and move to LE_PULSE.
    //  - In LE_PULSE: assert LE for LE_PULSE_CYCLES system-clks, then clear.
    // ---------------------------
    always @(posedge clk) begin
        case (state)
            IDLE: begin
                gain_le_o <= 1'b0;
                gain_si_o <= 1'b0;
                bit_idx <= 4'd0;
                le_cnt <= 0;
                // detect start
                if (start_pulse) begin
                    // load shift register and present LSB immediately (SI set before first rising)
                    shift_reg <= rx_gain_to_write;
                    gain_si_o <= rx_gain_to_write[0];
                    bit_idx <= 4'd0;
                    // prepare clock generator (div_cnt already cleared by clock gen when not SHIFT)
                    state <= SHIFT;
                end
            end

            SHIFT: begin
                // edge detection using gain_clk_prev and gain_clk_o
                // falling edge: gain_clk_prev == 1 && gain_clk_o == 0
                if (gain_clk_prev && !gain_clk_o) begin
                    // after the falling edge we present the next bit (so it will be
                    // stable before next rising edge)
                    if (bit_idx == 4'd15) begin
                        // completed last bit -> move to LE pulse
                        gain_le_o <= 1'b1;
                        le_cnt <= 0;
                        state <= LE_PULSE;
                    end else begin
                        bit_idx <= bit_idx + 1;
                        gain_si_o <= shift_reg[bit_idx + 1];
                    end
                end
                // note: on rising edge external device samples gain_si_o; we do not
                // need to do anything on rising edge here.
            end

            LE_PULSE: begin
                // keep LE asserted for configured cycles, then release and return IDLE
                if (le_cnt < LE_PULSE_CYCLES - 1) begin
                    le_cnt <= le_cnt + 1;
                    gain_le_o <= 1'b1;
                end else begin
                    gain_le_o <= 1'b0;
                    le_cnt <= 0;
                    gain_si_o <= 1'b0;
                    state <= IDLE;
                end
            end

            default: begin
                state <= IDLE;
            end
        endcase
    end

    // initialize regs to sensible defaults for simulation / synthesis-friendly
    // (no async reset per your request)
    // initial begin
    //     rx_gain_addr   = 3'b000;
    //     rx_gain_value  = 8'h00;
    //     rx_gain_write_r = 1'b0;
    //     shift_reg      = 16'h0000;
    //     state          = IDLE;
    //     bit_idx        = 4'h0;
    //     div_cnt        = 0;
    //     gain_clk_o     = 1'b0;
    //     gain_clk_prev  = 1'b0;
    //     gain_si_o      = 1'b0;
    //     gain_le_o      = 1'b0;
    //     gain_sel_o     = 3'b000;
    //     le_cnt         = 0;
    // end

endmodule
`endif
