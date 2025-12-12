`timescale 1ns / 1ps

module cic_dynamic_scaler #(
    parameter integer RX_WIDTH  = 224,     // Input width (112 I + 112 Q)
    parameter integer OUT_WIDTH = 64       // Output width (32 I + 32 Q)
)(
    input  wire                      clk,
    input  wire                      rst_n,      // Synchronous Reset (Low Active assumed)

    // --- Channel 0 Input ---
    input  wire                      rx0_axis_tvalid_i,
    output wire                      rx0_axis_tready_o,
    input  wire [RX_WIDTH-1:0]       rx0_axis_tdata_i,
    
    // --- Channel 0 Output ---
    input  wire                      rx0_axis_tready_i,
    output reg                       rx0_axis_tvalid_o,
    output reg  [OUT_WIDTH-1:0]      rx0_axis_tdata_o,

    // --- Channel 1 Input ---
    input  wire                      rx1_axis_tvalid_i,
    output wire                      rx1_axis_tready_o,
    input  wire [RX_WIDTH-1:0]       rx1_axis_tdata_i,
    
    // --- Channel 1 Output ---
    input  wire                      rx1_axis_tready_i,
    output reg                       rx1_axis_tvalid_o,
    output reg  [OUT_WIDTH-1:0]      rx1_axis_tdata_o,

    // --- Rate Control Interface ---
    input  wire [15:0]               rx0_rate_axis_tdata_i, 
    input  wire                      rx0_rate_axis_tvalid_i,
    
    input  wire [15:0]               rx1_rate_axis_tdata_i, 
    input  wire                      rx1_rate_axis_tvalid_i
);

    // Fixed CIC Parameter
    localparam integer N = 6;
    localparam integer HALF_WIDTH = RX_WIDTH / 2; // 112

    // LUT / Shift Calculation Logic
    function [7:0] calc_shift_amount;
        input [15:0] rate;
        integer i;
        reg [4:0] idx; 
        begin
            idx = 0;
            // Priority Encoder to find MSB location
            for (i = 0; i < 16; i = i + 1) begin
                if (rate[i]) begin
                    idx = i[4:0]; // Cast integer loop var to 5-bit
                end
            end
            calc_shift_amount = idx * N;
        end
    endfunction

    reg [7:0] shift_amt_0;
    reg [7:0] shift_amt_1;

    // Shift Amount Update (Control Path)
    always @(posedge clk) begin
        if (!rst_n) begin
            shift_amt_0 <= 8'd0;
            shift_amt_1 <= 8'd0;
        end else begin
            if (rx0_rate_axis_tvalid_i) begin
                shift_amt_0 <= calc_shift_amount(rx0_rate_axis_tdata_i);
            end
            if (rx1_rate_axis_tvalid_i) begin
                shift_amt_1 <= calc_shift_amount(rx1_rate_axis_tdata_i);
            end
        end
    end

    // 2. Channel 0 Processing
    // -----------------------------------------------------------
    wire [HALF_WIDTH-1:0] ch0_i_raw;
    wire [HALF_WIDTH-1:0] ch0_q_raw;
    wire [HALF_WIDTH-1:0] ch0_i_shifted;
    wire [HALF_WIDTH-1:0] ch0_q_shifted;
    wire [OUT_WIDTH-1:0]  ch0_data_comb;

    // 1. Split
    assign ch0_i_raw = rx0_axis_tdata_i[HALF_WIDTH-1:0];
    assign ch0_q_raw = rx0_axis_tdata_i[RX_WIDTH-1:HALF_WIDTH];

    // 2. Arithmetic Shift (Sign Extension is automatic with $signed)
    assign ch0_i_shifted = $signed(ch0_i_raw) >>> shift_amt_0;
    assign ch0_q_shifted = $signed(ch0_q_raw) >>> shift_amt_0;

    // 3. Truncate & Pack (Take low 32 bits which includes the new sign bit)
    assign ch0_data_comb[31:0]  = ch0_i_shifted[31:0];
    assign ch0_data_comb[63:32] = ch0_q_shifted[31:0];

    // -----------------------------------------------------------
    // ready_o logic: We can accept data if downstream is ready 
    // OR if we are currently holding invalid data (bubble).
    wire ch0_ready_o_int;
    assign ch0_ready_o_int   = rx0_axis_tready_i || (~rx0_axis_tvalid_o);
    assign rx0_axis_tready_o = ch0_ready_o_int;

    always @(posedge clk) begin
        if (!rst_n) begin
            rx0_axis_tvalid_o <= 1'b0;
            rx0_axis_tdata_o  <= {OUT_WIDTH{1'b0}};
        end else begin
            // Handshake: Only update pipeline register when we are ready to accept
            if (ch0_ready_o_int) begin
                rx0_axis_tvalid_o <= rx0_axis_tvalid_i;
                // If input is valid, capture the COMBINATORIAL result
                if (rx0_axis_tvalid_i) begin
                    rx0_axis_tdata_o <= ch0_data_comb;
                end
            end
        end
    end

    // 3. Channel 1 Processing (Duplicate of Channel 0)
    // Combinatorial Logic
    wire [HALF_WIDTH-1:0] ch1_i_raw;
    wire [HALF_WIDTH-1:0] ch1_q_raw;
    wire [HALF_WIDTH-1:0] ch1_i_shifted;
    wire [HALF_WIDTH-1:0] ch1_q_shifted;
    wire [OUT_WIDTH-1:0]  ch1_data_comb;

    assign ch1_i_raw = rx1_axis_tdata_i[HALF_WIDTH-1:0];
    assign ch1_q_raw = rx1_axis_tdata_i[RX_WIDTH-1:HALF_WIDTH];

    assign ch1_i_shifted = $signed(ch1_i_raw) >>> shift_amt_1;
    assign ch1_q_shifted = $signed(ch1_q_raw) >>> shift_amt_1;

    assign ch1_data_comb[31:0]  = ch1_i_shifted[31:0];
    assign ch1_data_comb[63:32] = ch1_q_shifted[31:0];

    // Handshake & Sequential Logic
    wire ch1_ready_o_int;
    assign ch1_ready_o_int   = rx1_axis_tready_i || (~rx1_axis_tvalid_o);
    assign rx1_axis_tready_o = ch1_ready_o_int;

    always @(posedge clk) begin
        if (!rst_n) begin
            rx1_axis_tvalid_o <= 1'b0;
            rx1_axis_tdata_o  <= {OUT_WIDTH{1'b0}};
        end else begin
            if (ch1_ready_o_int) begin
                rx1_axis_tvalid_o <= rx1_axis_tvalid_i;
                if (rx1_axis_tvalid_i) begin
                    rx1_axis_tdata_o <= ch1_data_comb;
                end
            end
        end
    end

endmodule