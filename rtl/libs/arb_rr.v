`include "soc.vh"

// This module implements an high speed round robin arbiter

module arb_rr #(parameter NUM_REQS=4) (
    input   logic                   clock,
    input   logic                   reset,
    output  logic [NUM_REQS-1:0]    grants,
    input   logic                   pop,
    input   logic [NUM_REQS-1:0]    reqs
);

logic [NUM_REQS-1:0] mask_code;
logic [NUM_REQS-1:0] masked_reqs = mask_code & reqs;

logic [NUM_REQS-1:0] reqs_code;
logic [NUM_REQS-1:0] masked_reqs_code;

genvar bn;
generate
   for (bn = 0; bn < NUM_REQS; bn = bn + 1) 
   begin : reqs_code_gen_block
      assign reqs_code[bn]        = |(       reqs[0 +: (bn + 1)]);
      assign masked_reqs_code[bn] = |(masked_reqs[0 +: (bn + 1)]);
   end
endgenerate

logic any_masked_req = |masked_reqs;
logic [NUM_REQS-1:0] muxed_reqs_code = any_masked_req ? masked_reqs_code : reqs_code;
logic [NUM_REQS-1:0] mask_code_next;

generate
    // Check corner case of only one master
    if (NUM_REQS == 1) 
        assign mask_code_next = 1'b0;
    else
        assign mask_code_next = {muxed_reqs_code[0 +: (NUM_REQS-1)], 1'b0}; // shift muxed_reqs_code left by one-bit
endgenerate

assign grants = muxed_reqs_code ^ mask_code_next; // grant is simple xor to find one-hot location of selected reqs code

// mask_code
logic req_valid;
assign req_valid = (|reqs) && pop; //if there are requests and asked for a pop

    //     CLK    RST    EN         DOUT       DIN             DEF
`RST_EN_FF(clock, reset, req_valid, mask_code, mask_code_next, '0)

endmodule
