`include "soc.vh"

// This module is a generic fifo
module fifo 
#(
    parameter WIDTH = 8, 
    parameter DEPTH = 4)
(
        // System control signals
    input   logic                clock,
    input   logic                reset,
    output  logic                full,

        // Push
    input   logic                push,
    input   logic [WIDTH-1:0]    wdata,

        // Pop
    input   logic                pop,
    output  logic                not_empty,
    output  logic [WIDTH-1:0]    rdata
);

localparam MEM_DEPTH = DEPTH-1;
localparam PTR_W     = $clog2(MEM_DEPTH);


/////////////////////////////
// Control the number of positions filled

    // Level is an array where the asserted bit controls how many requests are
    // and where they're placed
logic [DEPTH:0] level;
logic [DEPTH:0] level_next;
logic level_en;

    //     CLK    RST    EN        DOUT   DIN         DEF
`RST_EN_FF(clock, reset, level_en, level, level_next, `ZX((DEPTH+1), 1'b1))

assign  full  = level[DEPTH];
assign  not_empty = !level[0];

// Control signals for push and pop
logic wdata_taken;
logic rdata_taken;

always_comb
begin
        // Push request
    wdata_taken = (push && !full);

        // Pop request
    rdata_taken = (pop  && not_empty);

        // Update level if there is a push OR a pop, not both
    level_en = wdata_taken ^ rdata_taken;

    level_next = (wdata_taken) ? {level[DEPTH-1:0], 1'b0} : // in case of push we increase the level
                                 {1'b0, level[DEPTH:1]};    // in case of pop  we decrease the level
end

/////////////////////////////
// Control when to enable data storage and update pointers accordingly

    // memory where data is stored
logic [MEM_DEPTH-1:0][WIDTH-1:0]   memory;

logic               wr_en_fifo;   // Storage enable
logic [PTR_W-1:0]   wr_pointer;   // Storage pointer
logic [PTR_W-1:0]   wr_pointer_next;

logic               rd_en_fifo; // Read enable
logic [PTR_W-1:0]   rd_pointer; // Read pointer
logic [PTR_W-1:0]   rd_pointer_next;

    //     CLK    RST    EN          DOUT        DIN              DEF
`RST_EN_FF(clock, reset, wr_en_fifo, wr_pointer, wr_pointer_next, '0)
`RST_EN_FF(clock, reset, rd_en_fifo, rd_pointer, rd_pointer_next, '0)

    // CLK    EN          DOUT                DIN
`EN_FF(clock, wr_en_fifo, memory[wr_pointer], wdata)

 // Memory is valid if there is one than more level, so it is not empty
logic  mem_valid;
assign mem_valid = !(|level[1:0]);

always_comb
begin
        // Enable storing data in memory if not full and (no pop at this cycle OR XXX)
    wr_en_fifo = push  & !full & (mem_valid | (not_empty && !pop));

        // Enable reading data from memory if there is a POP and there is
        // a valid element on the FIFO
    rd_en_fifo = (pop & mem_valid);

        // Update write and read pointers
    wr_pointer_next = (wr_pointer == (MEM_DEPTH-1)) ? {PTR_W {1'b0}} : wr_pointer + 1'b1;
    rd_pointer_next = (rd_pointer == (MEM_DEPTH-1)) ? {PTR_W {1'b0}} : rd_pointer + 1'b1;
end

/////////////////////////////
// Control when to return data from storage
        
logic             rdata_en;
logic [WIDTH-1:0] rdata_next;

// Our logic works such that rdata always has the value of the first entry
    // CLK    EN        DOUT       DIN
`EN_FF(clock, rdata_en, rdata, rdata_next)

always_comb 
begin 
        // Update rdata if:
            // Pushing first entry
            // 2 levels and receive pop and push at the same time
            // Received pop and have more than two levels
    rdata_en =  ((push && !not_empty) | (push && pop && !mem_valid)) | ( rd_en_fifo );

    if ((push && !not_empty) || (push && pop && !mem_valid)) 
        rdata_next = wdata;

    else if (rd_en_fifo) // Pop received
        rdata_next = memory[rd_pointer];

    else // Maintain the value
        rdata_next = rdata;
end    
endmodule


