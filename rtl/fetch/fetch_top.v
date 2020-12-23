`include "soc.vh"

module fetch_top
(
    // System signals
    input   logic                               clock,
    input   logic                               reset,

    input   logic   [`PC_WIDTH-1:0]             boot_addr,
    input   priv_mode_t [`THR_PER_CORE-1:0]     priv_mode,
    input   multithreading_mode_t               mt_mode,
    
    // Exception
    output  fetch_xcpt_t                        xcpt_fetch,

    // Branches
    input   logic   [`THR_PER_CORE-1:0]                 take_branch,
    input   logic   [`THR_PER_CORE-1:0][`PC_WIDTH-1:0]  branch_pc,

    // Stall pipeline
    input   logic   [`THR_PER_CORE-1:0]         stall_fetch,

    // Fetched instruction
    output  logic   [`INSTR_WIDTH-1:0]          decode_instr_data,
    output  logic                               decode_instr_valid,
    output  logic   [`PC_WIDTH_RANGE]           decode_instr_pc, 
    output  logic   [`THR_PER_CORE_WIDTH-1:0]   decode_thread_id,
    
    // Request to the memory hierarchy
    output  logic                               req_valid_miss,
    output  memory_request_t                    req_info_miss,

    // Response from the memory hierarchy
    input   logic                               rsp_valid_miss,
    input   logic [`THR_PER_CORE_WIDTH-1:0]     rsp_thread_id,
    input   logic [`ICACHE_LINE_WIDTH-1:0]      rsp_data_miss,
    input   logic                               rsp_bus_error,

    // New entry for iTLB
    input   logic                               new_tlb_entry,
    input   logic [`THR_PER_CORE_WIDTH-1:0]     new_tlb_thread_id,
    input   tlb_req_info_t                      new_tlb_info
 );
 
/////////////////////////////////////////
//Signals that must be declared at the top
logic   [`THR_PER_CORE_WIDTH-1:0]   active_thread;
logic   [`THR_PER_CORE-1:0]         icache_ready;

// Response from the Instruction Cache
logic                                icache_rsp_valid;
logic   [`ICACHE_LINE_WIDTH-1:0]     icache_rsp_data;
logic   [`ICACHE_INSTR_IN_LINE_WIDTH-1:0]               word_in_line;
logic   [`INSTR_WIDTH-1:0]                              decode_instr_data_next;

// Response from iTLB
logic                     iTlb_rsp_valid;
logic   [`PHY_ADDR_RANGE] iTlb_rsp_phy_addr;

/////////////////////////////////////////
// Branches
logic   [`THR_PER_CORE-1:0]                take_branch_ff;
logic   [`THR_PER_CORE-1:0]                take_branch_update;
logic   [`THR_PER_CORE-1:0]                branch_executed;
logic   [`THR_PER_CORE-1:0][`PC_WIDTH-1:0] branch_pc_ff;

genvar ll;
generate for (ll=0; ll < `THR_PER_CORE; ll++) 
begin 
    logic thread_is_active;
    assign thread_is_active = (active_thread == ll);

        //     CLK    RST    EN                      DOUT                DIN              DEF
    `RST_EN_FF(clock, reset, take_branch_update[ll], take_branch_ff[ll], take_branch[ll], 1'b0)
    
        // CLK    EN                      DOUT              DIN
    `EN_FF(clock, take_branch_update[ll], branch_pc_ff[ll], branch_pc[ll])
    
    assign branch_executed[ll] =  thread_is_active 
                                & icache_ready[ll]
                                &(take_branch[ll] | take_branch_ff[ll]);
    
    assign take_branch_update[ll] = (!take_branch_ff[ll] & (take_branch[ll])) ? 1'b1 : // branch request received
                                    (branch_executed[ll])                     ? 1'b1 : // new PC has been requested and updated
                                                                                1'b0 ;
end
endgenerate

/////////////////////////////////////////
// Thread management
logic   [`THR_PER_CORE_WIDTH-1:0] active_thread_next;
logic   [`THR_PER_CORE_WIDTH-1:0] active_thread_ff;
logic   [`THR_PER_CORE_WIDTH-1:0] next_ready_thread;

    //  CLK    RST    DOUT              DIN                DEF
`RST_FF(clock, reset, active_thread_ff, active_thread_next, '0)

assign active_thread_next = (active_thread_ff == `THR_PER_CORE - 1'b1) ? '0 : // control overflow 
                                                                        active_thread_ff + 1'b1;

arb_rr
#(.NUM_REQS (`THR_PER_CORE))
arb_rr_next_thread 
(
    .clock      ( clock             ),
    .reset      ( reset             ),
    .grants     ( next_ready_thread ),
    .pop        ( 1'b1              ), // we look for a new thread each cycle
    .reqs       ( icache_ready      )  
);

// HOW TO SELECT THE ACTIVE THREAD
//------------------------------------
// We always try to schedule active_thread + 1. If the thread to be schedulled
// is waiting for a memory response, then we check if the response has arrived
// the same cycle so it can be scheduled. Othwerise, we look for the next
// thread that is ready to go using a RR approach. If no threads are ready,
// then no request is sent to Decode stage.
assign active_thread = (mt_mode  == Single_Threaded) ? '0 : 
                       (icache_ready[active_thread_ff]) ? active_thread_ff : //Thread has no pendent memory requests
                       (rsp_valid_miss & (rsp_thread_id == active_thread_ff)) ? active_thread_ff : //Thread received response from MM
                       (|icache_ready) ? next_ready_thread : //Schedule next thread that is ready to go
                       active_thread_ff; // if no thread is ready, keep the same thrID and use inject a bubble                       
      
/////////////////////////////////////////
// Program counter
logic   [`THR_PER_CORE-1:0][`PC_WIDTH-1:0] program_counter;
logic   [`THR_PER_CORE-1:0][`PC_WIDTH-1:0] program_counter_ff;
logic   [`THR_PER_CORE-1:0][`PC_WIDTH-1:0] program_counter_next;
logic   [`THR_PER_CORE-1:0]                program_counter_update;


genvar thr_id;
generate for (thr_id = 0; thr_id < `THR_PER_CORE; thr_id++) 
begin
    logic thread_is_active;
    assign thread_is_active = (active_thread == thr_id);

    assign program_counter[thr_id] = program_counter_ff[thr_id]; 
    //assign program_counter[thr_id] = (  mt_mode  == Single_Threaded 
    //                                  & stall_fetch_ff[thr_id]      ) ? program_counter_ff[thr_id] - 4  : 
    //                                                                    program_counter_ff[thr_id];

        //     CLK    RST    EN                              DOUT                        DIN                           DEF
    `RST_EN_FF(clock, reset, program_counter_update[thr_id], program_counter_ff[thr_id], program_counter_next[thr_id], boot_addr)
    
    assign program_counter_update[thr_id]   = (  reset
                                               | !thread_is_active  
                                               | stall_fetch[thr_id] 
                                               | !icache_ready[thr_id]) ? 1'b0 : 1'b1; 
    
    assign program_counter_next[thr_id]     = (  thread_is_active 
                                               & take_branch[thr_id] 
                                               & icache_ready[thr_id] ) ? branch_pc[thr_id] : 
                                              (  thread_is_active
                                               & take_branch_ff[thr_id]  
                                               & icache_ready[thr_id] ) ? branch_pc_ff[thr_id] :
                                              (req_valid_miss)          ? program_counter_ff[thr_id] :
                                                                          program_counter_ff[thr_id] + 4;
end
endgenerate

/////////////////////////////////////////
// Exceptions

fetch_xcpt_t   xcpt_fetch_next;

 // CLK    DOUT        DIN                   
`FF(clock, xcpt_fetch, xcpt_fetch_next)

logic   xcpt_bus_error_aux;
logic   xcpt_itlb_miss;

always_comb
begin
    xcpt_fetch_next.xcpt_itlb_miss   = xcpt_itlb_miss;
    xcpt_fetch_next.xcpt_bus_error   = xcpt_bus_error_aux;
    xcpt_fetch_next.xcpt_addr_val    = program_counter[active_thread];
    xcpt_fetch_next.xcpt_pc          = program_counter[active_thread];
end

/////////////////////////////////////////                                                
// Request to the Instruction TLB
logic                        itlb_req_valid;
logic   [`THR_PER_CORE-1:0]  stall_fetch_ff;

genvar ii;
generate for (ii=0; ii < `THR_PER_CORE; ii++) 
begin
    logic thread_is_active;
    assign thread_is_active = (active_thread == ii);

    //     CLK    EN                DOUT                DIN
    `EN_FF(clock, thread_is_active, stall_fetch_ff[ii], stall_fetch[ii])
end
endgenerate

assign itlb_req_valid = (reset)                         ? 1'b0 :
                        (stall_fetch[active_thread])    ? 1'b0 :
                        (stall_fetch_ff[active_thread]) ? 1'b1 : // !stall_fetch & stall_fetch_ff
                                                          program_counter_update[active_thread];                                                    

/////////////////////////////////////////                                                
// Request to the Instruction Cache
logic icache_req_valid;

assign icache_req_valid= (stall_fetch[active_thread]) ? 1'b0 :
                                                        iTlb_rsp_valid & !xcpt_itlb_miss;

/////////////////////////////////////////
// Fetch to Decode

// In case of stall we mantain the value of the instr to be decoded because
// decode stage may need it to relaunch the instruction
logic                               decode_instr_valid_ff;
logic                               decode_instr_valid_next;
logic   [`INSTR_WIDTH-1:0]          decode_instr_data_ff;
logic   [`PC_WIDTH-1:0]             decode_instr_pc_ff;
logic   [`THR_PER_CORE_WIDTH-1:0]   decode_thread_id_ff;

assign decode_instr_valid_next = (  (take_branch[active_thread]) 
                                  | take_branch_ff[active_thread]) ? 1'b0:
                                 (  xcpt_itlb_miss
                                  | xcpt_bus_error_aux           ) ? 1'b1:
                                                                    icache_rsp_valid;

//      CLK    RST    DOUT                   DIN                      DEF
`RST_FF(clock, reset, decode_instr_valid_ff, decode_instr_valid_next, 1'b0)


//  CLK    DOUT                   DIN                   
`FF(clock, decode_instr_data_ff,  decode_instr_data_next)
`FF(clock, decode_instr_pc_ff,    program_counter[active_thread])
`FF(clock, decode_thread_id_ff,   active_thread)

assign decode_instr_valid = decode_instr_valid_ff;
assign decode_instr_data  = decode_instr_data_ff; 
assign decode_instr_pc    = decode_instr_pc_ff;
assign decode_thread_id   = decode_thread_id_ff;

always_comb
begin
    word_in_line            = program_counter[active_thread][`ICACHE_INSTR_IN_LINE];
    decode_instr_data_next  = icache_rsp_data[`INSTR_WIDTH*word_in_line+:`INSTR_WIDTH];
end

/////////////////////////////////////////
// Instruction Cache instance

instruction_cache_mt
icache(
    // System signals
    .clock              ( clock                 ),
    .reset              ( reset                 ),
    .icache_ready       ( icache_ready          ),
    .xcpt_bus_error     ( xcpt_bus_error_aux    ),
    .mt_mode            ( mt_mode               ),
    
    // Request from the core pipeline
    .req_valid          ( icache_req_valid      ),
    .req_addr           ( iTlb_rsp_phy_addr     ),
    .req_thread_id      ( active_thread         ),

    // Response to the core pipeline
    .rsp_valid          ( icache_rsp_valid      ),
    .rsp_data           ( icache_rsp_data       ),
    
    // Request to the memory hierarchy
    .req_info_miss      ( req_info_miss         ),
    .req_valid_miss     ( req_valid_miss        ),
                    
    // Response from the memory hierarchy
    .rsp_data_miss      ( rsp_data_miss         ),
    .rsp_bus_error      ( rsp_bus_error         ),
    .rsp_thread_id      ( rsp_thread_id         ),
    .rsp_valid_miss     ( rsp_valid_miss        )
);


logic  tlb_wr_priv;

tlb_cache
itlb
(
    // System signals
    .clock              ( clock                         ),
    .reset              ( reset                         ),
    .mt_mode            ( mt_mode                       ),

    // Request from the core pipeline
    .req_valid          ( itlb_req_valid                ),
    .req_virt_addr      ( program_counter[active_thread]),
    .req_thread_id      ( active_thread                 ),
    .priv_mode          ( priv_mode[active_thread]      ),

    // Response to the cache
    .rsp_valid          ( iTlb_rsp_valid                ), 
    .tlb_miss           ( xcpt_itlb_miss                ), 
    .rsp_phy_addr       ( iTlb_rsp_phy_addr             ), 
    .writePriv          ( tlb_wr_priv                   ), //unused for Icache
    
    // Write request from the O.S
    .new_tlb_entry      ( new_tlb_entry                 ),
    .new_tlb_thread_id  ( new_tlb_thread_id             ),
    .new_tlb_info       ( new_tlb_info                  )
);



/////////////////////////////////////////
// Verbose
`ifdef VERBOSE_FETCH
always_ff @(posedge clock)
begin
    /*
    if (program_counter_update)
    begin
        $display("[FETCH] Program counter value is %h",program_counter);
        $display("[FETCH] Program counter next value is %h",program_counter_next);
    end
    */
    if (decode_instr_valid)
    begin
        $display("[FETCH] Request to decode. PC is %h",decode_instr_pc);
        $display("        Data to be decoded = %h",decode_instr_data);
    end
end
`endif

endmodule

