`include "soc.vh"

module cache_top
(
    // System signals
    input   logic                               clock,
    input   logic                               reset,
    input   priv_mode_t [`THR_PER_CORE-1:0]     priv_mode,
    input   multithreading_mode_t               mt_mode,

    // Send stall pipeline request
    output  logic [`THR_PER_CORE-1:0]           dcache_ready,
    input   logic [`THR_PER_CORE-1:0]           flush_cache, 

    // Request from the ALU stage
    input   logic                               req_valid,
    input   dcache_request_t                    req_info,
    input   logic [`THR_PER_CORE_WIDTH-1:0]     req_thread_id,

    // Request to WB stage
    output  logic                               req_wb_valid,
    output  writeback_request_t                 req_wb_info,
    output  logic [`THR_PER_CORE_WIDTH-1:0]     req_wb_thread_id,

    // Request to the memory hierarchy
    output  logic                               req_valid_miss,
    output  memory_request_t                    req_info_miss,

    // Response from the memory hierarchy
    input   logic [`DCACHE_LINE_WIDTH-1:0]      rsp_data_miss,
    input   logic [`THR_PER_CORE_WIDTH-1:0]     rsp_thread_id,
    input   logic                               rsp_bus_error,
    input   logic                               rsp_valid_miss,

    // Request from WB stage to add a new entry
    input   logic                               new_tlb_entry,
    input   logic [`THR_PER_CORE_WIDTH-1:0]     new_tlb_thread_id,
    input   tlb_req_info_t                      new_tlb_info
);
logic [`THR_PER_CORE_WIDTH-1:0] thread_id;
logic [`THR_PER_CORE_WIDTH-1:0] previous_thread;

assign thread_id = req_thread_id;

 // CLK    DOUT             DIN            
`FF(clock, previous_thread, thread_id)

//////////////////////////////////////////////////
// Store req info for misses
logic [`THR_PER_CORE-1:0]                       req_valid_ff;
dcache_request_t [`THR_PER_CORE-1:0]            req_info_ff;

logic instr_xcpt;
genvar u;
generate for (u=0; u < `THR_PER_CORE; u++) 
begin
    logic update_ff;
    assign update_ff = req_valid & !instr_xcpt & (thread_id == u);

        //     CLK   RST                                         EN        DOUT             DIN        DEF
    `RST_EN_FF(clock, reset | flush_cache[u] | dcache_rsp_valid, update_ff, req_valid_ff[u], req_valid, 1'b0)

        // CLK    EN         DOUT            DIN
    `EN_FF(clock, update_ff, req_info_ff[u], req_info)
end
endgenerate

//////////////////////////////////////////////////
// Exceptions
cache_xcpt_t  xcpt_cache;
logic xcpt_bus_error;
logic xcpt_dtlb_miss;

logic is_store;
always_comb
begin
    is_store = (req_valid              ) ? req_info.is_store : 
               (req_valid_ff[thread_id]) ? req_info_ff[thread_id].is_store :
                                           1'b0;

    xcpt_cache.xcpt_bus_error  = xcpt_bus_error;
    xcpt_cache.xcpt_dtlb_miss  =   xcpt_dtlb_miss 
                                 | (!dTlb_write_privilege & dTlb_rsp_valid & is_store); 
    xcpt_cache.xcpt_addr_fault = 1'b0;
    xcpt_cache.xcpt_addr_val   = (req_valid) ? req_info.addr : req_info_ff[thread_id].addr;
    xcpt_cache.xcpt_pc         = (req_valid) ? req_info.pc   : req_info_ff[thread_id].pc;
end

//////////////////////////////////////////////////
// Request to WB

// Signals from D$
logic [`DCACHE_MAX_ACC_SIZE-1:0]    rsp_data_dcache;
logic                               dcache_rsp_valid;
logic                               dcache_rsp_error;

// Signals for WB request
logic                               req_wb_valid_next;
logic                               req_wb_valid_ff;

writeback_request_t                 req_wb_info_next;
writeback_request_t                 req_wb_info_ff;

//      CLK    RST    DOUT             DIN                DEF
`RST_FF(clock, reset, req_wb_valid_ff, req_wb_valid_next, '0)
`RST_FF(clock, reset, req_wb_info_ff,  req_wb_info_next,  '0)

assign req_wb_valid     = (flush_cache[previous_thread]) ? 1'b0 : req_wb_valid_ff;
assign req_wb_info      = req_wb_info_ff;
assign req_wb_thread_id = previous_thread;

always_comb
begin
    req_wb_info_next.instr_id       = (req_valid) ? req_info.instr_id : req_info_ff[thread_id].instr_id;
    req_wb_info_next.pc             = (req_valid) ? req_info.pc : req_info_ff[thread_id].pc;
                                 
    req_wb_info_next.tlbwrite       = 1'b0;  
    req_wb_info_next.tlb_id         = '0;
    req_wb_info_next.tlb_req_info   = '0;
                                 
    req_wb_info_next.rf_wen         = (req_valid) ? (dcache_rsp_error) ? 1'b1 : 
                                                    !req_info.is_store : !req_info_ff[thread_id].is_store;

    req_wb_info_next.rf_dest        = (req_valid) ? (dcache_rsp_error) ? `COND_ERR_REG_ADDR : 
                                                    req_info.rd_addr   : req_info_ff[thread_id].rd_addr;

    req_wb_info_next.rf_data        = rsp_data_dcache ;

    req_wb_info_next.xcpt_fetch     = (req_valid) ? req_info.xcpt_fetch  : req_info_ff[thread_id].xcpt_fetch;
    req_wb_info_next.xcpt_decode    = (req_valid) ? req_info.xcpt_decode : req_info_ff[thread_id].xcpt_decode;
    req_wb_info_next.xcpt_alu       = (req_valid) ? req_info.xcpt_alu    : req_info_ff[thread_id].xcpt_alu;
    req_wb_info_next.xcpt_mul       = '0;
    req_wb_info_next.xcpt_cache     = xcpt_cache; 
end

// In case of LD request we will have to write that data on the RF.
// In addition, we also check if the request is for an ALU R type 
// instruction, which also writes on the RF.
assign instr_xcpt =   req_info.xcpt_fetch.xcpt_itlb_miss
                    | req_info.xcpt_fetch.xcpt_bus_error
                    | req_info.xcpt_decode.xcpt_illegal_instr
                    | req_info.xcpt_alu.xcpt_overflow
                    | xcpt_cache.xcpt_bus_error   
                    | xcpt_cache.xcpt_dtlb_miss   
                    | xcpt_cache.xcpt_addr_fault ;

assign req_wb_valid_next =  ( flush_cache[thread_id]) ? 1'b0 :
                            ( req_valid & instr_xcpt) ? 1'b1:
                                                        dcache_rsp_valid;


/////////////////////////////////////////                                                
// Data TLB signals

// Request to dTLB
logic dtlb_req_valid;
assign dtlb_req_valid = (flush_cache[thread_id]) ? 1'b0 :
                                                     dcache_ready[thread_id] 
                                                   & (req_valid | req_valid_ff[thread_id]);


// Response from dTLB
logic                   dTlb_rsp_valid;
logic [`PHY_ADDR_RANGE] dTlb_rsp_phy_addr;
logic                   dTlb_write_privilege;


//////////////////////////////////////////////////
// Request to the Data Cache
logic               dcache_req_valid;
dcache_request_t    req_dcache_info;

assign dcache_req_valid = (flush_cache[thread_id]) ? 1'b0 :
                                                    dTlb_rsp_valid & !xcpt_dtlb_miss;

always_comb
begin
    req_dcache_info      = (req_valid) ? req_info : req_info_ff[thread_id];
    req_dcache_info.addr = `ZX(`VIRT_ADDR_WIDTH,dTlb_rsp_phy_addr);
end
                                      
//////////////////////////////////////////////////
// Data Cache instance
data_cache_mt
dcache
(
    // System signals
    .clock              ( clock             ),
    .reset              ( reset             ),
    .dcache_ready       ( dcache_ready      ),
    .mt_mode            ( mt_mode           ),
    .active_thread_id   ( thread_id         ),

    // Exception
    .xcpt_bus_error     ( xcpt_bus_error    ),
    
    // Request from the core pipeline
    .req_valid          ( dcache_req_valid  ),
    .req_info           ( req_dcache_info   ),

    // Response to the core pipeline
    .rsp_valid          ( dcache_rsp_valid  ),
    .rsp_error          ( dcache_rsp_error  ),
    .rsp_data           ( rsp_data_dcache   ),
    
    // Request to the memory hierarchy
    .req_info_miss      ( req_info_miss     ),
    .req_valid_miss     ( req_valid_miss    ),
                    
    // Response from the memory hierarchy
    .rsp_data_miss      ( rsp_data_miss     ),
    .rsp_thread_id      ( rsp_thread_id     ),
    .rsp_bus_error      ( rsp_bus_error     ),
    .rsp_valid_miss     ( rsp_valid_miss    )
);

//////////////////////////////////////////////////
// Data TLB instance
logic [`VIRT_ADDR_RANGE] req_info_addr;
assign req_info_addr = (req_valid) ? req_info.addr :
                                     req_info_ff[thread_id].addr;

tlb_cache
dtlb
(
    // System signals
    .clock              ( clock                 ),
    .reset              ( reset                 ),
    .mt_mode            ( mt_mode               ),

    // Request from the core pipeline
    .req_valid          ( dtlb_req_valid        ),
    .req_virt_addr      ( req_info_addr         ),
    .req_thread_id      ( thread_id             ),
    .priv_mode          ( priv_mode[thread_id]  ),

    // Response to the cache
    .rsp_valid          ( dTlb_rsp_valid        ), 
    .tlb_miss           ( xcpt_dtlb_miss        ), 
    .rsp_phy_addr       ( dTlb_rsp_phy_addr     ), 
    .writePriv          ( dTlb_write_privilege  ), 
    
    // Write request from the O.S
    .new_tlb_entry      ( new_tlb_entry         ),
    .new_tlb_thread_id  ( new_tlb_thread_id     ),
    .new_tlb_info       ( new_tlb_info          )
);

//////////////////////////////////////////////////
// VERBOSE

`ifdef VERBOSE_DECODE
always_ff @(posedge clock)
begin
    if (req_wb_valid)
    begin
    end
end
`endif

endmodule

