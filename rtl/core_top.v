`include "soc.vh"

module core_top
(
    // System signals
    input   logic                                   clock,
    input   logic                                   reset,
 
    // Boot address
    input   logic   [`PC_WIDTH-1:0]                 boot_addr,

    // Request from D$ to the memory hierarchy
    output  logic                                   dcache_req_valid_miss,
    output  memory_request_t                        dcache_req_info_miss,

    // Request from I$ to the memory hierarchy
    output  logic                                   icache_req_valid_miss,
    output  memory_request_t                        icache_req_info_miss,

    // Response from the memory hierarchy
    input   logic                                   rsp_valid_miss,
    input   logic [`THR_PER_CORE_WIDTH-1:0]         rsp_thread_id,
    input   logic [`DCACHE_LINE_WIDTH-1:0]          rsp_data_miss,
    input   logic                                   rsp_bus_error,
    input   logic                                   rsp_cache_id // 0 for I$, 1 for D$
);


// Control signals needed by the stages to synchronize between them

/////////////////////////////////////////
// Fetch signals to other stages

// Instruction from fetch to decode phase
logic                               fetch_instr_valid;
logic   [`INSTR_WIDTH-1:0]          fetch_instr_data;
logic   [`PC_WIDTH_RANGE]           fetch_instr_pc;
logic   [`THR_PER_CORE_WIDTH-1:0]   fetch_thread_id;

// Exception signals
fetch_xcpt_t                xcpt_fetch_to_decode;
/////////////////////////////////////////

/////////////////////////////////////////
// Decode signals to other stages

// Request to ALU
logic                               req_to_alu_valid;
alu_request_t                       req_to_alu_info;
logic   [`ROB_ID_RANGE]             req_to_alu_instr_id;
logic   [`THR_PER_CORE_WIDTH-1:0]   req_to_alu_thread_id;
logic   [`PC_WIDTH_RANGE]           req_to_alu_pc;
fetch_xcpt_t                        xcpt_fetch_to_alu;
decode_xcpt_t                       xcpt_decode_to_alu;

// Request to MUL
logic                               req_to_mul_valid;
logic   [`THR_PER_CORE_WIDTH-1:0]   req_to_mul_thread_id;
mul_request_t                       req_to_mul_info;
logic   [`ROB_ID_RANGE]             req_to_mul_instr_id;
logic   [`PC_WIDTH_RANGE]           req_to_mul_pc;
fetch_xcpt_t                        xcpt_fetch_to_mul;
decode_xcpt_t                       xcpt_decode_to_mul;

// Privilege mode
priv_mode_t [`THR_PER_CORE-1:0]  priv_mode;
multithreading_mode_t            mt_mode;

/////////////////////////////////////////

/////////////////////////////////////////
// MUL signals to other stages

// Stall the pipeline
logic [`THR_PER_CORE-1:0] mul_stall_pipeline;

// Request to WB
logic                           mul_req_to_wb_valid;
writeback_request_t             mul_req_to_wb_info;
logic [`THR_PER_CORE_WIDTH-1:0] mul_req_to_wb_thread_id;

// Signals for bypass with Reorder buffer
logic [`THR_PER_CORE_WIDTH-1:0] rob_mul_thread_id;
logic [`ROB_ID_RANGE]           rob_mul_src1_id;
logic [`ROB_ID_RANGE]           rob_mul_src2_id;
logic                           rob_mul_src1_hit;
logic                           rob_mul_src2_hit;
logic [`REG_FILE_DATA_RANGE]    rob_mul_src1_data;
logic [`REG_FILE_DATA_RANGE]    rob_mul_src2_data;

/////////////////////////////////////////
// ALU signals to other stages

logic [`THR_PER_CORE-1:0] alu_stall_pipeline;

// Request to WB stage
logic                           alu_req_wb_valid;
writeback_request_t             alu_req_wb_info;
logic [`THR_PER_CORE_WIDTH-1:0] alu_req_wb_thread_id;
logic                           alu_req_wb_mem_blocked;
dcache_request_t                alu_req_wb_dcache_info;

// Request to cache stage
dcache_request_t                alu_req_to_dcache_info;
logic                           alu_req_to_dcache_valid;
logic [`THR_PER_CORE_WIDTH-1:0] alu_req_to_dcache_thread_id;

// Branch signals
logic                           alu_take_branch; // from ALU to fetch  
logic [`THR_PER_CORE_WIDTH-1:0] alu_branch_thr_id;
logic [`THR_PER_CORE-1:0]       alu_iret_instr; // from ALU to RF  
logic [`PC_WIDTH-1:0]           alu_branch_pc;

// Signals to RoB
logic                           alu_cache_stage_free;

// Signals for bypass with Reorder buffer
logic [`THR_PER_CORE_WIDTH-1:0] rob_alu_thread_id;
logic [`ROB_ID_RANGE]           rob_alu_src1_id;
logic [`ROB_ID_RANGE]           rob_alu_src2_id;
logic                           rob_alu_src1_hit;
logic                           rob_alu_src2_hit;
logic [`REG_FILE_DATA_RANGE]    rob_alu_src1_data;
logic [`REG_FILE_DATA_RANGE]    rob_alu_src2_data;
/////////////////////////////////////////

/////////////////////////////////////////
// Cache signals to other stages

logic [`THR_PER_CORE-1:0] dcache_ready;

// Request to WB
logic                            cache_req_to_wb_valid;
writeback_request_t              cache_req_to_wb_info;
logic [`THR_PER_CORE_WIDTH-1:0]  cache_req_to_wb_thread_id;

/////////////////////////////////////////

/////////////////////////////////////////
// WriteBack signals to other stages
logic  [`THR_PER_CORE-1:0]  wb_flush_pipeline;
logic  [`THR_PER_CORE-1:0]  wb_flush_cache;

// Request to Cache
dcache_request_t                    wb_req_to_dcache_info;
logic                               wb_req_to_dcache_valid;
logic [`THR_PER_CORE_WIDTH-1:0]     wb_req_to_dcache_thread_id;

// Request to RF
logic [`REG_FILE_DATA_RANGE]        wb_writeValRF;
logic 				                wb_writeEnRF;
logic [`REG_FILE_ADDR_RANGE]    	wb_destRF;
logic [`ROB_ID_RANGE]               wb_write_rob_idRF;
logic [`THR_PER_CORE_WIDTH-1:0]     wb_write_thread_idRF;

// xcpt data to RF / F
logic                               wb_xcpt_valid_o;
xcpt_type_t                         wb_xcpt_type;
logic [`PC_WIDTH_RANGE] 		    wb_rmPC;
logic [`REG_FILE_XCPT_ADDR_RANGE] 	wb_rmAddr;
logic [`THR_PER_CORE_WIDTH-1:0]     wb_xcpt_thread_id;

// Request to TLB
logic                               wb_new_tlb_entry;
logic                               wb_new_tlb_id; // 0 for iTLB; 1 for dTLB
tlb_req_info_t                      wb_new_tlb_info;
logic [`THR_PER_CORE_WIDTH-1:0]     wb_new_tlb_thread_id;

//RoB
logic [`THR_PER_CORE-1:0]        reorder_buffer_full;
logic [`ROB_NUM_ENTRIES_W_RANGE] rob_tail;

/////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Instruction Fetch. It fetches one instruction per cycle and sends it to the
// instruction decoder.
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
logic [`THR_PER_CORE-1:0]                   take_branch;
logic [`THR_PER_CORE-1:0][`PC_WIDTH-1:0]    branch_pc;
logic [`THR_PER_CORE-1:0]                   wb_xcpt_valid;

always_comb
begin
    for a
    begin
        wb_xcpt_valid[i] = (wb_xcpt_valid_o && i == wb_xcpt_thread_id);
        take_branch[i]   = (alu_take_branch && i == alu_branch_thr_id) | wb_xcpt_valid[i];
        branch_pc[i]     = (wb_xcpt_valid[i]) ? `CORE_XCPT_ADDRESS : alu_branch_pc;
    end
end

fetch_top
fetch_top
(
    // System signals
    .clock              ( clock                 ),
    .reset              ( reset                 ),

    .boot_addr          ( boot_addr             ),
    .priv_mode          ( priv_mode             ),
    .mt_mode            ( mt_mode               ),

    // Exception
    .xcpt_fetch         ( xcpt_fetch_to_decode  ),

    // Branches
    .take_branch        ( take_branch           ),
    .branch_pc          ( branch_pc             ),

    // Stop fetching instructions
    .stall_fetch        (  alu_stall_pipeline  
                         | mul_stall_pipeline
                         | reorder_buffer_full  
                         | wb_flush_pipeline    ),

    // Fetched instruction
    .decode_instr_data  ( fetch_instr_data      ),
    .decode_instr_valid ( fetch_instr_valid     ),
    .decode_instr_pc    ( fetch_instr_pc        ),
    .decode_thread_id   ( fetch_thread_id       ),
    
    // Request to the memory hierarchy
    .req_valid_miss     ( icache_req_valid_miss ),
    .req_info_miss      ( icache_req_info_miss  ),

    // Response from the memory hierarchy   
    .rsp_valid_miss     ( !rsp_cache_id & 
                           rsp_valid_miss       ),
    .rsp_thread_id      ( rsp_thread_id         ),
    .rsp_data_miss      ( rsp_data_miss         ),
    .rsp_bus_error      ( !rsp_cache_id &
                          rsp_bus_error         ),

    // New TLB entry
    .new_tlb_entry      (  wb_new_tlb_entry 
                         & !wb_new_tlb_id       ),
    .new_tlb_thread_id  ( wb_new_tlb_thread_id  ),
    .new_tlb_info       ( wb_new_tlb_info       )
);                

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Instruction Decoder. It decodes one instruction per cycle and sends it to the
// execution stage (alu).
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

decode_top
decode_top
(
    // System signals
    .clock              ( clock                 ),
    .reset              ( reset                 ),

    .priv_mode          ( priv_mode             ),
    .mt_mode            ( mt_mode               ),
    .iret_instr         ( alu_iret_instr        ),

    // Stall pipeline
    .stall_decode       (  alu_stall_pipeline
                         | mul_stall_pipeline
                         | reorder_buffer_full  ),

    .flush_decode       (  take_branch 
                         | wb_xcpt_valid       
                         | wb_flush_pipeline    ),

    .flush_rob          ( wb_xcpt_valid         ),

    // Exceptions from fetch
    .xcpt_fetch_in      ( xcpt_fetch_to_decode  ), 

    // Fetched instruction
    .fetch_instr_valid  ( fetch_instr_valid     ),
    .fetch_instr_data   ( fetch_instr_data      ),
    .fetch_instr_pc     ( fetch_instr_pc        ), 
    .fetch_thread_id    ( fetch_thread_id       ),

    // Instruction to ALU
    .req_to_alu_valid     ( req_to_alu_valid    ),
    .req_to_alu_thread_id (req_to_alu_thread_id ), 
    .req_to_alu_info      ( req_to_alu_info     ), 
    .req_to_alu_instr_id  ( req_to_alu_instr_id ),
    .req_to_alu_pc        ( req_to_alu_pc       ), 
    .alu_xcpt_fetch_out   ( xcpt_fetch_to_alu   ),
    .alu_decode_xcpt      ( xcpt_decode_to_alu  ),

    // Instruction to MUL
    .req_to_mul_valid   ( req_to_mul_valid      ),
    .req_to_mul_thread_id (req_to_mul_thread_id ), 
    .req_to_mul_info    ( req_to_mul_info       ),
    .req_to_mul_instr_id( req_to_mul_instr_id   ),
    .req_to_mul_pc      ( req_to_mul_pc         ),
    .mul_xcpt_fetch_out ( xcpt_fetch_to_mul     ),
    .mul_decode_xcpt    ( xcpt_decode_to_mul    ),

    // Write requests to the Register File from WB stage 
    .writeValRF         ( wb_writeValRF         ), 
    .writeEnRF          ( wb_writeEnRF          ), 
    .destRF             ( wb_destRF             ), 
    .write_rob_idRF     ( wb_write_rob_idRF     ),
    .write_thread_idRF  ( wb_write_thread_idRF  ),

    // Exceptions values to be stored on the RF
    .xcpt_valid         ( wb_xcpt_valid         ),
    .rmPC               ( wb_rmPC               ),
    .rmAddr             ( wb_rmAddr             ),
    .xcpt_type          ( wb_xcpt_type          ),
    .xcpt_thread_id     ( wb_xcpt_thread_id     )
);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Multiplication Unit (ALU). Performs multiplication operation taking into
// account the fixed latency for this operation and sends the result to the WB, 
// decode or cache stage depending on the instruction type
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
mul_top
mul_top
(
    // System signals
    .clock              ( clock                 ),
    .reset              ( reset                 ),

    // Stall pipeline
    .flush_mul          (  wb_xcpt_valid         
                         | wb_flush_pipeline    ),
    .stall_decode       ( mul_stall_pipeline    ),
    
    // Request from decode stage
        // Operation
    .req_mul_valid      ( req_to_mul_valid      ),
    .req_mul_thread_id  ( req_to_mul_thread_id  ),
    .req_mul_info       ( req_to_mul_info       ),
    .req_mul_instr_id   ( req_to_mul_instr_id   ),
    .req_mul_pc         ( req_to_mul_pc         ), 
   
        // Exceptions
    .xcpt_fetch_in      ( xcpt_fetch_to_mul     ),
    .xcpt_decode_in     ( xcpt_decode_to_mul    ),

    // Request to WB stage 
    .req_wb_valid       ( mul_req_to_wb_valid   ),
    .req_wb_info        ( mul_req_to_wb_info    ),
    .req_wb_thread_id   ( mul_req_to_wb_thread_id),

    // Bypasses
        // Reorder buffer
    .rob_thread_id      ( rob_mul_thread_id     ),
    .rob_src1_id        ( rob_mul_src1_id       ),
    .rob_src2_id        ( rob_mul_src2_id       ),
    .rob_src1_hit       ( rob_mul_src1_hit      ),
    .rob_src2_hit       ( rob_mul_src2_hit      ),
    .rob_src1_data      ( rob_mul_src1_data     ),
    .rob_src2_data      ( rob_mul_src2_data     ),

    // Intercept writes to RF
    .writeEnRF          ( wb_writeEnRF          ), 
    .writeValRF         ( wb_writeValRF         ), 
    .write_rob_idRF     ( wb_write_rob_idRF     ),
    .write_thread_idRF  ( wb_write_thread_idRF  )
);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Arithmetical Logic Unit (ALU). Performs one instruction per cycle and sends 
// the result to the fetch, decode, cache or WB stage depending on the instruction
// type
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
alu_top
alu_top
(
    // System signals
    .clock              ( clock                     ),
    .reset              ( reset                     ),

    // Control signals with RoB
    .rob_tail           ( rob_tail                  ),
    .alu_no_req_to_cache( alu_cache_stage_free      ),

    // Stall pipeline
    .dcache_ready       ( dcache_ready              ),
    .flush_alu          (  wb_xcpt_valid             
                         | wb_flush_pipeline        ),
    .stall_decode       ( alu_stall_pipeline        ),

    // Exceptions
    .xcpt_fetch_in      ( xcpt_fetch_to_alu         ),
    .xcpt_decode_in     ( xcpt_decode_to_alu        ),

    // Request from decode stage
    .req_alu_valid      ( req_to_alu_valid          ),
    .req_alu_thread_id  ( req_to_alu_thread_id      ),
    .req_alu_info       ( req_to_alu_info           ),
    .req_alu_instr_id   ( req_to_alu_instr_id       ),
    .req_alu_pc         ( req_to_alu_pc             ),

    // Request to dcache stage 
    .req_dcache_valid     ( alu_req_to_dcache_valid    ),
    .req_dcache_info      ( alu_req_to_dcache_info     ),
    .req_dcache_thread_id ( alu_req_to_dcache_thread_id),
     
    // Request to WB stage
    .req_wb_valid       ( alu_req_wb_valid          ),
    .req_wb_thread_id   ( alu_req_wb_thread_id      ),
    .req_wb_info        ( alu_req_wb_info           ),
    .req_wb_mem_blocked ( alu_req_wb_mem_blocked    ),
    .req_wb_dcache_info ( alu_req_wb_dcache_info    ),
    
    // Branch signals to fetch stage
    .take_branch        ( alu_take_branch           ),
    .branch_thr_id      ( alu_branch_thr_id         ),
    .iret_instr         ( alu_iret_instr            ),
    .branch_pc          ( alu_branch_pc             ),

    // Bypasses
        // Reorder buffer
    .rob_thread_id      ( rob_alu_thread_id         ),
    .rob_src1_id        ( rob_alu_src1_id           ),
    .rob_src2_id        ( rob_alu_src2_id           ),
    .rob_src1_hit       ( rob_alu_src1_hit          ),
    .rob_src2_hit       ( rob_alu_src2_hit          ),
    .rob_src1_data      ( rob_alu_src1_data         ),
    .rob_src2_data      ( rob_alu_src2_data         ),

    // Intercept writes to RF
    .writeEnRF          ( wb_writeEnRF              ), 
    .writeValRF         ( wb_writeValRF             ), 
    .write_rob_idRF     ( wb_write_rob_idRF         ),
    .write_thread_idRF  ( wb_write_thread_idRF      )
);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Cache stage (ALU). Performs the LD and ST requests by accessing the data
// cache.  
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
dcache_request_t                req_to_dcache_info;
logic                           req_to_dcache_valid;
logic [`THR_PER_CORE_WIDTH-1:0] req_to_dcache_thread_id;

assign req_to_dcache_valid =   wb_req_to_dcache_valid 
                             | alu_req_to_dcache_valid;

assign req_to_dcache_info  = (  !wb_req_to_dcache_valid
                              & !alu_req_to_dcache_valid) ? req_to_dcache_info    :
                             ( wb_req_to_dcache_valid )   ? wb_req_to_dcache_info :
                                                            alu_req_to_dcache_info;   

assign req_to_dcache_thread_id = (  !wb_req_to_dcache_valid
                                  & !alu_req_to_dcache_valid) ? req_to_dcache_thread_id :
                                 ( wb_req_to_dcache_valid )   ? wb_req_to_dcache_thread_id :
                                                                alu_req_to_dcache_thread_id;   
cache_top
cache_top
(
    // System signals
    .clock          ( clock                 ),
    .reset          ( reset                 ),
    .priv_mode      ( priv_mode             ),
    .mt_mode        ( mt_mode               ),

    // Control signals
    .dcache_ready   ( dcache_ready          ),
    .flush_cache    (  wb_xcpt_valid 
                     | wb_flush_cache       ),

    // Request from the ALU stage
    .req_valid      ( req_to_dcache_valid    ), 
    .req_info       ( req_to_dcache_info     ),
    .req_thread_id  ( req_to_dcache_thread_id),

    // Request to WB stage
    .req_wb_valid    ( cache_req_to_wb_valid    ),
    .req_wb_info     ( cache_req_to_wb_info     ),
    .req_wb_thread_id( cache_req_to_wb_thread_id),
    
    // Request to the memory hierarchy
    .req_valid_miss ( dcache_req_valid_miss ),
    .req_info_miss  ( dcache_req_info_miss  ),

    // Response from the memory hierarchy   
    .rsp_valid_miss (   rsp_cache_id  
                      & rsp_valid_miss      ),
    .rsp_thread_id  ( rsp_thread_id         ),
    .rsp_data_miss  ( rsp_data_miss         ),
    .rsp_bus_error  (   rsp_cache_id 
                      & rsp_bus_error       ),

    // Request from WB stage to add a new TLB entry
    .new_tlb_entry    (  wb_new_tlb_entry
                       & wb_new_tlb_id        ),
    .new_tlb_thread_id( wb_new_tlb_thread_id  ),
    .new_tlb_info     ( wb_new_tlb_info       )                      
);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Write Back stage (WB).   
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
wb_top
wb_top
(
    // System signals
    .clock                  ( clock                     ),
    .reset                  ( reset                     ),
    .thread_id              ( cache_req_to_wb_thread_id ),
    .flush_pipeline         ( wb_flush_pipeline         ),
    .flush_cache            ( wb_flush_cache            ),

    // Reorder buffer
    .reorder_buffer_full    ( reorder_buffer_full       ),
    .reorder_buffer_oldest  ( rob_tail                  ),
    .rob_thread_id          ( rob_alu_thread_id         ),
    

      // Request from ALU
    .alu_req_valid          ( alu_req_wb_valid          ),
    .alu_req_info           ( alu_req_wb_info           ),
    .alu_thread_id          ( alu_req_wb_thread_id      ),
    
    .mem_instr_blocked      ( alu_req_wb_mem_blocked    ),
    .mem_instr_info         ( alu_req_wb_dcache_info    ),

    // Request from MUL
    .mul_req_valid          ( mul_req_to_wb_valid       ),
    .mul_req_info           ( mul_req_to_wb_info        ),
    .mul_thread_id          ( mul_req_to_wb_thread_id   ),

    // Request from Cache
    .cache_req_valid        ( cache_req_to_wb_valid     ),
    .cache_req_info         ( cache_req_to_wb_info      ),
    .cache_thread_id        ( cache_req_to_wb_thread_id ),

    // Request to Cache
    .cache_stage_ready      ( alu_cache_stage_free      ), 
    .cache_ready            ( dcache_ready              ),
    .cache_active_thread    ( cache_active_thread       ),
    .req_to_dcache_valid    ( wb_req_to_dcache_valid    ),
    .req_to_dcache_info     ( wb_req_to_dcache_info     ),
    .req_to_dcache_thread_id( wb_req_to_dcache_thread_id),

    // Request to RF
    .req_to_RF_data         ( wb_writeValRF             ),
    .req_to_RF_writeEn      ( wb_writeEnRF              ),
    .req_to_RF_dest         ( wb_destRF                 ),
    .req_to_RF_instr_id     ( wb_write_rob_idRF         ),
    .req_to_RF_thread_id    ( wb_write_thread_idRF      ),

    // Exceptions values to be stored on the RF
    .xcpt_valid             ( wb_xcpt_valid_o           ),
    .xcpt_type              ( wb_xcpt_type              ),
    .xcpt_pc                ( wb_rmPC                   ),
    .xcpt_addr              ( wb_rmAddr                 ),
    .xcpt_thread_id         ( wb_xcpt_thread_id         ),

    // Request to TLB
    .new_tlb_entry          ( wb_new_tlb_entry          ),
    .new_tlb_id             ( wb_new_tlb_id             ), 
    .new_tlb_info           ( wb_new_tlb_info           ), 
    .new_tlb_thread_id      ( wb_new_tlb_thread_id      ),

    // Bypass info    
        // MUL
    .mul_rob_thread_id      ( rob_mul_thread_id         ),
    .mul_src1_id            ( rob_mul_src1_id           ),
    .mul_src2_id            ( rob_mul_src2_id           ),
    .mul_src1_hit           ( rob_mul_src1_hit          ),
    .mul_src2_hit           ( rob_mul_src2_hit          ),
    .mul_src1_data          ( rob_mul_src1_data         ),
    .mul_src2_data          ( rob_mul_src2_data         ),
        // ALU
    .alu_rob_thread_id      ( rob_alu_thread_id         ),
    .alu_src1_id            ( rob_alu_src1_id           ),
    .alu_src2_id            ( rob_alu_src2_id           ),
    .alu_src1_hit           ( rob_alu_src1_hit          ),
    .alu_src2_hit           ( rob_alu_src2_hit          ),
    .alu_src1_data          ( rob_alu_src1_data         ),
    .alu_src2_data          ( rob_alu_src2_data         )
);

endmodule

