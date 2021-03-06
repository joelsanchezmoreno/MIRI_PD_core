`include "soc.vh"
module mul_top
(
    // System signals
    input   logic                           clock,
    input   logic                           reset,

    // Stall pipeline
    input   logic [`THR_PER_CORE-1:0]       flush_mul,
    output  logic [`THR_PER_CORE-1:0]       stall_decode,
    
    // Request from decode stage
        // Operation
    input   logic                           req_mul_valid,  
    input   logic [`THR_PER_CORE_WIDTH-1:0] req_mul_thread_id,
    input   mul_request_t                   req_mul_info,  
    input   logic [`ROB_ID_RANGE]           req_mul_instr_id,
    input   logic [`PC_WIDTH-1:0]           req_mul_pc,
   
        // Exceptions
    input   fetch_xcpt_t                    xcpt_fetch_in,
    input   decode_xcpt_t                   xcpt_decode_in,

    // Request to WB stage
    output  logic                           req_wb_valid,
    output  writeback_request_t             req_wb_info,
    output  logic [`THR_PER_CORE_WIDTH-1:0] req_wb_thread_id,

    // Bypasses
        // Reorder buffer
    output  logic [`THR_PER_CORE_WIDTH-1:0] rob_thread_id,
    output  logic [`ROB_ID_RANGE]           rob_src1_id,
    output  logic [`ROB_ID_RANGE]           rob_src2_id,
    input   logic                           rob_src1_hit,
    input   logic                           rob_src2_hit,
    input   logic [`REG_FILE_DATA_RANGE]    rob_src1_data,
    input   logic [`REG_FILE_DATA_RANGE]    rob_src2_data,

    // Intercept write requests to the Register File
    input logic 				                writeEnRF,
    input logic [`REG_FILE_DATA_RANGE] 		    writeValRF,
    input logic [`THR_PER_CORE_WIDTH-1:0]       write_thread_idRF,
    input logic [`ROB_ID_RANGE]                 write_rob_idRF    
);

logic   [`THR_PER_CORE_WIDTH-1:0] thread_id;
assign thread_id = req_mul_thread_id;

/////////////////
logic fetch_xcpt_valid_in;
assign fetch_xcpt_valid_in = req_mul_valid &  
                             ( xcpt_fetch_in.xcpt_itlb_miss
                             | xcpt_fetch_in.xcpt_bus_error 
                             | xcpt_fetch_in.xcpt_addr_val); 

logic decode_xcpt_valid_in;
assign decode_xcpt_valid_in =  req_mul_valid
                             & xcpt_decode_in.xcpt_illegal_instr;

//////////////////////////////////////
// Stall

logic [`THR_PER_CORE-1:0] stall_decode_ff;

genvar jj;
generate for (jj=0; jj < `THR_PER_CORE; jj++) 
begin
    logic update_ff;
    assign update_ff = (jj == thread_id);

        //      CLK    RST                   EN         DOUT                 DIN               DEF
    `RST_EN_FF(clock, reset | flush_mul[jj], update_ff, stall_decode_ff[jj], stall_decode[jj], 1'b0)
end
endgenerate

//////////////
// Signals between stages
logic [`THR_PER_CORE-1:0][`ALU_MUL_LATENCY:0]                       instr_valid_next;
logic [`THR_PER_CORE-1:0][`ALU_MUL_LATENCY:0]                       instr_valid_ff;      


logic [`THR_PER_CORE-1:0][`ALU_MUL_LATENCY:0][`PC_WIDTH-1:0]        req_wb_pc_ff;
logic [`THR_PER_CORE-1:0][`ALU_MUL_LATENCY:0][`PC_WIDTH-1:0]        req_wb_pc_next;

logic [`THR_PER_CORE-1:0][`ALU_MUL_LATENCY:0][`ALU_OVW_DATA_RANGE]  mul_oper_data_next;
logic [`THR_PER_CORE-1:0][`ALU_MUL_LATENCY:0][`ALU_OVW_DATA_RANGE]  mul_oper_data_ff;

logic [`THR_PER_CORE-1:0][`ALU_MUL_LATENCY:0][`REG_FILE_ADDR_RANGE] rd_addr_next;
logic [`THR_PER_CORE-1:0][`ALU_MUL_LATENCY:0][`REG_FILE_ADDR_RANGE] rd_addr_ff;

logic [`THR_PER_CORE-1:0][`ALU_MUL_LATENCY:0][`ROB_ID_RANGE]        instr_id_next; 
logic [`THR_PER_CORE-1:0][`ALU_MUL_LATENCY:0][`ROB_ID_RANGE]        instr_id_ff; 

logic [`ALU_MUL_LATENCY:0][`THR_PER_CORE_WIDTH-1:0]                 mul_thread_id_next;
logic [`ALU_MUL_LATENCY:0][`THR_PER_CORE_WIDTH-1:0]                 mul_thread_id_ff;

fetch_xcpt_t    [`THR_PER_CORE-1:0][`ALU_MUL_LATENCY:0]             mul_xcpt_fetch_next;
fetch_xcpt_t    [`THR_PER_CORE-1:0][`ALU_MUL_LATENCY:0]             mul_xcpt_fetch_ff;
decode_xcpt_t   [`THR_PER_CORE-1:0][`ALU_MUL_LATENCY:0]             mul_xcpt_decode_next;
decode_xcpt_t   [`THR_PER_CORE-1:0][`ALU_MUL_LATENCY:0]             mul_xcpt_decode_ff;
mul_xcpt_t      [`THR_PER_CORE-1:0][`ALU_MUL_LATENCY:0]             mul_xcpt_stages_next;
mul_xcpt_t      [`THR_PER_CORE-1:0][`ALU_MUL_LATENCY:0]             mul_xcpt_stages_ff;

//  CLK    DOUT              DIN
`FF(clock, mul_thread_id_ff, mul_thread_id_next)

genvar ii;
generate for (ii=0; ii < `THR_PER_CORE; ii++) 
begin
        //  CLK    RST                    DOUT                DIN                  DEF
    `RST_FF(clock, reset | flush_mul[ii], instr_valid_ff[ii], instr_valid_next[ii], '0)
    
    //  CLK    DOUT                  DIN
    `FF(clock, req_wb_pc_ff[ii]    , req_wb_pc_next[ii])
    `FF(clock, mul_oper_data_ff[ii], mul_oper_data_next[ii])
    `FF(clock, rd_addr_ff[ii]      , rd_addr_next[ii])
    `FF(clock, instr_id_ff[ii]     , instr_id_next[ii])

        //  CLK    RST                    DOUT                    DIN                      DEF
    `RST_FF(clock, reset | flush_mul[ii], mul_xcpt_fetch_ff[ii],  mul_xcpt_fetch_next[ii],  '0)
    `RST_FF(clock, reset | flush_mul[ii], mul_xcpt_decode_ff[ii], mul_xcpt_decode_next[ii], '0)
    `RST_FF(clock, reset | flush_mul[ii], mul_xcpt_stages_ff[ii], mul_xcpt_stages_next[ii], '0)
end
endgenerate

// Overflow signal
logic [`ALU_OVW_DATA_RANGE]                      mul_overflow_data;

// Data from bypasses
logic   [`REG_FILE_DATA_RANGE]  ra_data;
logic   [`REG_FILE_DATA_RANGE]  rb_data;

// Bypass signals from RoB
logic rob_blocks_src1;
logic rob_blocks_src2;

logic [`THR_PER_CORE-1:0] rob_src1_found_next;
logic [`THR_PER_CORE-1:0] rob_src1_found_ff;
logic [`THR_PER_CORE-1:0] rob_src2_found_next;
logic [`THR_PER_CORE-1:0] rob_src2_found_ff;

logic [`THR_PER_CORE-1:0][`REG_FILE_DATA_RANGE] rob_src1_data_ff;
logic [`THR_PER_CORE-1:0][`REG_FILE_DATA_RANGE] rob_src2_data_ff;

logic [`THR_PER_CORE-1:0]                       update_rob_data1;
logic [`THR_PER_CORE-1:0]                       update_rob_data2;

logic [`THR_PER_CORE-1:0][`ROB_ID_RANGE]        rob_src1_id_ff;
logic [`THR_PER_CORE-1:0][`ROB_ID_RANGE]        rob_src2_id_ff;

logic [`THR_PER_CORE-1:0]                       req_mul_valid_ff;
mul_request_t [`THR_PER_CORE-1:0]               req_mul_info_ff;
logic [`THR_PER_CORE-1:0][`ROB_ID_RANGE]        req_mul_instr_id_ff;
logic [`THR_PER_CORE-1:0][`PC_WIDTH-1:0]        req_mul_pc_ff;

genvar pp;
generate for (pp=0; pp < `THR_PER_CORE; pp++) 
begin
    logic update_ff;
    assign update_ff = req_mul_valid & (thread_id == pp);


        //     CLK   RST                     EN         DOUT                  DIN            DEF
    `RST_EN_FF(clock, reset | flush_mul[pp], update_ff, req_mul_valid_ff[pp], req_mul_valid, 1'b0)

        // CLK    EN         DOUT                     DIN
    `EN_FF(clock, update_ff, req_mul_info_ff[pp],     req_mul_info)
    `EN_FF(clock, update_ff, req_mul_instr_id_ff[pp], req_mul_instr_id)
    `EN_FF(clock, update_ff, req_mul_pc_ff[pp],       req_mul_pc)

    logic update_rob;
    assign update_rob = (pp == thread_id);

    //     CLK    EN          DOUT                DIN
    `EN_FF(clock, update_rob, rob_src1_id_ff[pp], rob_src1_id)
    `EN_FF(clock, update_rob, rob_src2_id_ff[pp], rob_src2_id)

    //     CLK   EN                     DOUT                   DIN
    `EN_FF(clock, update_rob_data1[pp], rob_src1_data_ff[pp], (rob_src1_hit & pp == rob_thread_id) ? rob_src1_data: writeValRF)
    `EN_FF(clock, update_rob_data2[pp], rob_src2_data_ff[pp], (rob_src2_hit & pp == rob_thread_id) ? rob_src2_data: writeValRF)

        //      CLK   RST                    EN                                 DOUT                   DIN                       DEF
    `RST_EN_FF(clock, reset | flush_mul[pp], update_rob | update_rob_data1[pp], rob_src1_found_ff[pp], rob_src1_found_next[pp], 1'b0)
    `RST_EN_FF(clock, reset | flush_mul[pp], update_rob | update_rob_data2[pp], rob_src2_found_ff[pp], rob_src2_found_next[pp], 1'b0)
end
endgenerate

integer ll;
always_comb
begin
    rob_thread_id = thread_id;
    // Bypass values from RoB
    if (req_mul_valid)
    begin
        rob_src1_id = req_mul_info.ticket_src1;
        rob_src2_id = req_mul_info.ticket_src2;

        rob_blocks_src1 = req_mul_info.rob_blocks_src1;
        rob_blocks_src2 = req_mul_info.rob_blocks_src2;
    end
    else if (stall_decode_ff[thread_id])
    begin
        rob_src1_id = rob_src1_id_ff[thread_id];
        rob_src2_id = rob_src2_id_ff[thread_id];

        rob_blocks_src1 = req_mul_info_ff[thread_id].rob_blocks_src1;
        rob_blocks_src2 = req_mul_info_ff[thread_id].rob_blocks_src2;
    end
    else
    begin
        rob_blocks_src1 = '0;
        rob_blocks_src2 = '0;
    end

    rob_src1_found_next = rob_src1_found_ff;
    rob_src2_found_next = rob_src2_found_ff;

    stall_decode        = stall_decode_ff;

    // Logic to control RoB hit/miss for the current thread
    if (stall_decode_ff[thread_id])
    begin
        // Check if there is a hit on this cycle and store the hit, only if we
        // did not hit last cycle
        if (!rob_src1_found_ff[thread_id])
            rob_src1_found_next[thread_id] =  rob_src1_hit //hit on RoB
                                            | (   writeEnRF && write_thread_idRF == thread_id 
                                               && write_rob_idRF == rob_src1_id);//hit on RF write

        if (!rob_src2_found_ff[thread_id])
            rob_src2_found_next[thread_id] = rob_src2_hit //hit on RoB
                                            |(   writeEnRF && write_thread_idRF == thread_id 
                                               && write_rob_idRF == rob_src2_id);//hit on RF write

        // Check if we can unblock decode stage
        if (rob_blocks_src1 & rob_blocks_src2)
        begin
            stall_decode[thread_id] =!(  (rob_src1_found_ff[thread_id] | rob_src1_hit)
                                       & (rob_src2_found_ff[thread_id] | rob_src2_hit));
        end
        else if ( rob_blocks_src1 )
        begin
            stall_decode[thread_id] = !(rob_src1_found_ff[thread_id] | rob_src1_hit);
        end
        else // if ( rob_blocks_src2 )
        begin
            stall_decode[thread_id] = !(rob_src2_found_ff[thread_id] | rob_src2_hit);
        end
    end // stall_decode_ff

    // if !stall_decode
    else 
    begin
        rob_src1_found_next[thread_id] =  rob_src1_hit //hit on RoB
                                        | (   writeEnRF && write_thread_idRF == thread_id 
                                           && write_rob_idRF == rob_src1_id); //hit on RF write

        rob_src2_found_next[thread_id] =  rob_src2_hit //hit on RoB
                                        | (   writeEnRF && write_thread_idRF == thread_id 
                                           && write_rob_idRF == rob_src2_id); //hit on RF write

        stall_decode[thread_id] =  (  fetch_xcpt_valid_in | decode_xcpt_valid_in ) ? 1'b0 : 
                                   ( req_mul_valid ) ?  ( rob_blocks_src1 
                                                         & !rob_src1_found_next[thread_id]  
                                                         & (req_mul_info.ticket_src1 != req_mul_instr_id))
                                                      | ( rob_blocks_src2 
                                                         & !rob_src2_found_next[thread_id]  
                                                         & (req_mul_info.ticket_src2 != req_mul_instr_id)) :
                                   ( req_mul_valid_ff[thread_id]) ?  ( rob_blocks_src1 
                                                                      & !rob_src1_found_ff[thread_id] 
                                                                      & (req_mul_info_ff[thread_id].ticket_src1 != req_mul_instr_id_ff[thread_id]))
                                                                   | ( rob_blocks_src2 
                                                                      & !rob_src2_found_ff[thread_id] 
                                                                      & (req_mul_info_ff[thread_id].ticket_src2 != req_mul_instr_id_ff[thread_id])) :
                                                                   1'b0;
    end
    
    // Logic to intercept RF writes and update rob FF of non-active threads if
    // needed
    update_rob_data1 = '0;
    update_rob_data2 = '0;
    for (ll = 0; ll < `THR_PER_CORE; ll++)
    begin
            // Check if we hit on the RoB
        update_rob_data1[ll] = rob_blocks_src1 & rob_src1_hit & (thread_id == ll);
        update_rob_data2[ll] = rob_blocks_src2 & rob_src2_hit & (thread_id == ll);

        // Check if the thread RF being written is waiting for a value
        if(writeEnRF && write_thread_idRF == ll && stall_decode_ff[ll])
        begin
            // Check if the register being written is the one the thr was
            // waiting for
            if(write_rob_idRF == rob_src1_id_ff[ll]) //check src1
            begin
                rob_src1_found_next[ll] = 1'b1;
                update_rob_data1[ll]    = 1'b1;
            end
            if(write_rob_idRF == rob_src2_id_ff[ll]) //check src2
            begin
                rob_src2_found_next[ll] = 1'b1;
                update_rob_data2[ll]    = 1'b1;
            end
        end // threadID matches
    end // for loop
end

logic intercept_rf_write_src1;
logic intercept_rf_write_src2;
always_comb
begin

    intercept_rf_write_src1 = (   writeEnRF && write_thread_idRF == thread_id 
                               && write_rob_idRF == rob_src1_id);

    intercept_rf_write_src2 = (   writeEnRF && write_thread_idRF == thread_id 
                               && write_rob_idRF == rob_src2_id);
                               
    ra_data = (rob_blocks_src1) ? (rob_src1_hit           ) ? rob_src1_data               : 
                                  (intercept_rf_write_src1) ? writeValRF : // data from RF write   
                                                              rob_src1_data_ff[thread_id] :
                                  (req_mul_valid   )? req_mul_info.ra_data        :
                                                      req_mul_info_ff[thread_id].ra_data;
                                                    

    rb_data = (rob_blocks_src2) ?  (rob_src2_hit           ) ? rob_src2_data               : 
                                   (intercept_rf_write_src2) ? writeValRF : // data from RF write   
                                                               rob_src2_data_ff[thread_id] :
                                   (req_mul_valid   )?  req_mul_info.rb_data        :
                                                        req_mul_info_ff[thread_id].rb_data;
 
    // Request to MUL stage 0
        // Operation
    mul_thread_id_next[0]              = thread_id;
    instr_id_next[thread_id][0]        = (req_mul_valid) ? req_mul_instr_id : req_mul_instr_id_ff[thread_id];
    instr_valid_next[thread_id][0]     = ( flush_mul[thread_id]       ) ? 1'b0 :
                                         ( stall_decode[thread_id]    ) ? 1'b0 :
                                         ( req_mul_valid              ) ? 1'b1 :
                                         ( stall_decode_ff[thread_id] ) ? req_mul_valid_ff[thread_id] :
                                                                          1'b0;

    req_wb_pc_next[thread_id][0]       = (req_mul_valid) ? req_mul_pc : req_mul_pc_ff[thread_id];
    mul_overflow_data       =  `ZX(`ALU_OVW_DATA_WIDTH,ra_data) * `ZX(`ALU_OVW_DATA_WIDTH,rb_data);
    mul_oper_data_next[thread_id][0]   = mul_overflow_data[`REG_FILE_DATA_RANGE];
    rd_addr_next[thread_id][0]         = (req_mul_valid) ? req_mul_info.rd_addr : req_mul_info_ff[thread_id].rd_addr;
        // Exception
    mul_xcpt_fetch_next[thread_id][0]                  = (instr_valid_next[0]) ? xcpt_fetch_in : '0;
    mul_xcpt_decode_next[thread_id][0]                 = (instr_valid_next[0]) ? xcpt_decode_in : '0;
    mul_xcpt_stages_next[thread_id][0].xcpt_pc         = (req_mul_valid) ? req_mul_pc : req_mul_pc_ff[thread_id];
    mul_xcpt_stages_next[thread_id][0].xcpt_overflow   = (mul_overflow_data[`REG_FILE_DATA_WIDTH+:`REG_FILE_DATA_WIDTH] != '0) 
                                                          & instr_valid_next[thread_id][0];
end

genvar mulStage;

// Generate for MUL stages
generate for(mulStage = 0; mulStage < `MUL_STAGES; mulStage++) //-1 because first stage is FF
begin : gen_mul_stages
    // Local signals to mantain value in case of stall
    logic [`THR_PER_CORE_WIDTH-1:0] mul_thread_id_aux;
    logic [`THR_PER_CORE_WIDTH-1:0] thread_id_out;
    logic                           instr_valid_aux;      
    logic [`PC_WIDTH-1:0]           req_wb_pc_aux;
    logic [`ALU_OVW_DATA_RANGE]     mul_oper_data_aux;
    logic [`REG_FILE_ADDR_RANGE]    rd_addr_aux;
    logic [`ROB_ID_RANGE]           instr_id_aux;

    fetch_xcpt_t                 mul_xcpt_fetch_aux;
    decode_xcpt_t                mul_xcpt_decode_aux;
    mul_xcpt_t                   mul_xcpt_stages_aux;

    assign mul_thread_id_aux    = mul_thread_id_ff[mulStage];

    assign instr_valid_aux      = instr_valid_ff[mul_thread_id_aux][mulStage];

    assign req_wb_pc_aux        = req_wb_pc_ff[mul_thread_id_aux][mulStage];

    assign mul_oper_data_aux    = mul_oper_data_ff[mul_thread_id_aux][mulStage];

    assign rd_addr_aux          = rd_addr_ff[mul_thread_id_aux][mulStage];

    assign instr_id_aux         = instr_id_ff[mul_thread_id_aux][mulStage];

    assign mul_xcpt_fetch_aux   = mul_xcpt_fetch_ff[mul_thread_id_aux][mulStage];

    assign mul_xcpt_decode_aux  = mul_xcpt_decode_ff[mul_thread_id_aux][mulStage];

    assign mul_xcpt_stages_aux  = mul_xcpt_stages_ff[mul_thread_id_aux][mulStage];

    mul_stage
    mul_stage
    (
    // Signals from previous stage        
        // MUL request
        .thread_id_in       ( mul_thread_id_aux    ),
        .instr_valid_in     ( instr_valid_aux      ),
        .instr_id_in        ( instr_id_aux         ),
        .program_counter_in ( req_wb_pc_aux        ),
        .dest_reg_in        ( rd_addr_aux          ),
        .data_result_in     ( mul_oper_data_aux    ),
    
        // Exception input
        .xcpt_fetch_in      ( mul_xcpt_fetch_aux    ),
        .xcpt_decode_in     ( mul_xcpt_decode_aux   ),
        .xcpt_mul_in        ( mul_xcpt_stages_aux   ),

    // Signals to next stage        
        // MUL request
        .thread_id_out      ( thread_id_out                                     ),
        .instr_valid_out    ( instr_valid_next[thread_id_out][mulStage+1]       ),
        .instr_id_out       ( instr_id_next[thread_id_out][mulStage+1]          ),
        .program_counter_out( req_wb_pc_next[thread_id_out][mulStage+1]         ),
        .dest_reg_out       ( rd_addr_next[thread_id_out][mulStage+1]           ),
        .data_result_out    ( mul_oper_data_next[thread_id_out][mulStage+1]     ),

        // Exception output
        .xcpt_fetch_out     ( mul_xcpt_fetch_next[thread_id_out][mulStage+1]    ),
        .xcpt_decode_out    ( mul_xcpt_decode_next[thread_id_out][mulStage+1]   ),
        .xcpt_mul_out       ( mul_xcpt_stages_next[thread_id_out][mulStage+1]   )
    );
    assign  mul_thread_id_next[mulStage+1] = thread_id_out;
end
endgenerate

logic                           req_wb_valid_next;
writeback_request_t             req_wb_info_next;
logic                           req_wb_valid_ff;
writeback_request_t             req_wb_info_ff;
logic [`THR_PER_CORE_WIDTH-1:0] previous_thread;

 // CLK    DOUT             DIN            
`FF(clock, previous_thread, mul_stage_thread_id)


    //  CLK    RST    DOUT             DIN                DEF
`RST_FF(clock, reset, req_wb_valid_ff, req_wb_valid_next, '0)

 // CLK    DOUT                DIN                  
`FF(clock, req_wb_info_ff, req_wb_info_next)


logic [`THR_PER_CORE_WIDTH-1:0] mul_stage_thread_id;

logic fetch_xcpt_valid;
logic decode_xcpt_valid;
logic mul_xcpt_valid;

// Request to WB
always_comb
begin
    mul_stage_thread_id  = mul_thread_id_ff[`MUL_STAGES];
    assign fetch_xcpt_valid  =   mul_xcpt_fetch_ff[mul_stage_thread_id][`MUL_STAGES].xcpt_itlb_miss
                               | mul_xcpt_fetch_ff[mul_stage_thread_id][`MUL_STAGES].xcpt_bus_error;
    assign decode_xcpt_valid = mul_xcpt_decode_ff[mul_stage_thread_id][`MUL_STAGES].xcpt_illegal_instr;
    assign mul_xcpt_valid    = mul_xcpt_stages_ff[mul_stage_thread_id][`MUL_STAGES].xcpt_overflow;

    req_wb_valid_next = (flush_mul[mul_stage_thread_id]                  ) ? 1'b0 :
                        (  fetch_xcpt_valid
                         | decode_xcpt_valid   
                         | mul_xcpt_valid                                ) ? 1'b1 : 
                        (instr_valid_ff[mul_stage_thread_id][`MUL_STAGES]) ? 1'b1 :
                                                                             1'b0 ;
  
    req_wb_info_next.chg_core_mode = 1'b0;
    req_wb_info_next.instr_id    = instr_id_ff[mul_stage_thread_id][`MUL_STAGES];
    req_wb_info_next.pc          = req_wb_pc_ff[mul_stage_thread_id][`MUL_STAGES];

    req_wb_info_next.tlbwrite     = 1'b0;  
    req_wb_info_next.tlb_id       = '0; 
    req_wb_info_next.tlb_req_info = '0;
                                
    req_wb_info_next.rf_wen       = 1'b1;
    req_wb_info_next.rf_dest      = rd_addr_ff[mul_stage_thread_id][`MUL_STAGES];
    req_wb_info_next.rf_data      = mul_oper_data_ff[mul_stage_thread_id][`MUL_STAGES];
                      
    req_wb_info_next.xcpt_fetch   = mul_xcpt_fetch_ff[mul_stage_thread_id][`MUL_STAGES];
    req_wb_info_next.xcpt_decode  = mul_xcpt_decode_ff[mul_stage_thread_id][`MUL_STAGES];
    req_wb_info_next.xcpt_alu     = '0;
    req_wb_info_next.xcpt_mul     = mul_xcpt_stages_ff[mul_stage_thread_id][`MUL_STAGES];  //overflow
    req_wb_info_next.xcpt_cache   = '0;
end

assign req_wb_valid      = (flush_mul[previous_thread]) ? 1'b0 : req_wb_valid_ff;
assign req_wb_info       = req_wb_info_ff;
assign req_wb_thread_id  = previous_thread;


/////////////////////////////////
// VERBOSE
`ifdef VERBOSE_MUL
always_ff @(posedge clock)
begin
    if (req_wb_valid)
    begin
        $display("[MUL] Request to WB. PC = %h",req_wb_info.pc);
        $display("      req_wb_info.rf_wen  =  %h",req_wb_info.rf_wen);
        $display("      req_wb_info.rf_dest =  %h",req_wb_info.rf_dest);
        $display("      req_wb_info.rf_data =  %h",req_wb_info.rf_data);
    end
end
`endif
endmodule
