`include "soc.vh"
module alu_top
(
    // System signals
    input   logic                           clock,
    input   logic                           reset,

    // Control signals with RoB
    output  logic [`THR_PER_CORE_WIDTH-1:0]  rob_thread_id,
    input   logic [`ROB_NUM_ENTRIES_W_RANGE] rob_tail,
    output  logic                            alu_no_req_to_cache,

    // Stall pipeline
    input   logic [`THR_PER_CORE-1:0]       flush_alu,
    input   logic [`THR_PER_CORE-1:0]       dcache_ready,
    output  logic [`THR_PER_CORE-1:0]       stall_decode_o,

    // Exceptions
    input   fetch_xcpt_t                    xcpt_fetch_in,
    input   decode_xcpt_t                   xcpt_decode_in,
    
    // Request from decode stage
    input   logic                           req_alu_valid,  
    input   logic [`THR_PER_CORE_WIDTH-1:0] req_alu_thread_id,
    input   alu_request_t                   req_alu_info,  
    input   logic [`ROB_ID_RANGE]           req_alu_instr_id,
    input   logic [`PC_WIDTH-1:0]           req_alu_pc,
   
    // Request to dcache stage 
    output  logic                           req_dcache_valid,
    output  dcache_request_t                req_dcache_info,
    output  logic [`THR_PER_CORE_WIDTH-1:0] req_dcache_thread_id,
    
    // Request to WB stage
    output  logic                           req_wb_valid,
    output  logic [`THR_PER_CORE_WIDTH-1:0] req_wb_thread_id,
    output  writeback_request_t             req_wb_info,
    output  logic                           req_wb_mem_blocked,
    output  dcache_request_t                req_wb_dcache_info,

    // Branch signals to fetch stage
    output  logic                           take_branch,
    output  logic [`THR_PER_CORE_WIDTH-1:0] branch_thr_id,
    output  logic [`PC_WIDTH-1:0]           branch_pc,
    output  logic [`THR_PER_CORE-1:0]       iret_instr,

    // Bypasses
        // Reorder buffer
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
assign thread_id = req_alu_thread_id;

logic [`THR_PER_CORE_WIDTH-1:0] previous_thread;

 // CLK    DOUT             DIN            
`FF(clock, previous_thread, thread_id)

//////////////////////////////////////
// Stall
logic [`THR_PER_CORE-1:0] stall_decode;
logic [`THR_PER_CORE-1:0] stall_decode_ff;

genvar jj;
generate for (jj=0; jj < `THR_PER_CORE; jj++) 
begin
    logic update_ff;
    assign update_ff = (jj == thread_id);

        //      CLK    RST                   EN         DOUT                 DIN               DEF
    `RST_EN_FF(clock, reset | flush_alu[jj], update_ff, stall_decode_ff[jj], stall_decode[jj], 1'b0)
end
endgenerate

//assign stall_decode_o = stall_decode_ff;
assign stall_decode_o = stall_decode;


logic [`THR_PER_CORE-1:0]                       req_alu_valid_ff;
alu_request_t [`THR_PER_CORE-1:0]               req_alu_info_ff;
logic [`THR_PER_CORE-1:0][`ROB_ID_RANGE]        req_alu_instr_id_ff;
logic [`THR_PER_CORE-1:0][`PC_WIDTH-1:0]        req_alu_pc_ff;

genvar u;
generate for (u=0; u < `THR_PER_CORE; u++) 
begin
    logic update_ff;
    assign update_ff = req_alu_valid & (thread_id == u);

    logic taken_branch;
    assign taken_branch = take_branch && (thread_id == branch_thr_id);

        //     CLK   RST                                   EN        DOUT                 DIN            DEF
    `RST_EN_FF(clock, reset | flush_alu[u] | taken_branch, update_ff, req_alu_valid_ff[u], req_alu_valid, 1'b0)

        // CLK    EN         DOUT                    DIN
    `EN_FF(clock, update_ff, req_alu_info_ff[u],     req_alu_info)
    `EN_FF(clock, update_ff, req_alu_instr_id_ff[u], req_alu_instr_id)
    `EN_FF(clock, update_ff, req_alu_pc_ff[u],       req_alu_pc)
end
endgenerate



//////////////////////////////////////
// Exceptions
alu_xcpt_t      xcpt_alu;

logic fetch_xcpt_valid;
assign fetch_xcpt_valid = req_alu_valid &  
                         ( xcpt_fetch_in.xcpt_itlb_miss
                         | xcpt_fetch_in.xcpt_bus_error 
                         | xcpt_fetch_in.xcpt_addr_val); 

logic decode_xcpt_valid;
assign decode_xcpt_valid =  req_alu_valid
                          & xcpt_decode_in.xcpt_illegal_instr;



////////////////////////////////////
// Request to D$ stage or WB stage
    
// Cache will not receive a request next cycle from ALU because the thread was
// stalled or had an xcpt going to WB. This means that we can send a queued
// request on the RoB to the data cache 
//
logic               req_wb_valid_next;
assign alu_no_req_to_cache = req_wb_valid_next | !req_dcache_valid_next;

logic               req_dcache_valid_next;
logic               req_wb_mem_blocked_next;
dcache_request_t    req_dcache_info_next;
logic               req_dcache_valid_ff;
dcache_request_t    req_dcache_info_ff;

logic [`THR_PER_CORE-1:0] req_wb_mem_blocked_ff;

    //  CLK    RST    DOUT                 DIN                    DEF
`RST_FF(clock, reset, req_dcache_valid_ff, req_dcache_valid_next, '0)

 // CLK    DOUT                DIN                  
`FF(clock, req_dcache_info_ff, req_dcache_info_next)

// Request to WB stage in case ST/LD is not the oldest instr or dcache is not
// ready
logic wb_mem_blocked_type;
logic [`THR_PER_CORE-1:0] wb_mem_blocked_type_ff;
assign wb_mem_blocked_type =   is_m_type_instr(req_alu_info.opcode) 
                             & ((rob_tail != req_alu_instr_id) | !dcache_ready[thread_id] );

assign req_wb_mem_blocked_next = (flush_alu[thread_id]       ) ? 1'b0 :
                                 (stall_decode[thread_id]    ) ? 1'b0 : 
                                 (req_alu_valid              ) ? wb_mem_blocked_type :
                                 (stall_decode_ff[thread_id] ) ? (  req_alu_valid_ff[thread_id] 
                                                                  & wb_mem_blocked_type_ff[thread_id]):
                                                                 1'b0;

assign req_wb_mem_blocked       = req_wb_mem_blocked_ff[previous_thread];
assign req_wb_dcache_info       = req_dcache_info_ff;
                                                  
// Request to D$ stage in case ST/LD is the oldest instr on the pipe                                                  
logic dcache_mem_type;
logic [`THR_PER_CORE-1:0]  dcache_mem_type_ff;

assign dcache_mem_type =   is_m_type_instr(req_alu_info.opcode)
                         & dcache_ready[thread_id]
                         & (rob_tail == req_alu_instr_id);


assign req_dcache_valid_next = ( flush_alu[thread_id]       ) ? 1'b0 :     
                               (  fetch_xcpt_valid        
                                | decode_xcpt_valid 
                                | xcpt_alu.xcpt_overflow    ) ? 1'b0 : // in case of xcpt go to wb 
                               ( stall_decode[thread_id]    ) ? 1'b0 : 
                               ( req_alu_valid              ) ? dcache_mem_type :
                               ( stall_decode_ff[thread_id] ) ? (  req_alu_valid_ff[thread_id]
                                                                 & is_m_type_instr(req_alu_info_ff[thread_id].opcode)) :
                                                                1'b0;

assign req_dcache_valid = (flush_alu[previous_thread]) ? 1'b0 : req_dcache_valid_ff;
assign req_dcache_info  = req_dcache_info_ff;
assign req_dcache_thread_id = previous_thread;

genvar ii;
generate for (ii=0; ii < `THR_PER_CORE; ii++) 
begin
    logic update_ff;
    assign update_ff = (ii == thread_id); 

        //     CLK    RST                    EN         DOUT                       DIN                      DEF
    `RST_EN_FF(clock, reset | flush_alu[ii], update_ff, req_wb_mem_blocked_ff[ii], req_wb_mem_blocked_next, '0)

        //     CLK    RST                    EN         DOUT                        DIN                  DEF
    `RST_EN_FF(clock, reset | flush_alu[ii], update_ff, wb_mem_blocked_type_ff[ii], wb_mem_blocked_type, '0)
    
        // CLK    EN         DOUT                    DIN                  
    `EN_FF(clock, update_ff, dcache_mem_type_ff[ii], dcache_mem_type)
end
endgenerate

////////////////////////////////////
// Request to WB

logic alu_to_wb_intr;
logic [`THR_PER_CORE-1:0]  alu_to_wb_intr_ff;

assign alu_to_wb_intr = (  is_r_type_instr(req_alu_info.opcode) 
                         | is_mov_instr(req_alu_info.opcode) 
                         | is_nop_instr(req_alu_info.opcode) 
                         | is_branch_type_instr(req_alu_info.opcode) 
                         | is_jump_instr(req_alu_info.opcode) 
                         | is_iret_instr(req_alu_info.opcode) 
                         | is_tlb_instr(req_alu_info.opcode)
                         | is_get_thread_id_instr(req_alu_info.opcode)
                         | is_privileged_instr(req_alu_info.opcode));
genvar kk;
generate for (kk=0; kk < `THR_PER_CORE; kk++) 
begin
    logic update_ff;
    assign update_ff = req_alu_valid & (kk == thread_id); 
    
        // CLK    EN         DOUT                   DIN                  
    `EN_FF(clock, update_ff, alu_to_wb_intr_ff[kk], alu_to_wb_intr)
end
endgenerate

// TLB write request
logic           tlb_req_valid_next;
logic           tlb_id_next;
tlb_req_info_t  tlb_req_info_next;

writeback_request_t req_wb_info_next;
logic               req_wb_valid_ff;
writeback_request_t req_wb_info_ff;


 // CLK    DOUT                DIN                  
`FF(clock, req_wb_info_ff, req_wb_info_next)

    //  CLK    RST    DOUT             DIN                DEF
`RST_FF(clock, reset, req_wb_valid_ff, req_wb_valid_next, '0)

assign req_wb_valid_next = ( flush_alu[thread_id]   ) ? 1'b0 :
                           (  fetch_xcpt_valid
                            | decode_xcpt_valid     
                            | xcpt_alu.xcpt_overflow) ? 1'b1 : 
                           ( stall_decode[thread_id]) ? 1'b0 :
                           ( req_alu_valid          ) ? alu_to_wb_intr :
                           ( stall_decode_ff[thread_id]) ? (  req_alu_valid_ff[thread_id]
                                                            & alu_to_wb_intr_ff[thread_id]) :
                                                           1'b0;

                                                    
assign req_wb_valid     = (flush_alu[previous_thread]) ? 1'b0 : req_wb_valid_ff;
assign req_wb_info      = req_wb_info_ff;
assign req_wb_thread_id = previous_thread;



////////////////////////////////////
// Branch signals
logic [`PC_WIDTH-1:0]   branch_pc_next;
logic			 	    take_branch_next;
logic [`THR_PER_CORE-1:0] iret_instr_next;

assign branch_thr_id = previous_thread;

//      CLK    RST    DOUT         DIN               DEF
`RST_FF(clock, reset, take_branch, take_branch_next, 1'b0)
`RST_FF(clock, reset, iret_instr,  iret_instr_next,  '0)

//  CLK    DOUT       DIN
`FF(clock, branch_pc, branch_pc_next)

////////////////////////////////////
// Perform ALU instruction

logic   [`REG_FILE_DATA_RANGE]  ra_data;
logic   [`REG_FILE_DATA_RANGE]  rb_data;

// Overflow signal
logic [`ALU_OVW_DATA_RANGE] oper_data;
logic [`ALU_OVW_DATA_RANGE] oper_data_2;

// Bypass signals from RoB
logic rob_blocks_src1;
logic rob_blocks_src2;

logic [`THR_PER_CORE-1:0] rob_src1_found_next;
logic [`THR_PER_CORE-1:0] rob_src1_found_ff;
logic [`THR_PER_CORE-1:0] rob_src2_found_next;
logic [`THR_PER_CORE-1:0] rob_src2_found_ff;

logic [`THR_PER_CORE-1:0][`REG_FILE_DATA_RANGE]  rob_src1_data_ff;
logic [`THR_PER_CORE-1:0][`REG_FILE_DATA_RANGE]  rob_src2_data_ff;

logic [`THR_PER_CORE-1:0]                       update_rob_data1;
logic [`THR_PER_CORE-1:0]                       update_rob_data2;

logic [`THR_PER_CORE-1:0][`ROB_ID_RANGE]        rob_src1_id_ff;
logic [`THR_PER_CORE-1:0][`ROB_ID_RANGE]        rob_src2_id_ff;

genvar pp;
generate for (pp=0; pp < `THR_PER_CORE; pp++) 
begin
    logic update_ff;
    assign update_ff = (thread_id == pp);

        // CLK    EN         DOUT                DIN
    `EN_FF(clock, update_ff, rob_src1_id_ff[pp], rob_src1_id)
    `EN_FF(clock, update_ff, rob_src2_id_ff[pp], rob_src2_id)

        // CLK   EN                     DOUT                   DIN
    `EN_FF(clock, update_rob_data1[pp], rob_src1_data_ff[pp], (rob_src1_hit & pp == rob_thread_id) ? rob_src1_data: writeValRF)
    `EN_FF(clock, update_rob_data2[pp], rob_src2_data_ff[pp], (rob_src2_hit & pp == rob_thread_id) ? rob_src2_data: writeValRF)
    
        //      CLK   RST                    EN                    DOUT                   DIN                       DEF
    `RST_EN_FF(clock, reset | flush_alu[pp], update_ff | update_rob_data1[pp], rob_src1_found_ff[pp], rob_src1_found_next[pp], 1'b0)
    `RST_EN_FF(clock, reset | flush_alu[pp], update_ff | update_rob_data2[pp], rob_src2_found_ff[pp], rob_src2_found_next[pp], 1'b0)
end
endgenerate

integer ll;
always_comb
begin
    rob_thread_id = thread_id;
    // Bypass values from RoB
    if (req_alu_valid)
    begin
        rob_src1_id = req_alu_info.ticket_src1;
        rob_src2_id = req_alu_info.ticket_src2;

        rob_blocks_src1 = req_alu_info.rob_blocks_src1;
        rob_blocks_src2 = req_alu_info.rob_blocks_src2;
    end
    else if (stall_decode_ff[thread_id])
    begin
        rob_src1_id = rob_src1_id_ff[thread_id];
        rob_src2_id = rob_src2_id_ff[thread_id];

        rob_blocks_src1 = req_alu_info_ff[thread_id].rob_blocks_src1;
        rob_blocks_src2 = req_alu_info_ff[thread_id].rob_blocks_src2;
    end
    else
    begin
        rob_blocks_src1 = '0;
        rob_blocks_src2 = '0;
    end

    rob_src1_found_next = rob_src1_found_ff;
    rob_src2_found_next = rob_src2_found_ff;

    stall_decode    = stall_decode_ff;

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

        rob_src2_found_next[thread_id] = rob_src2_hit //hit on RoB
                                        | (   writeEnRF && write_thread_idRF == thread_id 
                                           && write_rob_idRF == rob_src2_id); //hit on RF write

        stall_decode[thread_id] = ( fetch_xcpt_valid | decode_xcpt_valid ) ? 1'b0 : 
                                  ( req_alu_valid      ) ?   (  rob_blocks_src1 
                                                              & !rob_src1_found_next[thread_id]  
                                                              & (req_alu_info.ticket_src1 != req_alu_instr_id))
                                                           | (  rob_blocks_src2 
                                                              & !rob_src2_found_next[thread_id]  
                                                              & (req_alu_info.ticket_src2 != req_alu_instr_id)) :
                                  ( req_alu_valid_ff[thread_id]) ?  (  rob_blocks_src1 
                                                                     & !rob_src1_found_ff[thread_id] 
                                                                     & (req_alu_info_ff[thread_id].ticket_src1 != req_alu_instr_id_ff[thread_id]))
                                                                  | (  rob_blocks_src2 
                                                                     & !rob_src2_found_ff[thread_id] 
                                                                     & (req_alu_info_ff[thread_id].ticket_src2 != req_alu_instr_id_ff[thread_id])) :
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
        end // weiteEnRF and threadID matches
    end // for loop
end

logic   [`ALU_OFFSET_RANGE]     offset;  // Offset value

logic [`REG_FILE_DATA_RANGE] rf_data;
logic [`INSTR_OPCODE_RANGE]  opcode;
logic intercept_rf_write_src1;
logic intercept_rf_write_src2;

always_comb
begin
    rf_data = '0;

    opcode = (req_alu_valid) ? req_alu_info.opcode : req_alu_info_ff[thread_id].opcode;

    // Branch
	take_branch_next    = 1'b0;
    iret_instr_next     = '0;
    branch_pc_next      = '0;

    // Dcache request
    req_dcache_info_next = '0;
    req_dcache_info_next.pc          = (req_alu_valid) ? req_alu_pc           : req_alu_pc_ff[thread_id];
    req_dcache_info_next.instr_id    = (req_alu_valid) ? req_alu_instr_id     : req_alu_instr_id_ff[thread_id];
    req_dcache_info_next.rd_addr     = (req_alu_valid) ? req_alu_info.rd_addr : req_alu_info_ff[thread_id].rd_addr;
    req_dcache_info_next.xcpt_fetch  = '0 ;
    req_dcache_info_next.xcpt_decode = '0;

    // Exception
    xcpt_alu.xcpt_overflow = 1'b0 ;
    xcpt_alu.xcpt_pc       = (req_alu_valid) ? req_alu_pc : req_alu_pc_ff[thread_id];


    intercept_rf_write_src1 = (   writeEnRF && write_thread_idRF == thread_id 
                               && write_rob_idRF == rob_src1_id);

    intercept_rf_write_src2 = (   writeEnRF && write_thread_idRF == thread_id 
                               && write_rob_idRF == rob_src2_id);



    ra_data = ( rob_blocks_src1 ) ? (rob_src1_hit) ? rob_src1_data         : // data from RoB
                                    (intercept_rf_write_src1) ? writeValRF : // data from RF write   
                                                                rob_src1_data_ff :
                                    (req_alu_valid)? req_alu_info.ra_data        :
                                                     req_alu_info_ff[thread_id].ra_data;


    rb_data = ( rob_blocks_src2 ) ? (rob_src2_hit) ? rob_src2_data         : // data from RoB
                                    (intercept_rf_write_src2) ? writeValRF : // data from RF write   
                                                                rob_src2_data_ff : // stored data
                                    (req_alu_valid)? req_alu_info.rb_data        :
                                                     req_alu_info_ff[thread_id].rb_data;

    offset =  (req_alu_valid) ? req_alu_info.offset : req_alu_info_ff[thread_id].offset;

    // TLB
    tlb_req_valid_next  = 1'b0;

    // ADD
	if (opcode == `INSTR_ADD_OPCODE)
	begin
        oper_data =  `ZX(`ALU_OVW_DATA_WIDTH,ra_data) + `ZX(`ALU_OVW_DATA_WIDTH,rb_data);
        rf_data   =  oper_data[`REG_FILE_DATA_RANGE];
        xcpt_alu.xcpt_overflow = !stall_decode[thread_id] & (req_alu_valid_ff[thread_id] | req_alu_valid) ? (oper_data[`REG_FILE_DATA_WIDTH+:`REG_FILE_DATA_WIDTH] != '0) : 1'b0;
    end
    // SUB
	else if (opcode == `INSTR_SUB_OPCODE)
    begin
        rf_data = ra_data - rb_data;        
    end
    //ADDI
    else if (opcode == `INSTR_ADDI_OPCODE)
    begin
        oper_data =  `ZX(`ALU_OVW_DATA_WIDTH,ra_data) + `ZX(`ALU_OVW_DATA_WIDTH,offset);
        rf_data   =  oper_data[`REG_FILE_DATA_RANGE];
        xcpt_alu.xcpt_overflow =  !stall_decode[thread_id] & (req_alu_valid_ff[thread_id] | req_alu_valid) ? (oper_data[`REG_FILE_DATA_WIDTH+:`REG_FILE_DATA_WIDTH] != '0) : 1'b0;
    end
    //SLL
    else if (opcode == `INSTR_SLL_OPCODE)
    begin
        oper_data =  `ZX(`ALU_OVW_DATA_WIDTH,ra_data) << `ZX(`ALU_OVW_DATA_WIDTH,offset);
        rf_data   =  oper_data[`REG_FILE_DATA_RANGE];
        xcpt_alu.xcpt_overflow =  !stall_decode[thread_id] & (req_alu_valid_ff[thread_id] | req_alu_valid) ? (oper_data[`REG_FILE_DATA_WIDTH+:`REG_FILE_DATA_WIDTH] != '0) : 1'b0;
    end
    //SRL
    else if (opcode == `INSTR_SRL_OPCODE)
    begin
        oper_data =  `ZX(`ALU_OVW_DATA_WIDTH,ra_data) >> `ZX(`ALU_OVW_DATA_WIDTH,offset);
        rf_data   =  oper_data[`REG_FILE_DATA_RANGE];
        xcpt_alu.xcpt_overflow =  !stall_decode[thread_id] & (req_alu_valid_ff[thread_id] | req_alu_valid) ? (oper_data[`REG_FILE_DATA_WIDTH+:`REG_FILE_DATA_WIDTH] != '0) : 1'b0;
    end
    // MEM
	else if (is_m_type_instr(opcode)) 
    begin
        //LD
        if (is_load_instr(opcode))
        begin
            oper_data = `ZX(`ALU_OVW_DATA_WIDTH,ra_data + `ZX(`REG_FILE_DATA_WIDTH,offset)) ;
        end
        //ST
        else
        begin
            oper_data = `ZX(`ALU_OVW_DATA_WIDTH,rb_data + `ZX(`REG_FILE_DATA_WIDTH,offset)) ;
        end

        // Used only on store requests
		oper_data_2 = `ZX(`ALU_OVW_DATA_WIDTH,ra_data);
        
        // Specify LD or ST for dcache request
        if (is_load_instr(opcode))
        begin
            req_dcache_info_next.is_store    = 1'b0;
            req_dcache_info_next.conditional = is_mem_conditional_instr(opcode);
        end
        else
        begin
            req_dcache_info_next.is_store    = 1'b1;
            req_dcache_info_next.conditional = is_mem_conditional_instr(opcode);
        end

        // Specify size for dcache request
        if (  opcode == `INSTR_LDB_OPCODE 
            | opcode == `INSTR_STB_OPCODE 
            | opcode == `INSTR_STCB_OPCODE)
            req_dcache_info_next.size = Byte;
        else
            req_dcache_info_next.size = Word;

        // Check possible overflow
        req_dcache_info_next.addr   =  oper_data[`REG_FILE_DATA_RANGE];
        req_dcache_info_next.data   =  oper_data_2[`REG_FILE_DATA_RANGE];
        xcpt_alu.xcpt_overflow      =  !stall_decode[thread_id] & (req_alu_valid_ff[thread_id] | req_alu_valid) ? 
                                                            (  (oper_data[`REG_FILE_DATA_WIDTH+:`REG_FILE_DATA_WIDTH] != '0)
                                                             | (oper_data_2[`REG_FILE_DATA_WIDTH+:`REG_FILE_DATA_WIDTH] != '0)) : 1'b0;
    end	
    // BEQ
	else if (opcode == `INSTR_BEQ_OPCODE) 
	begin
        if (ra_data == rb_data)
        begin
            branch_pc_next   = `ZX(`PC_WIDTH,offset);
		    //take_branch_next =  !stall_decode[thread_id] & (req_alu_valid_ff[thread_id] | req_alu_valid);
            take_branch_next =  !stall_decode[thread_id] & (req_alu_valid | (req_alu_valid_ff[thread_id] & stall_decode_ff[thread_id]));
        end
	end
    // BNE
	else if (opcode == `INSTR_BNE_OPCODE) 
	begin
        if (ra_data != rb_data)
        begin
            branch_pc_next   = `ZX(`PC_WIDTH,offset);
            take_branch_next =  !stall_decode[thread_id] & (req_alu_valid | (req_alu_valid_ff[thread_id] & stall_decode_ff[thread_id]));
        end
	end
    // BLT
	else if (opcode == `INSTR_BLT_OPCODE) 
	begin
        if (ra_data < rb_data)
        begin
            branch_pc_next   = `ZX(`PC_WIDTH,offset);
            take_branch_next =  !stall_decode[thread_id] & (req_alu_valid | (req_alu_valid_ff[thread_id] & stall_decode_ff[thread_id]));
        end
	end
    // BGT
	else if (opcode == `INSTR_BGT_OPCODE) 
	begin
        if (ra_data > rb_data)
        begin
            branch_pc_next   = `ZX(`PC_WIDTH,offset);
            take_branch_next =  !stall_decode[thread_id] & (req_alu_valid | (req_alu_valid_ff[thread_id] & stall_decode_ff[thread_id]));
        end
	end
    // BLE
	else if (opcode == `INSTR_BLE_OPCODE) 
	begin
        if (ra_data <= rb_data)
        begin
            branch_pc_next   = `ZX(`PC_WIDTH,offset);
            take_branch_next =  !stall_decode[thread_id] & (req_alu_valid | (req_alu_valid_ff[thread_id] & stall_decode_ff[thread_id]));
        end
	end
    // BGE
	else if (opcode == `INSTR_BGE_OPCODE) 
	begin
        if (ra_data >= rb_data)
        begin
            branch_pc_next   = `ZX(`PC_WIDTH,offset);
            take_branch_next =  !stall_decode[thread_id] & (req_alu_valid | (req_alu_valid_ff[thread_id] & stall_decode_ff[thread_id]));
        end
	end
    // JUMP
    else if (opcode == `INSTR_JUMP_OPCODE) 
	begin
		branch_pc_next   = `ZX(`PC_WIDTH,offset);
        take_branch_next =  !stall_decode[thread_id] & (req_alu_valid | (req_alu_valid_ff[thread_id] & stall_decode_ff[thread_id]));
	end

    // MOV
    else if (is_mov_instr(opcode)) 
    begin
        rf_data = (!stall_decode[thread_id] & req_alu_valid) ? req_alu_info.ra_data : req_alu_info_ff[thread_id].ra_data;          
    end
    // TLBWRITE
	else if (is_tlb_instr(opcode)) 
    begin
        tlb_req_valid_next           = !stall_decode[thread_id] & (req_alu_valid_ff[thread_id] | req_alu_valid);
        tlb_id_next                  = offset[0];
        tlb_req_info_next.virt_addr  = ra_data;
        tlb_req_info_next.phy_addr   = rb_data;
        tlb_req_info_next.writePriv  = 1'b1;            
    end
    // IRET
	else if (is_iret_instr(opcode)) 
    begin
        ra_data = (!stall_decode[thread_id] & req_alu_valid) ? req_alu_info.ra_data : req_alu_info_ff[thread_id].ra_data;
        branch_pc_next   = `ZX(`PC_WIDTH,ra_data);
		take_branch_next =  !stall_decode[thread_id] & (req_alu_valid_ff[thread_id] | req_alu_valid);
        iret_instr_next[thread_id] = !stall_decode[thread_id] & (req_alu_valid_ff[thread_id] | req_alu_valid);
    end
    //GET_THREAD_ID
    else if (opcode == `INSTR_GET_THR_ID_OPCODE)
    begin
        oper_data =  `ZX(`ALU_OVW_DATA_WIDTH,thread_id);
        rf_data   =  oper_data[`REG_FILE_DATA_RANGE];
    end

    req_dcache_info_next.xcpt_alu    = '0;

    req_wb_info_next.instr_id    = (req_alu_valid) ? req_alu_instr_id : req_alu_instr_id_ff[thread_id];
    req_wb_info_next.pc          = (req_alu_valid) ? req_alu_pc       : req_alu_pc_ff[thread_id];

    req_wb_info_next.tlbwrite     = tlb_req_valid_next; 
    req_wb_info_next.tlb_id       = tlb_id_next; 
    req_wb_info_next.tlb_req_info = tlb_req_info_next;
                                
    req_wb_info_next.rf_wen       =  is_r_type_instr(opcode) | is_mov_instr(opcode) | is_get_thread_id_instr(opcode);

    req_wb_info_next.rf_dest      = (req_alu_valid) ? req_alu_info.rd_addr :
                                                      req_alu_info_ff[thread_id].rd_addr;
    req_wb_info_next.rf_data      = rf_data;
    req_wb_info_next.chg_core_mode= is_privileged_instr(opcode);
                      
    req_wb_info_next.xcpt_fetch   = (req_alu_valid) ? xcpt_fetch_in : '0;
    req_wb_info_next.xcpt_decode  = (req_alu_valid) ? xcpt_decode_in: '0;
    req_wb_info_next.xcpt_alu     = xcpt_alu;
    req_wb_info_next.xcpt_mul     = '0;
    req_wb_info_next.xcpt_cache   = '0;
end

/////////////////////////////////
// VERBOSE
`ifdef VERBOSE_ALU
always_ff @(posedge clock)
begin
    if (req_dcache_valid)
    begin
        $display("[ALU] Request to Cache. PC = %h",req_dcache_info.pc);
        $display("      addr             =  %h",req_dcache_info.addr);      
        $display("      size             =  %h",req_dcache_info.size) ;     
        $display("      is_store         =  %h",req_dcache_info.is_store);
        $display("      data             =  %h",req_dcache_info.data)  ;    
    end

    if (take_branch)
    begin
        $display("[ALU] Take branch. Current PC = %h",req_dcache_info.pc);
        $display("      Jump to  =  %h",branch_pc); 
    end
end
`endif
endmodule
