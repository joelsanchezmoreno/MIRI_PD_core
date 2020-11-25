`include "soc.vh"

module decode_top
(
    // System signals
    input   logic                               clock,
    input   logic                               reset,

    output  priv_mode_t [`THR_PER_CORE-1:0]     priv_mode,
    output  multithreading_mode_t               mt_mode,
    input   logic   [`THR_PER_CORE-1:0]         iret_instr,

    // Stall pipeline
    input   logic   [`THR_PER_CORE-1:0]         stall_decode,
    input   logic   [`THR_PER_CORE-1:0]         flush_decode,
    input   logic   [`THR_PER_CORE-1:0]         flush_rob,

    // Exceptions from fetch
    input   fetch_xcpt_t                        xcpt_fetch_in,

    // Fetched instruction
    input   logic                               fetch_instr_valid,
    input   logic   [`INSTR_WIDTH-1:0]          fetch_instr_data,
    input   logic   [`PC_WIDTH_RANGE]           fetch_instr_pc, 
    input   logic   [`THR_PER_CORE_WIDTH-1:0]   fetch_thread_id,

    // Instruction to ALU
    output  logic                               req_to_alu_valid,
    output  alu_request_t                       req_to_alu_info,
    output  logic   [`ROB_ID_RANGE]             req_to_alu_instr_id,
    output  logic   [`PC_WIDTH_RANGE]           req_to_alu_pc,
    output  logic   [`THR_PER_CORE_WIDTH-1:0]   req_to_alu_thread_id,
    output  fetch_xcpt_t                        alu_xcpt_fetch_out,
    output  decode_xcpt_t                       alu_decode_xcpt,

    // Instruction to MUL
    output  logic                               req_to_mul_valid,
    output  mul_request_t                       req_to_mul_info,
    output  logic   [`ROB_ID_RANGE]             req_to_mul_instr_id,
    output  logic   [`PC_WIDTH_RANGE]           req_to_mul_pc,
    output  logic   [`THR_PER_CORE_WIDTH-1:0]   req_to_mul_thread_id,
    output  fetch_xcpt_t                        mul_xcpt_fetch_out,
    output  decode_xcpt_t                       mul_decode_xcpt,

    // Write requests to the Register File
    input logic 				                writeEnRF,
    input logic [`REG_FILE_DATA_RANGE] 		    writeValRF,
    input logic [`REG_FILE_ADDR_RANGE] 		    destRF,
    input logic [`THR_PER_CORE_WIDTH-1:0]       write_thread_idRF,
    input logic [`ROB_ID_RANGE]                 write_rob_idRF,
    
    // Exceptions values to be stored on the RF
    input   logic 				                xcpt_valid,
    input   logic [`PC_WIDTH_RANGE] 		    rmPC,
    input   logic [`REG_FILE_XCPT_ADDR_RANGE]   rmAddr,
    input   xcpt_type_t                         xcpt_type,
    input   logic   [`THR_PER_CORE_WIDTH-1:0]   xcpt_thread_id
);
logic   [`THR_PER_CORE_WIDTH-1:0]   thread_id;
assign thread_id = fetch_thread_id;

logic [`THR_PER_CORE_WIDTH-1:0] previous_thread;

 // CLK    DOUT             DIN            
`FF(clock, previous_thread, thread_id)


// Control signals for requests to be sent to ALU
logic                                       req_to_alu_valid_next;
alu_request_t                               req_to_alu_info_next;
logic          [`THR_PER_CORE-1:0]          req_to_alu_valid_ff;
alu_request_t  [`THR_PER_CORE-1:0]          req_to_alu_info_ff;
logic [`THR_PER_CORE-1:0] [`PC_WIDTH-1:0]   req_to_alu_pc_ff;

logic   [`INSTR_OFFSET_LO_ADDR_RANGE]       fetch_instr_data_ff;

// Control signals for requests to be sent to MUL
logic                                       req_to_mul_valid_next;
mul_request_t                               req_to_mul_info_next;
logic           [`THR_PER_CORE-1:0]         req_to_mul_valid_ff;
mul_request_t   [`THR_PER_CORE-1:0]         req_to_mul_info_ff;
logic [`THR_PER_CORE-1:0][`PC_WIDTH-1:0]    req_to_mul_pc_ff;

/////////////////////////////////////////
// Decode fetch data
logic [`REG_FILE_ADDR_RANGE] rd_addr;
logic [`REG_FILE_ADDR_RANGE] ra_addr;
logic [`REG_FILE_ADDR_RANGE] rb_addr;
logic [`INSTR_OPCODE_RANGE]  opcode;
assign rd_addr = (req_to_alu_valid_ff[thread_id]) ? req_to_alu_info_ff[thread_id].rd_addr :
                 (req_to_mul_valid_ff[thread_id]) ? req_to_mul_info_ff[thread_id].rd_addr :                                                        
                                                    fetch_instr_data[`INSTR_DST_ADDR_RANGE];

assign opcode  = (req_to_alu_valid_ff[thread_id]) ? req_to_alu_info_ff[thread_id].opcode :
                 (req_to_mul_valid_ff[thread_id]) ? req_to_mul_info_ff[thread_id].opcode :                                                        
                                                    fetch_instr_data[`INSTR_OPCODE_ADDR_RANGE];

assign ra_addr = (req_to_alu_valid_ff[thread_id]) ? req_to_alu_info_ff[thread_id].ra_addr :
                 (req_to_mul_valid_ff[thread_id]) ? req_to_mul_info_ff[thread_id].ra_addr :                                                        
                                                    fetch_instr_data[`INSTR_SRC1_ADDR_RANGE];

assign rb_addr = (req_to_alu_valid_ff[thread_id]) ? req_to_alu_info_ff[thread_id].rb_addr :
                 (req_to_mul_valid_ff[thread_id]) ? req_to_mul_info_ff[thread_id].rb_addr :                                                        
                                                    fetch_instr_data[`INSTR_SRC2_ADDR_RANGE];

// Compute if is mul instruction
logic                     mul_instr;
logic [`THR_PER_CORE-1:0] mul_instr_ff;
assign  mul_instr = fetch_instr_valid & is_mul_instr(opcode);

/////////////////////////////////////////
// Exceptions
decode_xcpt_t                      decode_xcpt_next;
decode_xcpt_t [`THR_PER_CORE-1:0]  decode_xcpt_ff;
fetch_xcpt_t  [`THR_PER_CORE-1:0]  xcpt_fetch_ff;

assign alu_decode_xcpt      = (flush_decode[previous_thread]   ) ? '0 : // Cancel xcpt propagation if there's a flush 
                              (mul_instr_ff                    ) ? '0 :
                              (stall_decode_ff[previous_thread]) ? '0 : // Cancel xcpt prop if there was a stall last cycle
                                                                    decode_xcpt_ff[previous_thread];

assign alu_xcpt_fetch_out   = (flush_decode[previous_thread]   ) ? '0 : // Cancel xcpt propagation if there's a flush
                              (mul_instr_ff                    ) ? '0 :
                              (stall_decode_ff[previous_thread]) ? '0 : // Cancel xcpt prop if there was a stall last cycle
                                                                    xcpt_fetch_ff[previous_thread];

assign mul_decode_xcpt      = (flush_decode[previous_thread]   ) ? '0 : // Cancel xcpt propagation if there's a flush 
                              (!mul_instr_ff                   ) ? '0 :
                              (stall_decode_ff[previous_thread]) ? '0 : // Cancel xcpt prop if there was a stall last cycle
                                                                    decode_xcpt_ff[previous_thread];

assign mul_xcpt_fetch_out   = (flush_decode[previous_thread]   ) ? '0 : // Cancel xcpt propagation if there's a flush 
                              (!mul_instr_ff                   ) ? '0 :
                              (stall_decode_ff[previous_thread]) ? '0 : // Cancel xcpt prop if there was a stall last cycle
                                                                    xcpt_fetch_ff[previous_thread];

genvar jj;
generate for (jj=0; jj < `THR_PER_CORE; jj++) 
begin
    logic update_ff;
    assign update_ff = fetch_instr_valid & (jj == thread_id);

        //     CLK    RST                       EN         DOUT                DIN               DEF
    `RST_EN_FF(clock, reset | flush_decode[jj], update_ff, decode_xcpt_ff[jj], decode_xcpt_next, '0)
    `RST_EN_FF(clock, reset | flush_decode[jj], update_ff, xcpt_fetch_ff[jj],  xcpt_fetch_in,    '0)
end
endgenerate

/////////////////////////////////////////
// Control logic for requests to be sent to ALU

assign req_to_alu_valid_next =  ( flush_decode[thread_id]       ) ? 1'b0       : // Invalidate instruction
                                ( stall_decode[thread_id]       ) ? 1'b0       :
                                ( fetch_instr_valid             ) ? !mul_instr : // New instruction from fetch and not MUL
                                ( req_to_alu_valid_ff[thread_id]) ? !mul_instr_ff[thread_id] : // New instruction from fetch and not MUL
                                (  xcpt_fetch.xcpt_itlb_miss
                                 | xcpt_fetch.xcpt_bus_error
                                 | decode_xcpt_next.xcpt_illegal_instr) ? 1'b1: // There has been an xcpt
                                                                          1'b0;

assign req_to_alu_valid     = (flush_decode[previous_thread]) ? 1'b0 : req_to_alu_valid_ff[previous_thread];
assign req_to_alu_info      = req_to_alu_info_ff[previous_thread];
assign req_to_alu_pc        = req_to_alu_pc_ff[previous_thread];
assign req_to_alu_thread_id = previous_thread;

logic [`THR_PER_CORE-1:0] update_info;
genvar pp;
generate for (pp=0; pp < `THR_PER_CORE; pp++) 
begin
    logic update_ff;
    assign update_ff = fetch_instr_valid & (pp == thread_id);

    logic update_info;
    assign update_info = (pp == thread_id);
    
        //     CLK    RST                       EN                             DOUT                     DIN                    DEF
    `RST_EN_FF(clock, reset | flush_decode[pp], update_ff | !stall_decode[pp], req_to_alu_valid_ff[pp], req_to_alu_valid_next, '0)
    
        // CLK    EN         DOUT                  DIN            
    `EN_FF(clock, update_ff, req_to_alu_pc_ff[pp], fetch_instr_pc)
   
    // INFO must be updated at all cycles given that RF could have been
    // written while other thread was active and we were stalled. So we need
    // to ensure that not only RoB is updated but also that ra_data and
    // rb_data are properly updated 
        // CLK    EN           DOUT                    DIN                 
    `EN_FF(clock, update_info, req_to_alu_info_ff[pp], req_to_alu_info_next)

        // CLK    EN         DOUT                 DIN            
    `EN_FF(clock, update_ff, mul_instr_ff,        mul_instr)
    `EN_FF(clock, update_ff, fetch_instr_data_ff, fetch_instr_data[`INSTR_OFFSET_LO_ADDR_RANGE])
end
endgenerate

/////////////////////////////////////////
// Control logic for requests to be sent to MUL

assign req_to_mul_valid_next =  ( flush_decode[thread_id]       ) ? 1'b0         : // Invalidate instruction
                                ( stall_decode[thread_id]       ) ? 1'b0         : // Stall the pipeline 
                                ( fetch_instr_valid             ) ? mul_instr    : // New instruction from fetch
                                ( req_to_mul_valid_ff[thread_id]) ? mul_instr_ff[thread_id] : // New instruction from fetch and not MUL
                                                                    1'b0;

assign req_to_mul_valid     = (flush_decode[previous_thread]) ? 1'b0 : req_to_mul_valid_ff[previous_thread];
assign req_to_mul_info      = req_to_mul_info_ff[previous_thread];
assign req_to_mul_pc        = req_to_mul_pc_ff[previous_thread];
assign req_to_mul_thread_id = previous_thread;

genvar kk;
generate for (kk=0; kk < `THR_PER_CORE; kk++) 
begin
    logic update_ff;
    assign update_ff = fetch_instr_valid & (kk == thread_id); 
 
    logic update_info;
    assign update_info = (pp == thread_id);

        //     CLK    RST                       EN                             DOUT                     DIN                   DEF
    `RST_EN_FF(clock, reset | flush_decode[kk], update_ff | !stall_decode[kk], req_to_mul_valid_ff[kk], req_to_mul_valid_next, '0)

        // CLK    EN         DOUT                  DIN                  
    `EN_FF(clock, update_ff, req_to_mul_pc_ff[kk], fetch_instr_pc)
    
        // CLK    EN          DOUT                    DIN                 
    `EN_FF(clock,update_info, req_to_mul_info_ff[kk], req_to_mul_info_next)
end
endgenerate

/////////////////////////////////////////
// Register file signals      
logic [`THR_PER_CORE-1:0][`REG_FILE_DATA_RANGE] rf_reg1_data; 
logic [`THR_PER_CORE-1:0][`REG_FILE_DATA_RANGE] rf_reg2_data; 
logic [`THR_PER_CORE-1:0][`REG_FILE_DATA_RANGE] rm0_data;
logic [`THR_PER_CORE-1:0][`REG_FILE_DATA_RANGE] rm1_data;
logic [`THR_PER_CORE-1:0][`REG_FILE_DATA_RANGE] rm2_data;

/////////////////////////////////////////
// Bypass control signals

logic [`THR_PER_CORE-1:0] stall_decode_ff;

// FF to store the instr. ID that blocks each register
logic [`THR_PER_CORE-1:0][`REG_FILE_NUM_REGS_RANGE][`ROB_NUM_ENTRIES_W_RANGE] reg_rob_id_next;
logic [`THR_PER_CORE-1:0][`REG_FILE_NUM_REGS_RANGE][`ROB_NUM_ENTRIES_W_RANGE] reg_rob_id_ff;


// Valid bit for each register that is asserted if we are waiting for a instr.
// to finish before performing the operation
logic [`THR_PER_CORE-1:0][`REG_FILE_NUM_REGS_RANGE] reg_blocked_valid_next;
logic [`THR_PER_CORE-1:0][`REG_FILE_NUM_REGS_RANGE] reg_blocked_valid_ff;

logic   [`ROB_ID_RANGE]         ticket_src1;    // instr. that is blocking src1
logic                           rob_blocks_src1;// Asserted if there is an instr. blocking src1
logic   [`ROB_ID_RANGE]         ticket_src2;    // instr. that is blocking src2
logic                           rob_blocks_src2;// Asserted if there is an instr. blocking src2

logic   [`THR_PER_CORE-1:0][`ROB_NUM_ENTRIES_W_RANGE] reorder_buffer_tail_next;
logic   [`THR_PER_CORE-1:0][`ROB_NUM_ENTRIES_W_RANGE] reorder_buffer_tail_ff;

// Needed in case ALU forces to take a branch, so we have to restore the
// value we had, instead of taking into account the current instruction
logic [`THR_PER_CORE-1:0][`REG_FILE_NUM_REGS_RANGE] reg_blocked_valid_ff_2;
logic [`THR_PER_CORE-1:0][`REG_FILE_NUM_REGS_RANGE][`ROB_NUM_ENTRIES_W_RANGE] reg_rob_id_ff_2;

// Needed in case we receive a RF write request while restoring status
logic [`THR_PER_CORE-1:0][`REG_FILE_NUM_REGS_RANGE] reg_blocked_valid_next_2;
logic [`THR_PER_CORE-1:0][`REG_FILE_NUM_REGS_RANGE][`ROB_NUM_ENTRIES_W_RANGE] reg_rob_id_next_2;


logic [`THR_PER_CORE-1:0] flush_decode_ff;
genvar ii;
generate for (ii=0; ii < `THR_PER_CORE; ii++) 
begin
    logic thread_is_active;
    assign thread_is_active = (thread_id == ii);

    logic update_en;
    assign update_en = !stall_decode[ii] & thread_is_active;

        //  CLK    RST                    DOUT                 DIN              DEF
    `RST_FF(clock, reset | flush_rob[ii], stall_decode_ff[ii], stall_decode[ii], '0)

        // CLK    EN         DOUT               DIN   
    `EN_FF(clock, update_en, reg_rob_id_ff[ii], (flush_decode_ff[ii]) ? reg_rob_id_next_2[ii] : reg_rob_id_next[ii])

        //  CLK    RST                    DOUT                      DIN                                                                                DEF
    `RST_FF(clock, reset | flush_rob[ii], reg_blocked_valid_ff[ii], (flush_decode_ff[ii]) ? reg_blocked_valid_next_2[ii] : reg_blocked_valid_next[ii], '0)
   
    
        //     CLK    RST                    EN         DOUT                        DIN                           DEF
    `RST_EN_FF(clock, reset | flush_rob[ii], update_en, reorder_buffer_tail_ff[ii], reorder_buffer_tail_next[ii], '0)

        //  CLK    RST                    DOUT                        DIN                           DEF
    `RST_FF(clock, reset | flush_rob[ii], reg_blocked_valid_ff_2[ii], reg_blocked_valid_next_2[ii], '0)

        //     CLK    RST                    DOUT                            DIN                    DEF
    `RST_EN_FF(clock, reset | flush_rob[ii], update_en, reg_rob_id_ff_2[ii], reg_rob_id_next_2[ii], '0)
    
        //  CLK    RST                    DOUT                 DIN              DEF
    `RST_FF(clock, reset | flush_rob[ii], flush_decode_ff[ii], flush_decode[ii], '0)
end
endgenerate


///////////////////////
// Manage RoB tickets and blocker
always_comb
begin
    // Mantain FF values 
    reg_blocked_valid_next_2 = reg_blocked_valid_ff_2;
    reg_rob_id_next_2        = reg_rob_id_ff_2;

    // Mantain values from previous cycle by default
    reg_blocked_valid_next     = reg_blocked_valid_ff;
    reg_rob_id_next            = reg_rob_id_ff;
    reorder_buffer_tail_next   = reorder_buffer_tail_ff;

   // Maintain values from 2 cycles ago for restore purposes
    reg_blocked_valid_next_2[thread_id]   = (  flush_decode[thread_id] 
                                             | flush_decode_ff[thread_id]) ? reg_blocked_valid_ff_2[thread_id] : 
                                                                             reg_blocked_valid_ff[thread_id];

    reg_rob_id_next_2[thread_id]          = (  flush_decode[thread_id] 
                                             | flush_decode_ff[thread_id]) ? reg_rob_id_ff_2[thread_id] : 
                                                                             reg_rob_id_ff[thread_id];


    rob_blocks_src1 = 1'b0;
    rob_blocks_src2 = 1'b0;

    // Send blocker information to ALU/MUL stages
    // Check if the instruction makes use of source1 register
    if (  is_r_type_instr(opcode) | is_mul_instr(opcode) 
        | is_m_type_instr(opcode) | is_branch_type_instr(opcode) 
        | is_tlb_instr(opcode))
    begin
        // Check if this register is protected because was the destination
        // of a previous instruction. If that is the case we also have to
        // check if the instruction being written this cycle corresponds
        // to the blocker instruction that was protecting the register.
        if (reg_blocked_valid_ff[thread_id][ra_addr]) //check if this register is protected
        begin
            // Check if we the register is being written this same cycle
            if(  writeEnRF
               & (write_thread_idRF == thread_id)
               & (reg_rob_id_ff[thread_id][ra_addr] == write_rob_idRF))
                rob_blocks_src1 = 1'b0;
            else
                // Otherwise, send to ALU/MUL that this register still
                // protected
                rob_blocks_src1 = 1'b1;
        end
       ticket_src1     = reg_rob_id_ff[thread_id][ra_addr];
    end  
    // Check if the instruction makes use of source2 register
    if (  is_r_type_instr(opcode)| is_mul_instr(opcode) 
        | is_store_instr(opcode) | is_branch_type_instr(opcode) 
        | is_tlb_instr(opcode))
    begin
        // if store instr we need to check rd instead of rb
        if (is_store_instr(opcode))
        begin
            ticket_src2 = reg_rob_id_ff[thread_id][rd_addr];
            if (reg_blocked_valid_ff[thread_id][rd_addr]) //check if this register is protected
            begin
                // check if the value written to RF corresponds to the
                // blocker instr
                if(  writeEnRF 
                   & (write_thread_idRF == thread_id)
                   & (reg_rob_id_ff[thread_id][rd_addr] == write_rob_idRF))
                    rob_blocks_src2 = 1'b0;
                else
                    // Otherwise, send to ALU/MUL that this register still
                    // protected
                    rob_blocks_src2 = 1'b1;
            end
        end
        else // if not store instruction we check rb instead of rd
        begin
            ticket_src2 = reg_rob_id_ff[thread_id][rb_addr];
            if (  !is_addi_type_instr(opcode) 
                & reg_blocked_valid_ff[thread_id][rb_addr]) //check if this register is protected
            begin
                // check if the value written to RF corresponds to the
                // blocker instr
                if(  writeEnRF 
                   & (write_thread_idRF == thread_id)
                   & (reg_rob_id_ff[thread_id][rb_addr] == write_rob_idRF))
                    rob_blocks_src2 = 1'b0;
                else
                    // Otherwise, send to ALU/MUL that this register still
                    // protected
                    rob_blocks_src2 = 1'b1;
            end
        end
    end          
   
    if (!flush_decode[thread_id] & !stall_decode[thread_id])
        reorder_buffer_tail_next[thread_id] = reorder_buffer_tail_ff[thread_id] + 1'b1;

    ////////////////////////////////////////
    // Update blocker arrays if needed
    if (fetch_instr_valid)
    begin
        // Check if the instruction generates a result
        if (  is_r_type_instr(opcode) | is_mul_instr(opcode) 
            | is_load_instr(opcode)   | is_mov_instr(opcode))
        begin
            reg_blocked_valid_next[thread_id][rd_addr]  = 1'b1;
            reg_rob_id_next[thread_id][rd_addr]         = reorder_buffer_tail_next[thread_id];
        end
    end

    // Check if some register is going to be written and if
    // the the thread that is targetted had a blocker under the destination
    // register with the instr ID (rob_id). If it is the case, then the thread
    // is no longer blocked by this instr, and the next time is active it can
    // continue
    if (  writeEnRF 
        & (reg_rob_id_ff[write_thread_idRF][destRF] == write_rob_idRF))
    begin
        reg_blocked_valid_next[write_thread_idRF][destRF] = 1'b0;
    end

    if (  writeEnRF 
        & (reg_rob_id_ff_2[write_thread_idRF][destRF] == write_rob_idRF))
    begin
        reg_blocked_valid_next_2[write_thread_idRF][destRF] = 1'b0;        
    end
end

/////////////////////////////////////////
// Decode data
logic [`REG_FILE_DATA_RANGE] ra_data;
logic [`REG_FILE_DATA_RANGE] rb_data;   

integer it;
always_comb	
begin
    // Exception
    decode_xcpt_next.xcpt_illegal_instr = 1'b0;
    decode_xcpt_next.xcpt_pc = fetch_instr_pc;

    // Opcode and destination register are always decoded from the instruction
    // provided by the fetch stage
        // ALU
    req_to_alu_instr_id                             = reorder_buffer_tail_ff[thread_id];
    req_to_alu_info_next[thread_id].opcode          = (req_to_alu_valid_ff[thread_id]) ? req_to_alu_info_ff[thread_id].opcode :
                                                                                         opcode;
    req_to_alu_info_next[thread_id].rd_addr         = (req_to_alu_valid_ff[thread_id]) ? req_to_alu_info_ff[thread_id].rd_addr :
                                                                                         rd_addr;
    req_to_alu_info_next[thread_id].ra_addr         = (req_to_alu_valid_ff[thread_id]) ? req_to_alu_info_ff[thread_id].ra_addr :
                                                                                         ra_addr;
    req_to_alu_info_next[thread_id].rb_addr         = (req_to_alu_valid_ff[thread_id]) ? req_to_alu_info_ff[thread_id].rb_addr :
                                                                                         rb_addr;  
    req_to_alu_info_next[thread_id].ticket_src1     = ticket_src1;     
    req_to_alu_info_next[thread_id].rob_blocks_src1 = rob_blocks_src1;  
    req_to_alu_info_next[thread_id].ticket_src2     = ticket_src2;     
    req_to_alu_info_next[thread_id].rob_blocks_src2 = rob_blocks_src2;  

        // MUL                                                                              
    req_to_mul_instr_id                  = reorder_buffer_tail_ff[thread_id];
    req_to_mul_info_next.rd_addr         = (req_to_mul_valid_ff[thread_id]) ? req_to_mul_info_ff[thread_id].rd_addr :
                                                                              rd_addr;
    req_to_mul_info_next.ra_addr         = (req_to_mul_valid_ff[thread_id]) ? req_to_mul_info_ff[thread_id].ra_addr :
                                                                              ra_addr;
    req_to_mul_info_next.rb_addr         = (req_to_mul_valid_ff[thread_id]) ? req_to_mul_info_ff[thread_id].rb_addr :
                                                                              rb_addr;
    req_to_mul_info_next.ticket_src1     = ticket_src1;     
    req_to_mul_info_next.rob_blocks_src1 = rob_blocks_src1;  
    req_to_mul_info_next.ticket_src2     = ticket_src2;     
    req_to_mul_info_next.rob_blocks_src2 = rob_blocks_src2; 

    // Register A value depends on the instructions being performed at the
    // mul, alu and cache stage at this cycle, because we may need to bypass data
    ra_data = (  writeEnRF 
               & (destRF == ra_addr) 
               & (write_thread_idRF == thread_id)) ? writeValRF    : // intercept write to RF
	   		      				                     rf_reg1_data[thread_id]; // data from register file

    // Register B value depends on the instructions being performed at the
    // mul, alu and cache stage at this cycle, because we may need to bypass data
    if (is_store_instr(opcode) & reg_blocked_valid_ff[rd_addr])
    begin
        rb_data = (  writeEnRF
                   & (destRF == rd_addr) 
                   & (write_thread_idRF == thread_id)) ? writeValRF    : // intercept write to RF
                                                         rf_reg2_data[thread_id]; // data from register file
    end
    else
        rb_data = (  writeEnRF
                   & (destRF == rb_addr) 
                   & (write_thread_idRF == thread_id)) ? writeValRF    : // intercept write to RF
                                                         rf_reg2_data[thread_id]; // data from register file
     

                                                                                                         
    // Data for MUL and ALU
    req_to_alu_info_next[thread_id].ra_data = ra_data;
    req_to_alu_info_next[thread_id].rb_data = rb_data;
    req_to_mul_info_next.ra_data = ra_data;
    req_to_mul_info_next.rb_data = rb_data;
     
    // Encoding for ADDI and M-type instructions                                                                                                                     
    req_to_alu_info_next[thread_id].offset = (req_to_alu_valid_ff[thread_id]) ? req_to_alu_info_ff[thread_id].offset :
                                                                                `ZX(`ALU_OFFSET_WIDTH,fetch_instr_data[14:0]);

    ////////////////////////////////////////////////
    // Decode instruction to determine RB or offset 
    
    // B-format
    if (  is_branch_type_instr(opcode) | is_jump_instr(opcode) | is_tlb_instr(opcode) )  
    begin
    
        if ( is_branch_type_instr(opcode))// BEQ,BNE, BLT, BGT, BLE, BGE CASE
        begin
            req_to_alu_info_next[thread_id].offset = (req_to_alu_valid_ff[thread_id]) ? req_to_alu_info_ff[thread_id].offset :
                                                                                        `ZX(`ALU_OFFSET_WIDTH, {fetch_instr_data[`INSTR_OFFSET_HI_ADDR_RANGE], 
                                                                                                                fetch_instr_data[`INSTR_OFFSET_LO_ADDR_RANGE]}); 
        end
        else if (is_jump_instr(opcode))  // JUMP CASE
        begin
            req_to_alu_info_next[thread_id].offset = (req_to_alu_valid_ff[thread_id]) ? req_to_alu_info_ff[thread_id].offset :
                                                                                        `ZX(`ALU_OFFSET_WIDTH, {fetch_instr_data[`INSTR_OFFSET_HI_ADDR_RANGE], 
                                                                                                                fetch_instr_data[`INSTR_OFFSET_M_ADDR_RANGE], 
                                                                                                                fetch_instr_data[`INSTR_OFFSET_LO_ADDR_RANGE]}); 
        end
        else //TLBWRITE
        begin
            req_to_alu_info_next[thread_id].offset = (req_to_alu_valid_ff[thread_id]) ? req_to_alu_info_ff[thread_id].offset :
                                                                                        `ZX(`ALU_OFFSET_WIDTH, fetch_instr_data[`INSTR_OFFSET_LO_ADDR_RANGE]);
            decode_xcpt_next.xcpt_illegal_instr = (priv_mode[thread_id] == User) ?  fetch_instr_valid
                                                                                  & !flush_decode[thread_id] : 
                                                                                  1'b0;
        end
    end
    else
    begin
        // MOV
        if (is_mov_instr(opcode))
        begin
            // rm1 : @ fault
            req_to_alu_info_next[thread_id].ra_data = rm1_data[thread_id]; 
            if (req_to_alu_valid_ff[thread_id])
            begin
                // rm0 : xcpt PC
                if (fetch_instr_data_ff == 9'h001)
                    req_to_alu_info_next[thread_id].ra_data = rm0_data[thread_id];
                // rm2 : xcpt type
                else if(fetch_instr_data_ff == 9'h002)
                    req_to_alu_info_next[thread_id].ra_data = rm2_data[thread_id];
            end
            else
                // rm0 : xcpt PC
                if (fetch_instr_data[`INSTR_OFFSET_LO_ADDR_RANGE] == 9'h001)
                    req_to_alu_info_next[thread_id].ra_data = rm0_data[thread_id];
                // rm2 : xcpt type
                else if (fetch_instr_data[`INSTR_OFFSET_LO_ADDR_RANGE] == 9'h002)
                    req_to_alu_info_next[thread_id].ra_data = rm2_data[thread_id];
        end
        // IRET
        else if (is_iret_instr(opcode))
        begin
            req_to_alu_info_next[thread_id].ra_data = rm0_data[thread_id];
        end

        // Raise an exception because the instruction is not supported
        else if (  !is_r_type_instr(opcode) & !is_mul_instr(opcode)
                 & !is_m_type_instr(opcode) & !is_nop_instr(opcode))
        begin
            decode_xcpt_next.xcpt_illegal_instr = fetch_instr_valid & !flush_decode[thread_id];
        end                  
    end
end

////////////////////////////////
// Multi-threading mode
multithreading_mode_t   mt_mode_next;
logic                   req_mt_mode;

assign req_mt_mode  = (writeEnRF && (destRF == `MT_MODE_REG_ADDR)) ? 1'b1 : 1'b0;
assign mt_mode_next = (req_mt_mode) ? writeValRF : mt_mode;

    //     CLK    RST    EN           DOUT     DIN           DEF
`RST_EN_FF(clock, reset, req_mt_mode, mt_mode, mt_mode_next, '0) // Single-threaded by default

////////////////////////////////
// Register File
    
logic [`REG_FILE_ADDR_RANGE]   src1_addr; 
logic [`REG_FILE_ADDR_RANGE]   src2_addr;
logic [`REG_FILE_ADDR_RANGE]   dest_addr;

assign src1_addr =  (req_to_alu_valid_ff[thread_id]) ? req_to_alu_info_ff[thread_id].rd_addr :
                    (req_to_mul_valid_ff[thread_id]) ? req_to_mul_info_ff[thread_id].rd_addr : 
                                                       fetch_instr_data[`INSTR_SRC1_ADDR_RANGE];

assign src2_addr = (req_to_alu_valid_ff[thread_id]) ? is_store_instr(opcode) ? req_to_alu_info_ff[thread_id].rd_addr :
                                                                               req_to_alu_info_ff[thread_id].rb_addr :
                   (req_to_mul_valid_ff[thread_id]) ? is_store_instr(opcode) ? req_to_mul_info_ff[thread_id].rd_addr : 
                                                                               req_to_mul_info_ff[thread_id].rb_addr :
                   (is_store_instr(opcode)        ) ? fetch_instr_data[`INSTR_DST_ADDR_RANGE]:
                                                      fetch_instr_data[`INSTR_SRC2_ADDR_RANGE];

assign dest_addr = destRF;

genvar ll;
generate for (ll=0; ll < `THR_PER_CORE; ll++) 
begin
    logic  writeEnRF_aux;
    assign writeEnRF_aux = writeEnRF & !req_mt_mode & (ll == write_RF_thread_id);

    logic xcpt_valid_aux;
    assign xcpt_valid_aux = xcpt_valid & (ll == xcpt_thread_id) ;
    regFile 
    registerFile
    (
        // System signals
        .clock      ( clock             ),
        .reset      ( reset             ),
    
        // Internal register values
        .iret_instr ( iret_instr[ll]    ),
        .priv_mode  ( priv_mode[ll]     ),
        .rm0_data   ( rm0_data[ll]      ),
        .rm1_data   ( rm1_data[ll]      ),
        .rm2_data   ( rm2_data[ll]      ),
    
        // Read port
        .src1_addr  ( src1_addr         ),
        .src2_addr  ( src2_addr         ),
        .reg1_data  ( rf_reg1_data[ll]  ),
        .reg2_data  ( rf_reg2_data[ll]  ),
    
        // Write port
        .writeEn    ( writeEnRF_aux     ),
        .dest_addr  ( dest_addr         ),
        .writeVal   ( writeValRF        ),
    
        // Exception input
        .xcpt_valid ( xcpt_valid_aux    ),
        .rmPC	    ( rmPC              ),
        .rmAddr     ( rmAddr            ),
        .xcpt_type  ( xcpt_type         )
    );
end
endgenerate


`ifdef VERBOSE_DECODE
always_ff @(posedge clock)
begin
    if (req_to_alu_valid)
    begin
        $display("[DECODE] Request to ALU. PC = %h",req_to_alu_pc);
        $display("         opcode  =  %h",req_to_alu_info.opcode);
        $display("         rd addr =  %h",req_to_alu_info.rd_addr);
        $display("         ra addr =  %h",req_to_alu_info.ra_addr);
        $display("         ra data =  %h",req_to_alu_info.ra_data);
        $display("         rb data =  %h",req_to_alu_info.rb_data);
        $display("         offset  =  %h",req_to_alu_info.offset );
        $display("[RF]     src1_addr    = %h",src1_addr);
        $display("         src2_addr    = %h",src2_addr);
        $display("         rf_reg1_data = %h",rf_reg1_data[thread_id]);
        $display("         rf_reg2_data = %h",rf_reg2_data[thread_id]);
     `ifdef VERBOSE_DECODE_BYPASS
        $display("[BYPASSES SRC1]");
        $display("         fetch_instr_data[`INSTR_SRC1_ADDR_RANGE]= %h",fetch_instr_data[`INSTR_SRC1_ADDR_RANGE]);
        $display("         ----------------------");
        $display("         fetch_instr_data[`INSTR_SRC1_ADDR_RANGE]= %h",fetch_instr_data[`INSTR_SRC1_ADDR_RANGE]);
        $display("         ----------------------");
        $display("         ----------------------");
        $display("[BYPASSES SRC2]");
        $display("         fetch_instr_data[`INSTR_SRC2_ADDR_RANGE]= %h",fetch_instr_data[`INSTR_SRC2_ADDR_RANGE]);
        $display("         ----------------------");
        $display("         fetch_instr_data[`INSTR_SRC2_ADDR_RANGE]= %h",fetch_instr_data[`INSTR_SRC2_ADDR_RANGE]);
     `endif
    end
end
`endif

endmodule

