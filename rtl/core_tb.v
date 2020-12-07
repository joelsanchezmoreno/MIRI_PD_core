`include "soc.vh"

module core_tb(
    input   logic   clk_i,
    input   logic   reset_i
);


initial
begin
    $display("[CORE TB] PA core tb init");
end

//////////////////////////////////////////////////
// Interface signals with main memory

// Request from D$ to the memory hierarchy
logic                                   dcache_req_valid_miss;
memory_request_t                        dcache_req_info_miss;

// Request from I$ to the memory hierarchy
logic                                   icache_req_valid_miss;
memory_request_t                        icache_req_info_miss;

// Response from the memory hierarchy
logic [`DCACHE_LINE_WIDTH-1:0]          rsp_data_miss;
logic                                   rsp_valid_miss;
logic                                   rsp_cache_id;
logic                                   rsp_bus_error;
logic [`THR_PER_CORE_WIDTH-1:0]         rsp_thread_id;

//////////////////////////////////////////////////
// Core top instance
core_top
core_top
(
    // System signals
    .clock                  ( clk_i                 ),
    .reset                  ( reset_i               ),

    // Boot address
    .boot_addr              ( `CORE_BOOT_ADDRESS    ),

    // Request from I$ to the memory hierarchy
    .dcache_req_valid_miss  ( dcache_req_valid_miss ),
    .dcache_req_info_miss   ( dcache_req_info_miss  ),

    // Request from D$ to the memory hierarchy                                      
    .icache_req_valid_miss  ( icache_req_valid_miss ),
    .icache_req_info_miss   ( icache_req_info_miss  ),
                                      
    // Response from the memory hierarchy                                  
    .rsp_data_miss          ( rsp_data_miss         ),
    .rsp_thread_id          ( rsp_thread_id         ),
    .rsp_bus_error          ( rsp_bus_error         ),
    .rsp_valid_miss         ( rsp_valid_miss        ),
    .rsp_cache_id           ( rsp_cache_id          ) // 0 for I$, 1 for D$
);

//////////////////////////////////////////////////
// MAIN MEMORY

// FF to act as main memory
logic [`MAIN_MEMORY_LINE_RANGE] main_memory [`MAIN_MEMORY_DEPTH_RANGE];

// Request from core arbiter to MM
logic               req_mm_valid;
logic               req_mm_valid_ff;
memory_request_t    req_mm_info;
memory_request_t    req_mm_info_ff;
logic req_mm_info_is_dcache;
logic req_mm_info_is_dcache_ff;
logic order_fifo_pendant_request;
logic order_fifo_pendant_request_ff;

    //  CLK    RST    DOUT                           DIN                        DEF
`RST_FF(clk_i, rst_i, req_mm_valid_ff,               req_mm_valid,              1'b0)
`RST_FF(clk_i, rst_i, order_fifo_pendant_request_ff, order_fifo_pendant_request,1'b0)

//  CLK    DOUT                      DIN           
`FF(clk_i, req_mm_info_ff,           req_mm_info)
`FF(clk_i, req_mm_info_is_dcache_ff, req_mm_info_is_dcache)

// Response from MM to core arbiter
logic rsp_mm_valid;
logic rsp_mm_ready;
logic rsp_mm_bus_error;
logic [`ICACHE_LINE_WIDTH-1:0]  rsp_mm_data;

//////////////////////////////////////////////////
// FIFO - Arbiter logic
// 
// Each thread can have two requests in flight (Fetch and Cache)
// Two requests can be received at the same cycle (Fetch and Cache)

// Dcache FIFO
logic               push_dcache_fifo;
logic               pop_dcache_fifo;
logic               dcache_fifo_not_empty;
memory_request_t    dcache_pop_info;

fifo
#(
  .WIDTH ( $bits(dcache_req_info_miss)),
  .DEPTH ( `THR_PER_CORE              ) 
)
main_memory_dcache_fifo
(
    // System signals
    .clock      ( clock                     ),
    .reset      ( reset                     ),
    .full       (                           ) 

    // Push data
    .push       ( push_dcache_fifo          ),
    .wdata      ( dcache_req_info_miss      ),                                                   

    // Pop data
    .pop        ( pop_dcache_fifo           ),
    .rdata      ( dcache_pop_info           ),
    .valid      ( dcache_fifo_not_empty     )
);

// Icache FIFO
logic               push_icache_fifo;
logic               pop_icache_fifo;
logic               icache_fifo_not_empty;
memory_request_t    icache_pop_info;

fifo
#(
  .WIDTH ( $bits(icache_req_info_miss)),
  .DEPTH ( `THR_PER_CORE              ) 
)
main_memory_icache_fifo
(
    // System signals
    .clock      ( clock                     ),
    .reset      ( reset                     ),
    .full       (                           ) 

    // Push data
    .push       ( push_icache_fifo          ),
    .wdata      ( icache_req_info_miss      ),                                                   

    // Pop data
    .pop        ( pop_icache_fifo           ),
    .rdata      ( icache_pop_info           ),
    .valid      ( icache_fifo_not_empty     )
);

// Dcache-Icache request order FIFO
logic               pop_order_fifo;
logic [1:0]         order_pop_info;
logic               order_fifo_not_empty;
fifo
#(
  .WIDTH ( 2                ),
  .DEPTH ( `THR_PER_CORE*2  ) 
)
main_memory_order_fifo
(
    // System signals
    .clock      ( clock                     ),
    .reset      ( reset                     ),
    .full       (                           ) 

    // Push data
    .push       (  push_icache_fifo 
                 | push_dcache_fifo         ),
    .wdata      ( {push_dcache_fifo,
                   push_icache_fifo}        ), // D$ - b1 ; I$ - b0 
 
    // Pop data
    .pop        ( pop_order_fifo            ),
    .rdata      ( order_pop_info            ),
    .valid      ( order_fifo_not_empty      )
);


always_comb
begin
        // Response to core
    rsp_valid_miss  = 1'b0;
    rsp_bus_error   = rsp_mm_bus_error;

        // Push & Pop FIFOs
    push_dcache_fifo    = dcache_req_valid_miss;
    push_icache_fifo    = icache_req_valid_miss;
    pop_dcache_fifo     = 1'b0;
    pop_icache_fifo     = 1'b0;
    pop_order_fifo      = 1'b0;

        // Request to Main Memory
    req_mm_valid = req_mm_valid_ff;
    req_mm_info  = req_mm_info_ff;

        // Control signals
    req_mm_info_is_dcache       = req_mm_info_is_dcache_ff;
    order_fifo_pendant_request  = order_fifo_pendant_request_ff;

    // If there is a pendent request from D$ or I$ and Main Memory is ready.
    // Then, send the request to Main Memory
    if ( (dcache_fifo_not_empty | icache_fifo_not_empty) 
        & rsp_mm_ready )
    begin
        req_mm_valid = 1'b1;

            // Check if on the last order FIFO pop there were two requests to
            // be handled, and there is one that has not been done yet
        if (order_fifo_pendant_request_ff)
        begin
            // If there were two request, since we always prioritize the D$.
            // The missing one is the icache one
            pop_icache_fifo         = 1'b1;
            req_mm_info             = icache_pop_info;
            req_mm_info_is_dcache   = 1'b0;
        end
        else // No pendent requests
        begin
            pop_order_fifo  = 1'b1;
                // In case of two request at the same cycle, we prioritize the
                // data cache one and we mark that there is a pendent request
            pop_dcache_fifo = order_pop_info[1];
            pop_icache_fifo = !order_pop_info[1] & order_pop_info[0];
            order_fifo_pendant_request = order_pop_info[1] & order_pop_info[0];

            req_mm_info_is_dcache = order_pop_info[1];
            req_mm_info  = (pop_dcache_fifo) ? dcache_pop_info : 
                                               icache_pop_info ;
        end
    end

    if(rsp_mm_valid)
    begin
            // De-assert request to the MM
        req_mm_valid    = 1'b0;

            // Response to the core
        rsp_valid_miss  = 1'b1;
        rsp_thread_id   = req_mm_info_ff.thread_id;
        rsp_cache_id    = req_mm_info_is_dcache_ff; // 1 for D$ ; 0 for I$
        rsp_data_miss   = rsp_mm_data; 
    end

end

//////////////////////////////////////////////////
// Main memory

// Logic to emulate main memory latency
logic [`LATENCY_MM_RSP_RANGE] mem_rsp_count;

always_ff @(posedge clk_i) 
begin
    rsp_mm_valid     <= 1'b0;
    rsp_mm_ready     <= 1'b1;
    rsp_mm_bus_error <= 1'b0;

    if (reset_i)
    begin
        `ifdef MATRIX_MULTIPLY_TEST
    	    $readmemh("tests/matrix_multiply/verilator_MxM_src_code.hex", main_memory, `MM_BOOT_ADDR);
    	    $readmemh("tests/matrix_multiply/data_in_MxM_A.hex", main_memory, `MM_MATRIX_A_ADDR);
    	    $readmemh("tests/matrix_multiply/data_in_MxM_B.hex", main_memory, `MM_MATRIX_B_ADDR);
        `else
            `ifdef BUFFER_SUM_TEST
    	        $readmemh("tests/buffer_sum/buffer_sum.hex", main_memory, `MM_BOOT_ADDR);
    	        $readmemh("tests/buffer_sum/data_in_buffer_A.hex", main_memory, `MM_ARRAY_A_ADDR);
            `else
    	        $readmemh("data_input_file.hex", main_memory, `MM_BOOT_ADDR);
            `endif
        `endif
         $display("[CORE TB] Main memory loaded.");
         $display("           Source code. PC@'h1000 = %h",main_memory['h1000 >> `ICACHE_RSH_VAL]);
         $display("Exception handler code. PC@'h2000 = %h",main_memory['h2000 >> `ICACHE_RSH_VAL]);
        `ifdef MATRIX_MULTIPLY_TEST
            $display("              Matrix C. PC@'h3000  = %h",main_memory['h3000 >> `ICACHE_RSH_VAL]);
            $display("              Matrix A. PC@'h13000 = %h",main_memory['h13000 >> `ICACHE_RSH_VAL]);
            $display("              Matrix B. PC@'h23000 = %h",main_memory['h23000 >> `ICACHE_RSH_VAL]);
        `endif        
        $display("------------------------------------------");          
        $display("------------------------------------------");          
    end
    else if (req_mm_valid_ff)
    begin
        rsp_mm_ready  <= 1'b0;
        mem_rsp_count <= mem_rsp_count + 1'b1;

        if (mem_rsp_count == `LATENCY_MM_RSP-1)
        begin
            // Send response to the core arbiter
            rsp_mm_valid  <= 1'b1;
            rsp_mm_ready  <= 1'b1;

            if (req_mm_info_ff.addr >=  (`MAIN_MEMORY_DEPTH*`MAIN_MEMORY_LINE_SIZE) ) 
            begin
                rsp_mm_bus_error <= 1'b1;
            end
            else
            begin
                // Load
                if (!req_mm_info_ff.is_store)
                begin
                    `ifdef VERBOSE_CORETB_MM       
                        $display("[CORE TB] Main memory LD to address %h",req_mm_info_ff.addr );  
                    `endif             
                    rsp_mm_data <= main_memory[req_mm_info_ff.addr];
                end
                //Store
                else
                begin
                    `ifdef VERBOSE_CORETB_MM       
                        $display("[CORE TB] Main memory ST to address %h with data %h",req_mm_info_ff.addr,req_mm_info_ff.data);  
                    `endif   
        	        main_memory[req_mm_info_ff.addr] <= req_mm_info_ff.data;
                end
            end
            // Reset counter
            mem_rsp_count <= '0; 
        end
    end
end

`ifdef VERBOSE_CORETB 
integer out_file,iter_out;  

initial
begin
    `ifdef MATRIX_MULTIPLY_TEST
        out_file = $fopen("tests/matrix_multiply/verilator_matrix_C.hex","w");  
    `else
        out_file = $fopen("data_output_file.hex","w");  
    `endif
end

always_ff @(posedge clk_i) 
begin
    // If there is a request from the D$ and we are not busy sending the
    // response for the I$ we perform the D$ request
    if (dcache_req_valid_ff & !wait_rsp_icache_ff)
    begin
        if (rsp_mm_valid)
        begin
            `ifdef VERBOSE_CORETB_MM
            $display("[CORE TB] Response arbiter. Data to D$ %h",rsp_mm_data);     
            `endif            
        end
    end

    // If there is a request from the I$ and not from the D$ or we are 
    // already performing the I$ request we (continue) perform the I$ request
    if ((!dcache_req_valid_ff & icache_req_valid_ff) | wait_rsp_icache_ff)
    begin
        if (rsp_mm_valid)
        begin     
            `ifdef VERBOSE_CORETB_MM
            $display("[CORE TB] Response arbiter. Data to I$ %h",rsp_mm_data);  
            `endif 
        end
    end

    if ( rsp_mm_data == '1 & rsp_mm_valid & !dcache_req_valid_ff)
    begin
        $display("[CORE TB] Finishing simulation, we found all NOPs on memory");

        `ifndef MATRIX_MULTIPLY_TEST
            for (iter_out = `MM_BOOT_ADDR; iter_out < `MAIN_MEMORY_DEPTH; iter_out++)
                $fwrite(out_file,"%h\n", main_memory[iter_out]);
        `else
            for (iter_out = `MM_BOOT_ADDR; iter_out < `MAIN_MEMORY_DEPTH; iter_out++)
            //for (iter_out = `MATRIX_C_ADDR; iter_out < `MATRIX_A_ADDR; iter_out++)
                $fwrite(out_file,"%h\n", main_memory[iter_out]);
        `endif
        $fclose(out_file);

        $finish;
    end
end
`endif

endmodule

