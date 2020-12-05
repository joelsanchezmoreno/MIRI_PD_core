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
logic [`THR_PER_CORE_WIDTH-1:0]         dcache_req_thread_id;

// Request from I$ to the memory hierarchy
logic                                   icache_req_valid_miss;
memory_request_t                        icache_req_info_miss;
logic [`THR_PER_CORE_WIDTH-1:0]         icache_req_thread_id;

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
    .dcache_req_thread_id   ( dcache_req_thread_id  ),

    // Request from D$ to the memory hierarchy                                      
    .icache_req_valid_miss  ( icache_req_valid_miss ),
    .icache_req_info_miss   ( icache_req_info_miss  ),
    .icache_req_thread_id   ( icache_req_thread_id  ),
                                      
    // Response from the memory hierarchy                                  
    .rsp_data_miss          ( rsp_data_miss         ),
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
memory_request_t    req_mm_info;
memory_request_t    req_mm_info_ff;

//  CLK    DOUT            DIN           
`FF(clk_i, req_mm_info_ff, req_mm_info)

// Response from MM to core arbiter
logic rsp_mm_valid;
logic rsp_mm_bus_error;
logic [`ICACHE_LINE_WIDTH-1:0]  rsp_mm_data;

//////////////////////////////////////////////////
// Arbiter logic
//
// Arbiter between instruction cache and data cache. 
// D$ has always priority except if we are performing an instruction cache
// request

// Request from D$ to the memory hierarchy
logic                           dcache_req_valid_next;
logc                            dcache_req_valid_ff;
logic [`THR_PER_CORE_WIDTH-1:0] dcache_thread_id_ff; // thread id required???
memory_request_t                dcache_req_info_ff;  //TODO: Maybe storage has to bee an array for threads


// Request from I$ to the memory hierarchy
logic                           icache_req_valid_next;
logic                           icache_req_valid_ff;
logic [`THR_PER_CORE_WIDTH-1:0] icache_thread_id_ff; // thread id required???
memory_request_t                icache_req_info_ff; //TODO: Maybe storage has to bee an array for threads

//      CLK    RST      DOUT                 DIN           DEF
`RST_FF(clk_i, reset_i, dcache_req_valid_ff, dcache_req_valid_next, '0)
`RST_FF(clk_i, reset_i, icache_req_valid_ff, icache_req_valid_next, '0)

//         CLK    RST      EN                     DOUT                 DIN                   DEF
`RST_EN_FF(clk_i, reset_i, dcache_req_valid_miss, dcache_req_info_ff,  dcache_req_info_miss, '0)
`RST_EN_FF(clk_i, reset_i, dcache_req_valid_miss, dcache_thread_id_ff, dcache_thread_id,     '0)
`RST_EN_FF(clk_i, reset_i, icache_req_valid_miss, icache_req_info_ff,  icache_req_info_miss, '0)
`RST_EN_FF(clk_i, reset_i, icache_req_valid_miss, icache_thread_id_ff, icache_thread_id,     '0)

logic   wait_rsp_icache_next;
lgoci   wait_rsp_icache_ff ;
logic   wait_rsp_enable;
logic   wait_icache_rsp_update;

assign wait_rsp_enable = (!dcache_req_valid_miss & icache_req_valid_miss) | wait_icache_rsp_update;

//         CLK    RST      EN               DOUT                DIN                   DEF
`RST_EN_FF(clk_i, reset_i, wait_rsp_enable, wait_rsp_icache_ff, wait_rsp_icache_next, '0)

always_comb
begin
    rsp_valid_miss  = 1'b0;
    rsp_bus_error   = rsp_mm_bus_error;

        // Hold values for next cycles
    dcache_req_valid_next = dcache_req_valid_ff;
    icache_req_valid_next = icache_req_valid_ff;
    req_mm_info           = req_mm_info_ff;

    // We store that we have a pending request from D$
    if (dcache_req_valid_miss)
    begin
        dcache_req_valid_next   = 1'b1;
    end

    // We store that we have a pending request from I$    
    if (icache_req_valid_miss)
    begin
        icache_req_valid_next   = 1'b1;
    end

    // If there is a request from the D$ and we are not busy sending the
    // response for the I$ we perform the D$ request
    if (dcache_req_valid_ff & !wait_rsp_icache_ff)
    begin
        req_mm_valid = !rsp_mm_valid;
        req_mm_info  = dcache_req_info_ff;

        if(rsp_mm_valid)
        begin
            // De-assert request to the MM
            req_mm_valid    = 1'b0;

            // Response to the core
            rsp_valid_miss  = 1'b1;
            rsp_cache_id    = 1'b1; // response to D$
            rsp_data_miss   = rsp_mm_data; 

            // Reset control signal
            dcache_req_valid_next   = dcache_req_valid_miss;
        end
    end

    // If there is a request from the I$ and not from the D$ or we are 
    // already performing the I$ request we (continue) perform the I$ request
    if ((!dcache_req_valid_ff & icache_req_valid_ff) | wait_rsp_icache_ff)
    begin
        wait_rsp_icache_next = 1'b1;
        req_mm_valid = !rsp_mm_valid;
        req_mm_info  = icache_req_info_ff;    

        if(rsp_mm_valid)
        begin
            // De-assert request to the MM
            req_mm_valid    = 1'b0;

            // Response to the core                
            rsp_valid_miss  = 1'b1;
            rsp_cache_id    = 1'b0; // response to I$
            rsp_data_miss   = rsp_mm_data;
            
            // Reset control signal
            wait_rsp_icache_next    = 1'b0;
            wait_icache_rsp_update  = 1'b1; 
            icache_req_valid_next   = icache_req_valid_miss;
        end
    end
end

//////////////////////////////////////////////////
// Main memory

// Logic to emulate main memory latency
logic [`LATENCY_MM_RSP_RANGE] mem_rsp_count;

always_ff @(posedge clk_i) 
begin
    rsp_mm_valid     <= 1'b0;
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
    else if (req_mm_valid)
    begin
        mem_rsp_count <= mem_rsp_count + 1'b1;

        if (mem_rsp_count == `LATENCY_MM_RSP-1)
        begin
            // Send response to the core arbiter
            rsp_mm_valid  <= 1'b1;

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

