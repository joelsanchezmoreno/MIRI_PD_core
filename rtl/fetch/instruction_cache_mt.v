// Instruction Cache does not allow write requests from the fetch logic, only
// reads are allowed. The Instruction cache implementation must
// ensure that one request can be served each cycle if there are no misses.
// Otheriwse, in case of a miss it takes `MAIN_MEMORY_LATENCY cycles to go to 
// memory and bring the line. 
`include "soc.vh"

module instruction_cache_mt
(
    input  logic                            clock,
    input  logic                            reset,
    output logic  [`THR_PER_CORE-1:0]       icache_ready, 
    output logic                            xcpt_bus_error, 
    input  multithreading_mode_t            mt_mode,

    // Request from the core pipeline
    input  logic [`ICACHE_ADDR_WIDTH-1:0]   req_addr,
    input  logic                            req_valid,
    input  logic [`THR_PER_CORE_WIDTH-1:0]  req_thread_id,

    // Response to the core pipeline
    output logic [`ICACHE_LINE_WIDTH-1:0]   rsp_data,
    output logic                            rsp_valid,

    // Request to the memory hierarchy
    output logic                            req_valid_miss,
    output memory_request_t                 req_info_miss,

    // Response from the memory hierarchy
    input  logic [`ICACHE_LINE_WIDTH-1:0]   rsp_data_miss,
    input  logic                            rsp_bus_error,
    input  logic [`THR_PER_CORE_WIDTH-1:0]  rsp_thread_id,
    input  logic                            rsp_valid_miss
);

//////////////////////////////////////////////////
// Instruction Cache arrays: tag, data and valid
logic [`ICACHE_LINE_WIDTH-1:0] instMem_data     [`ICACHE_NUM_WAYS-1:0];
logic [`ICACHE_LINE_WIDTH-1:0] instMem_data_ff  [`ICACHE_NUM_WAYS-1:0];
logic [`ICACHE_TAG_RANGE]      instMem_tag      [`ICACHE_NUM_WAYS-1:0];
logic [`ICACHE_TAG_RANGE]      instMem_tag_ff   [`ICACHE_NUM_WAYS-1:0];
logic [`ICACHE_NUM_WAYS-1:0]   instMem_valid;
logic [`ICACHE_NUM_WAYS-1:0]   instMem_valid_ff;

//  CLK        DOUT         DIN         
`FF(clock, instMem_data_ff, instMem_data)
`FF(clock, instMem_tag_ff , instMem_tag )

//      CLK    RST    DOUT               DIN           DEF
`RST_FF(clock, reset, instMem_valid_ff, instMem_valid, '0)

//////////////////////////////////////////////////
// Control signals 
logic icache_hit;
logic [`ICACHE_WAYS_PER_SET_RANGE]  hit_way; 
logic [`ICACHE_TAG_RANGE]           req_addr_tag;
logic [`ICACHE_NUM_WAY_RANGE]       req_addr_pos; // Position of the data in case there is a hit on tag array
logic [`ICACHE_NUM_WAY_RANGE]       miss_icache_pos;

//////////////////////////////////////////////////
// Position of the victim to be evicted from the I$
logic [`ICACHE_NUM_SET_RANGE]                           req_addr_set;  
logic [`THR_PER_CORE-1:0][`ICACHE_NUM_SET_RANGE]        miss_icache_set_next;  
logic [`THR_PER_CORE-1:0][`ICACHE_NUM_SET_RANGE]        miss_icache_set_ff;  
logic [`THR_PER_CORE-1:0][`ICACHE_WAYS_PER_SET_RANGE]   miss_icache_way_next; 
logic [`ICACHE_WAYS_PER_SET_RANGE]                      miss_icache_way; 
logic [`THR_PER_CORE-1:0][`ICACHE_WAYS_PER_SET_RANGE]   miss_icache_way_ff; 
logic [`THR_PER_CORE-1:0][`ICACHE_TAG_RANGE]            req_addr_tag_next;
logic [`THR_PER_CORE-1:0][`ICACHE_TAG_RANGE]            req_addr_tag_ff;

//         CLK    RST    EN           DOUT                DIN                  DEF
`RST_EN_FF(clock, reset, !icache_hit, miss_icache_set_ff, miss_icache_set_next, '0)
`RST_EN_FF(clock, reset, !icache_hit, miss_icache_way_ff, miss_icache_way_next, '0)
`RST_EN_FF(clock, reset, !icache_hit, req_addr_tag_ff,    req_addr_tag_next,    '0)

//////////////////////////////////////////////////
// Ready signal to stall the pipeline if ICache is busy
logic   [`THR_PER_CORE-1:0] icache_ready_next, icache_ready_ff;
assign icache_ready = (reset) ? '0 : icache_ready_ff;

    //  CLK    RST    DOUT             DIN                DEF
`RST_FF(clock, reset, icache_ready_ff, icache_ready_next, 2'b11)

logic   [`THR_PER_CORE-1:0] pendent_req, pendent_req_ff;

    //  CLK    RST    DOUT            DIN          DEF
`RST_FF(clock, reset, pendent_req_ff, pendent_req, '0)

integer iter;
always_comb
begin
    // Mantain values for next clock
    instMem_valid       = instMem_valid_ff;
    instMem_tag         = instMem_tag_ff;
    instMem_data        = instMem_data_ff;
    icache_ready_next   = icache_ready_ff;
    pendent_req         = pendent_req_ff;

    miss_icache_set_next = miss_icache_set_ff;
    miss_icache_way_next = miss_icache_way_ff;
    req_addr_tag_next    = req_addr_tag_ff;

    // There is a miss if the tag is not stored
    req_addr_tag    = req_addr[`ICACHE_TAG_ADDR_RANGE];
    req_addr_set    = req_addr[`ICACHE_SET_ADDR_RANGE];

    hit_way         = '0;
    req_addr_pos    = '0; 
  
    // Do not send request to MM until we ensure we miss on the icache
    req_valid_miss          = 1'b0;
    req_info_miss.data      = '0;
    req_info_miss.thread_id = req_thread_id;
    icache_hit              = 1'b0;

    // Do not respond to the fetch top until we ensure we have the correct
    // data
    rsp_valid       = 1'b0;
    xcpt_bus_error  = 1'b0;

    // If there is a request and we are not performing one
    if (req_valid & !pendent_req_ff[req_thread_id])
    begin
        // Look if the tag is on the cache
        for (iter = 0; iter < `ICACHE_WAYS_PER_SET; iter++)
        begin
            if ((instMem_tag_ff[iter + req_addr_set*`ICACHE_WAYS_PER_SET]   == req_addr_tag) &
                 instMem_valid_ff[iter + req_addr_set*`ICACHE_WAYS_PER_SET] == 1'b1)
            begin
                req_addr_pos      = iter + req_addr_set*`ICACHE_WAYS_PER_SET;
                icache_hit        = 1'b1;
                hit_way           = iter;

                // Return data to fetch top
                rsp_data    = instMem_data_ff[req_addr_pos];
                rsp_valid   = 1'b1;

                `ifdef VERBOSE_ICACHE
                    $display("[ICACHE] TAG hit. Position in I$ is %h , way %h",req_addr_pos,hit_way);
                `endif
            end
        end

        // If there is a miss we send a request to main memory to get the line
        if ( !icache_hit )
        begin
            `ifdef VERBOSE_ICACHE
                $display("[ICACHE] TAG miss asserted. Requested addr is %h",req_addr);
            `endif  
            miss_icache_set_next[req_thread_id] = req_addr_set;
            miss_icache_way_next[req_thread_id] = miss_icache_way;
            req_addr_tag_next[req_thread_id]    = req_addr_tag;
            pendent_req[req_thread_id]          = 1'b1;
            req_info_miss.addr                  = req_addr >> `ICACHE_RSH_VAL;
            req_info_miss.is_store              = 1'b0;
            req_info_miss.thread_id             = req_thread_id;
            req_valid_miss                      = !reset;
            icache_ready_next[req_thread_id]    = 1'b0;
        end
    end

    // We wait until we receive the response from main memory. Then, we update
    // the tag, data and valid information for the position related to that
    // tag 
    if (rsp_valid_miss)
    begin
        xcpt_bus_error  = rsp_bus_error;
        if (!rsp_bus_error)
        begin
            miss_icache_pos = miss_icache_way_ff[rsp_thread_id] + miss_icache_set_ff[rsp_thread_id]*`ICACHE_WAYS_PER_SET;
            `ifdef VERBOSE_ICACHE
                $display("[ICACHE] Response from MM valid. instMem_tag[%h] = %h",miss_icache_pos,rsp_data_miss);
            `endif            
            instMem_tag[miss_icache_pos]     = req_addr_tag_ff[rsp_thread_id];
            instMem_data[miss_icache_pos]    = rsp_data_miss;
            instMem_valid[miss_icache_pos]   = 1'b1;
            pendent_req[rsp_thread_id]       = 1'b0;
            icache_ready_next[rsp_thread_id] = 1'b1;
        end
    end
end

// This module returns the oldest way accessed for a given set and updates the
// the LRU logic when there's a hit on the I$ or we bring a new line                        
cache_lru_mt
#(
    .NUM_SET       ( `ICACHE_NUM_SET        ),
    .NUM_WAYS      ( `ICACHE_NUM_WAYS       ),
    .NUM_WAYS_MT   ( `ICACHE_NUM_WAYS_MT    ),
    .WAYS_PER_SET  ( `ICACHE_WAYS_PER_SET   )
)
icache_lru
(
    // System signals
    .clock              ( clock             ),
    .reset              ( reset             ),
    .mt_mode            ( mt_mode           ),
    .thread_id          ( req_thread_id     ),

    // Info to select the victim
    .victim_req         ( !icache_hit       ),
    .victim_set         ( req_addr_set      ),
    .victim_way         ( miss_icache_way   ),

    // Update the LRU logic in case of hit in the active thread
    .update_req         ( icache_hit        ),
    .update_set         ( req_addr_set      ),
    .update_way         ( hit_way           ),

    // Update the LRU logic in case of rsp from memory
    .update_req_mt      ( rsp_valid_miss & !rsp_bus_error   ),
    .update_set_mt      ( miss_icache_set_ff[rsp_thread_id] ),
    .update_way_mt      ( miss_icache_way_ff[rsp_thread_id] ),
    .update_thread_mt   ( rsp_thread_id                     )
);

endmodule 
