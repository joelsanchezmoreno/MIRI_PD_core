// Data Cache allows write and read requests, and keeps track of modified lines
// such that dirty lines are evicted to main memory before being replaced. 
// The data cache implementation ensures that one request can be served each cycle 
// if there are no misses. Otheriwse, in case of a miss it takes `MAIN_MEMORY_LATENCY 
// cycles to go to memory and bring the line if evict is not needed.
`include "soc.vh"

module data_cache_mt
(
    input   logic                               clock,
    input   logic                               reset,
    output  logic [`THR_PER_CORE-1:0]           dcache_ready,
    input   multithreading_mode_t               mt_mode,
    input   logic [`THR_PER_CORE_WIDTH-1:0]     active_thread_id,

    // Exception
    output  logic                               xcpt_bus_error,

    // Request from the core pipeline
    input   dcache_request_t                    req_info,
    input   logic                               req_valid,

    // Response to the core pipeline
    output  logic [`DCACHE_MAX_ACC_SIZE-1:0]    rsp_data,
    output  logic                               rsp_error, //conditional error
    output  logic                               rsp_valid,

    // Request to the memory hierarchy
    output  logic                               req_valid_miss,
    output  memory_request_t                    req_info_miss,

    // Response from the memory hierarchy
    input   logic [`DCACHE_LINE_WIDTH-1:0]      rsp_data_miss,
    input   logic [`THR_PER_CORE_WIDTH-1:0]     rsp_thread_id,
    input   logic                               rsp_bus_error,
    input   logic                               rsp_valid_miss
);
///////////////////////////////////
// Main Memory Request management

logic                                    req_valid_miss_active_thread;
logic               [`THR_PER_CORE-1:0]  req_valid_miss_arb;
memory_request_t    [`THR_PER_CORE-1:0]  req_info_miss_arb;
memory_request_t    [`THR_PER_CORE-1:0]  req_info_miss_arb_ff;

dcache_state_t [`THR_PER_CORE-1:0]  dcache_state_aux;
dcache_state_t [`THR_PER_CORE-1:0]  dcache_state_aux_ff;

//  CLK    DOUT                  DIN
`FF(clock, req_info_miss_arb_ff, req_info_miss_arb)

    //  CLK    RST    DOUT                   DIN               DEF
`RST_FF(clock, reset, dcache_state_aux_ff,   dcache_state_aux, '0)

logic [`THR_PER_CORE_WIDTH-1:0] arb_winner;
logic [`THR_PER_CORE_WIDTH-1:0] arb_winner_ff;
`FF(clock, arb_winner_ff, arb_winner)

arbiter_priority
#(.NUM_ENTRIES(`THR_PER_CORE))
arb_prio_mm
(
    .client_valid   ( req_valid_miss_arb    ),
    .top_client     ( active_thread_id      ),  // give priority to active thread
    .client_ready   (                       ),  // threads always ready to send req
    .winner         ( arb_winner            ),
    
    .valid          (                       ),
    .ready          ( 1'b1                  )
);

assign req_valid_miss = req_valid_miss_active_thread | (|req_valid_miss_arb);
assign req_info_miss  = (req_valid_miss_active_thread) ? req_info_miss_arb[active_thread_id] :
                                                         req_info_miss_arb[arb_winner_ff];

///////////////////////////////////
// Thread waits for a request from another thread
logic [`THR_PER_CORE-1:0][`DCACHE_TAG_RANGE]        blocked_by_thread_tag;
logic [`THR_PER_CORE-1:0][`DCACHE_TAG_RANGE]        blocked_by_thread_tag_ff;
logic [`THR_PER_CORE-1:0][`THR_PER_CORE_WIDTH-1:0]  blocked_by_thread_id;
logic [`THR_PER_CORE-1:0][`THR_PER_CORE_WIDTH-1:0]  blocked_by_thread_id_ff;
logic [`THR_PER_CORE-1:0]                           blocked_by_thread_valid;
logic [`THR_PER_CORE-1:0]                           blocked_by_thread_valid_ff;

//      CLK    RST    DOUT                        DIN                      DEF
`RST_FF(clock, reset, blocked_by_thread_valid_ff, blocked_by_thread_valid, '0)

//  CLK    DOUT                      DIN       
`FF(clock, blocked_by_thread_tag_ff, blocked_by_thread_tag)
`FF(clock, blocked_by_thread_id_ff,  blocked_by_thread_id)

///////////////////////////////////
// Reserved ways through conditional load/store
logic [`DCACHE_NUM_WAYS_R][`THR_PER_CORE_WIDTH-1:0] dCache_reserved_way;
logic [`DCACHE_NUM_WAYS_R][`THR_PER_CORE_WIDTH-1:0] dCache_reserved_way_ff;
logic [`DCACHE_NUM_WAYS_R]                          dCache_reserved_valid;
logic [`DCACHE_NUM_WAYS_R]                          dCache_reserved_valid_ff;

//      CLK    RST    DOUT                      DIN                    DEF
`RST_FF(clock, reset, dCache_reserved_valid_ff, dCache_reserved_valid, '0)

//  CLK    DOUT                      DIN       
`FF(clock, dCache_reserved_way_ff, dCache_reserved_way)


///////////////////////////////////
// Data Cache state for each thread
dcache_state_t [`THR_PER_CORE-1:0]   dcache_state;
dcache_state_t [`THR_PER_CORE-1:0]   dcache_state_ff;

//      CLK    RST    DOUT             DIN           DEF
`RST_FF(clock, reset, dcache_state_ff, dcache_state, '0)

//////////////////////////////////////////////////
// Data Cache arrays: tag, data, dirty and valid
logic [`DCACHE_LINE_RANGE]   dCache_data    [`DCACHE_NUM_WAYS_R];
logic [`DCACHE_LINE_RANGE]   dCache_data_ff [`DCACHE_NUM_WAYS_R];
logic [`DCACHE_TAG_RANGE]    dCache_tag     [`DCACHE_NUM_WAYS_R];
logic [`DCACHE_TAG_RANGE]    dCache_tag_ff  [`DCACHE_NUM_WAYS_R];
logic [`DCACHE_NUM_WAYS_R]   dCache_dirty;
logic [`DCACHE_NUM_WAYS_R]   dCache_dirty_ff;
logic [`DCACHE_NUM_WAYS_R]   dCache_valid;
logic [`DCACHE_NUM_WAYS_R]   dCache_valid_ff;

//  CLK    DOUT             DIN       
`FF(clock, dCache_data_ff , dCache_data)
`FF(clock, dCache_tag_ff  , dCache_tag )

//      CLK    RST    DOUT               DIN         DEF
`RST_FF(clock, reset, dCache_valid_ff, dCache_valid, '0)
`RST_FF(clock, reset, dCache_dirty_ff, dCache_dirty, '0)

//////////////////////////////////////////////////
// Control signals 
logic dcache_tags_hit;  // asserted when there is a hit on the instr. cache
logic [`DCACHE_WAYS_PER_SET_RANGE] hit_way; // stores the way in case of a D$ hit 

//////////////////////////////////////////////////
// Store Buffer signals 
store_buffer_t  store_buffer_push_info;
store_buffer_t  store_buffer_pop_info;
logic store_buffer_perform;
logic store_buffer_pending;
logic store_buffer_full;

    // During the idle times for the D$, we ensure that the Store Buffer
    // requests are performed whenever possible. We must ensure that only one
    // thread takes the pop data and writes the data cache
logic [`THR_PER_CORE_WIDTH-1:0]  store_buffer_chosen_thread;

integer k;
always_comb
begin
    store_buffer_perform = 1'b0;
    for (k = 0; k< `THR_PER_CORE; k++)
    begin
        if (!req_valid && store_buffer_pending && dcache_state_ff[k] == idle)
        begin
            store_buffer_chosen_thread  = k;
            store_buffer_perform        = 1'b1;
        end
    end
end

//////////////////////////////////////////////////
// Signals to save the request information for possible next stages

// Position of the D$ data in case there is a hit on tag array
logic [`THR_PER_CORE-1:0][`DCACHE_NUM_WAY_RANGE]   req_target_pos; 
logic [`THR_PER_CORE-1:0][`DCACHE_NUM_WAY_RANGE]   req_target_pos_ff; 

//  CLK    DOUT                DIN       
`FF(clock, req_target_pos_ff,  req_target_pos)

//////////////////////////////////////////////////
// Position of the victim to be evicted from the D$
logic [`THR_PER_CORE-1:0][`DCACHE_NUM_SET_RANGE]    req_set;  
logic [`DCACHE_WAYS_PER_SET_RANGE]                  miss_dcache_way;  

//////////////////////////////////////////////////
// Ready signal to stall the pipeline if DCache is busy
logic [`THR_PER_CORE-1:0] dcache_ready_next;
logic [`THR_PER_CORE-1:0] dcache_ready_ff;

//      CLK    RST    DOUT          DIN                DEF
`RST_FF(clock, reset, dcache_ready_ff, dcache_ready_next, '0)

assign dcache_ready = dcache_ready_ff;

//////////////////////////////////////////////////
// Store buffer signals

// Asserted when we request the store_buffer to search for a specific address
logic [`THR_PER_CORE-1:0]                     search_store_buffer;
logic [`THR_PER_CORE-1:0][`DCACHE_TAG_RANGE]  search_tag;

// Asserted if the store buffer contains a req. to the same TAG as the one requested
// in which case we have to perform the store before returning/modifying the line
logic [`THR_PER_CORE-1:0] store_buffer_hit_tag;
logic [`THR_PER_CORE-1:0] store_buffer_hit_tag_ff ;

// Asserted if the store buffer contains a req. to the same line as the one requested
// in which case we have to perform the store before evicting the line (if needed)
logic [`THR_PER_CORE-1:0]   store_buffer_hit_way;
logic [`THR_PER_CORE-1:0]   store_buffer_hit_way_ff;


// Saves the request extracted from the ST buffer
store_buffer_t [`THR_PER_CORE-1:0] pending_store_req;
store_buffer_t [`THR_PER_CORE-1:0] pending_store_req_ff;

genvar u;
generate for (u=0; u < `THR_PER_CORE; u++) 
begin
        //     CLK    RST    EN                      DOUT                        DIN                     DEF
    `RST_EN_FF(clock, reset, search_store_buffer[u], store_buffer_hit_tag_ff[u], store_buffer_hit_tag[u], '0)
    `RST_EN_FF(clock, reset, search_store_buffer[u], store_buffer_hit_way_ff[u], store_buffer_hit_way[u], '0)
    
        //     CLK    RST    EN                                                   DOUT                     DIN                  DEF
    `RST_EN_FF(clock, reset, (store_buffer_hit_tag[u] | store_buffer_hit_way[u]), pending_store_req_ff[u], pending_store_req[u], '0)
end
endgenerate

// Saves the request received in case we need to perform a request from the ST
// buffer
dcache_request_t [`THR_PER_CORE-1:0]    pending_req;
dcache_request_t [`THR_PER_CORE-1:0]    pending_req_ff;
`FF(clock, pending_req_ff, pending_req)

// Signals for operating the partial stores to write on the D$
logic [`THR_PER_CORE-1:0][`DCACHE_OFFSET_RANGE] req_offset;
logic [`THR_PER_CORE-1:0][`DCACHE_TAG_RANGE]    req_tag   ;
req_size_t [`THR_PER_CORE-1:0]                  req_size  ;


//////////////////////////////////////////////////
// Logic

integer thread_id;    
integer iter;
integer thr;

always_comb
begin        
        // Status signals
    dcache_ready_next   = dcache_ready_ff;
    dcache_state        = dcache_state_ff;

        // Cache arrays
    dCache_valid        = dCache_valid_ff;
    dCache_tag          = dCache_tag_ff;
    dCache_data         = dCache_data_ff;
    dCache_dirty        = dCache_dirty_ff;

        // Control signals to allocate new lines
    req_target_pos      = req_target_pos_ff;
    pending_req         = pending_req_ff;
    req_valid_miss_active_thread = 1'b0;

        // Control signals for main memory request tracking
    blocked_by_thread_valid = blocked_by_thread_valid_ff;
    blocked_by_thread_id    = blocked_by_thread_id_ff;
    blocked_by_thread_tag   = blocked_by_thread_tag_ff;
       
        // Control signals for conditional operations
    dCache_reserved_valid    = dCache_reserved_valid_ff;
    dCache_reserved_way      = dCache_reserved_way_ff;
   
        // Arbiter
    dcache_state_aux    = dcache_state_aux_ff;
    req_info_miss_arb   = req_info_miss_arb_ff;

        // Store Buffer
    search_store_buffer = '0;

        // Exception
    xcpt_bus_error = 1'b0;

        // Response to core
    rsp_valid       = 1'b0;
    rsp_error       = 1'b0;
    dcache_tags_hit = 1'b0;

    // Mantain values for next clock
    for (thread_id=0; thread_id < `THR_PER_CORE; thread_id++) 
    begin
        case(dcache_state_ff[thread_id])
            idle:
            begin
                req_valid_miss_arb[thread_id]  = 1'b0;
                dcache_ready_next[thread_id] = !store_buffer_full;
              
                // Compute the tag and set for the given address 
                req_tag[thread_id]    = req_info.addr[`DCACHE_TAG_ADDR_RANGE];
                req_set[thread_id]    = req_info.addr[`DCACHE_SET_ADDR_RANGE]; 
                req_offset[thread_id] = req_info.addr[`DCACHE_OFFSET_ADDR_RANGE] >> $clog2(req_info.size+1);

                // Perform the request
                if (req_valid && (active_thread_id == thread_id))
                begin
                    // Look if the requested tag is on the cache
                    for (iter = 0; iter < `DCACHE_WAYS_PER_SET; iter++)
                    begin
                         if((dCache_tag_ff[iter + req_set[thread_id]*`DCACHE_WAYS_PER_SET] == req_tag[thread_id]) &
                             dCache_valid[iter + req_set[thread_id]*`DCACHE_WAYS_PER_SET]  == 1'b1)
                        begin
                            req_target_pos[thread_id]   = iter + req_set[thread_id]*`DCACHE_WAYS_PER_SET;
                            dcache_tags_hit             = 1'b1;
                            hit_way                     = iter;
                        end
                    end

                    // Look on the ST buffer if there are pendent requests for
                    // the requested address. 
                    //    LD miss -- look if we need to modify the line to be
                    //               evicted in case evict is needed
                    //    LD hit  -- look if we need to modify the line before
                    //               returning the data
                    //    ST miss -- look if we need to modify the line to be
                    //               evicted in case evict is needed
                    //    ST hit  -- we just push the request to ST buffer, no
                    //               need to look for requests to the same way/tag

                    // Position of the way to be written
                    if (!dcache_tags_hit)
                        req_target_pos[thread_id] = miss_dcache_way + req_set[thread_id]*`DCACHE_WAYS_PER_SET;
                        
                    search_store_buffer[thread_id] = ( req_info.is_store & dcache_tags_hit  ) ? 1'b0 : // ST hit
                                                     (!req_info.is_store & dcache_tags_hit  ) ? 1'b1 : // LD hit
                                                                                                dCache_dirty_ff[req_target_pos[thread_id]]; // LD/ST miss                                      

                    search_tag[thread_id]          = req_info.addr[`DCACHE_TAG_ADDR_RANGE];

                    // [INFO] If we hit on the D$ we may hit on the ST buffer 
                    // since the ST buffer cannot have requests to lines that
                    // are not on the D$ 
                    
                    // If there is a ST hit we push the request to the store buffer
                    if (dcache_tags_hit & req_info.is_store)
                    begin
                        if (  (!dCache_reserved_valid_ff[hit_way]) 
                            | ( dCache_reserved_way_ff[hit_way] == thread_id))
                        begin
                                // Remove reserved once STC has been performed
                            dCache_reserved_valid[hit_way] = 1'b0;

                            //FIXME.TODO. Merge requests if there are more on the store buffer
                            //            with same tag?
                            rsp_valid = 1'b1;
                            store_buffer_push_info.addr = req_info.addr;
                            store_buffer_push_info.way  = hit_way;
                            store_buffer_push_info.size = req_info.size;
                            store_buffer_push_info.data = req_info.data;
                            store_buffer_push_info.thread_id = active_thread_id;
                            `ifdef VERBOSE_DCACHE
                                $display("[DCACHE] D$ hit and store -- adding req to store buffer");
                                $display("          addr = %h",req_info.addr);
                            `endif
                        end
                        // If it is a ST conditional but the way was not reserved 
                        // for this thread. Return error.
                        else 
                        begin
                            rsp_valid = 1'b1;
                            rsp_error = 1'b1;
                            rsp_data  = '1;
                        end
                    end

                    // If there is a LD hit we evaluate the conditions of that hit
                    // depending if there are ST on the store buffer waiting to
                    // modify the same line
                    else if (dcache_tags_hit & !req_info.is_store) //LD_hit
                    begin
                            // Reserve way if conditional LD
                        dCache_reserved_valid[hit_way] |= req_info.conditional;
                        if (req_info.conditional)
                            dCache_reserved_way[hit_way] = thread_id;

                        // If there is no store waiting to modify that line we return
                        // the data
                        if (!store_buffer_hit_tag[thread_id])
                        begin
                            `ifdef VERBOSE_DCACHE
                                $display("[DCACHE] D$ hit and load with ST buffer empty -- respond request");
                                $display("         addr = %h",req_info.addr);
                            `endif

                            if ( req_info.size == Byte)
                            begin
                                req_offset[thread_id]  = req_info.addr[`DCACHE_OFFSET_ADDR_RANGE];
                                rsp_data    = `ZX_BYTE(`DCACHE_MAX_ACC_SIZE,dCache_data[req_target_pos[thread_id]][`GET_LOWER_BOUND(`BYTE_BITS,req_offset[thread_id])+:`BYTE_BITS]);
                            end
                            else
                            begin
                                req_offset[thread_id]  = req_info.addr[`DCACHE_OFFSET_ADDR_RANGE] >> $clog2(req_info.size+1);
                                rsp_data  = `ZX_DWORD(`DCACHE_MAX_ACC_SIZE, dCache_data[req_target_pos[thread_id]][`GET_LOWER_BOUND(`DWORD_BITS,req_offset[thread_id])+:`DWORD_BITS]);
                            end
                            rsp_valid = 1'b1;
                        end
                        // If there is a store request on the store buffer that
                        // modifies the same line, we should perform the store before
                        // returning the line
                        else
                        begin
                            `ifdef VERBOSE_DCACHE
                                $display("[DCACHE] D$ hit and load with ST buffer NOT empty -- jump to write cache line state");
                                $display("         addr = %h",req_info.addr);
                            `endif
                            dcache_ready_next[thread_id]  = 1'b0;
                            pending_req[thread_id]        = req_info; // We save the request we received
                            dcache_state[thread_id]       = write_cache_line;
                        end
                    end
                    // If we do NOT hit on the D$ Tags (miss either LD or ST)
                    else
                    begin
                        dcache_ready_next[thread_id]   = 1'b0;

                            // Reserve way if conditional LD. Report error if
                            // conditional ST and no reservation was done
                        if (req_info.conditional)
                        begin
                            if (!req_info.is_store) // If load
                            begin
                                dCache_reserved_valid[req_target_pos[thread_id]]  = 1'b1;
                                dCache_reserved_way[req_target_pos[thread_id]]    = thread_id;
                            end
                            else // If store
                            begin
                                if(dCache_reserved_way_ff[req_target_pos[thread_id]] != thread_id)
                                begin
                                    rsp_valid = 1'b1;
                                    rsp_error = 1'b1;
                                    rsp_data  = '1;
                                    dcache_ready_next[thread_id]   = 1'b1;
                                end
                            end
                        end
                        if (!rsp_error)
                        begin
                            // We evaluate if a different thread failed to get the
                            // same tag, if that is the case we then move to
                            // bring_line and wait for the other thread to
                            // receive the response from memory
                            for(thr = 0; thr < `THR_PER_CORE; thr++)
                            begin
                               if(  blocked_by_thread_valid_ff[thr]
                                  & blocked_by_thread_tag_ff[thr] == req_tag[thread_id])
                               begin
                                   blocked_by_thread_id[thread_id] = thr;
                                   dcache_state[thread_id] = bring_line;
                               end
                            end
                            // If we found that we're blocked by another thread,
                            // then do nothing and go to bring line.
                            // Otherwise, check if we need to replace ways, or
                            // if the store buffer has pendent requests or if
                            // we need to bring the line
                            if (dcache_state[thread_id] != bring_line)
                            begin
                                // If there is a request on the store buffer that targets
                                // the way we want to replace, we need to perform the ST
                                // and then evict the line
                                if (store_buffer_hit_way[thread_id])
                                begin
                                    pending_req[thread_id]   = req_info;
                                    dcache_state[thread_id]  = write_cache_line;
                                end
                                // If there are NO requests on the store buffer that targets
                                // the line we want to replace. Then, we can evict the line
                                // if needed 
                                else 
                                begin
                                    // If the data is dirty on the cache we have to evict
                                    if ( dCache_dirty_ff[req_target_pos[thread_id]] )
                                    begin
                                        `ifdef VERBOSE_DCACHE
                                            $display("[DCACHE] D$ miss and dirty -- send evict request to main memory");
                                            $display("         addr = %h",req_info.addr);
                                            $display("    dirty pos = %h",req_target_pos[thread_id]);
                                        `endif
                                        // Send request to evict the line
                                        req_info_miss_arb[thread_id].addr     = ( {dCache_tag[req_target_pos[thread_id]],req_set[thread_id], 
                                                                                  {`DCACHE_OFFSET_WIDTH{1'b0}}} >> `DCACHE_ADDR_RSH_VAL);
                                        req_info_miss_arb[thread_id].is_store  = 1'b1;
                                        req_info_miss_arb[thread_id].data      = dCache_data_ff[req_target_pos[thread_id]];
                                        req_info_miss_arb[thread_id].thread_id = thread_id;
                                        req_valid_miss_active_thread           = 1'b1;
                                        
                                        // Invalidate the line
                                        dCache_valid[req_target_pos[thread_id]] = 1'b0;
                                        dCache_dirty[req_target_pos[thread_id]] = 1'b0;

                                        // Save pendent request addr (for future miss)
                                        // on the MM array such that if a new thread
                                        // asks for the same line, we block it
                                        blocked_by_thread_tag[thread_id]    = req_tag[thread_id];
                                        blocked_by_thread_valid[thread_id]  = 1'b1;

                                        // Next stage
                                        pending_req[thread_id]     = req_info;                    
                                        dcache_state[thread_id]    = evict_line;
                                    end
                                    // If the line is not dirty on the cache we just need to bring
                                    // the new one.
                                    else 
                                    begin
                                        `ifdef VERBOSE_DCACHE
                                            $display("[DCACHE] D$ miss and NOT dirty -- send miss request to main memory");
                                            $display("         addr = %h",req_info.addr);
                                        `endif

                                        req_info_miss_arb[thread_id].addr      = req_info.addr >> `DCACHE_ADDR_RSH_VAL;
                                        req_info_miss_arb[thread_id].is_store  = 1'b0;                            
                                        req_info_miss_arb[thread_id].thread_id = thread_id;
                                        req_valid_miss_active_thread           = 1'b1;

                                        // Save pendent request addr (for future miss)
                                        // on the MM array such that if a new thread
                                        // asks for the same line, we block it
                                        blocked_by_thread_tag[thread_id]    = req_tag[thread_id];
                                        blocked_by_thread_valid[thread_id]  = 1'b1;

                                        // Next stage
                                        pending_req[thread_id]  = req_info;                    
                                        dcache_state[thread_id] = bring_line;
                                    end //!dCache_dirty_ff[req_target_pos[thread_id]]
                                end // store_buffer_hit_way
                            end // !blocked by another thread
                        end //!rsp_error
                    end // !LD_hit
                end // req_valid
                else
                begin
                        // Modify the D$ with the store buffer request information
                    if (store_buffer_perform && (store_buffer_chosen_thread == thread_id))
                    begin
                        // Compute the tag and set for the given address 
                        req_tag[thread_id]     = store_buffer_pop_info.addr[`DCACHE_TAG_ADDR_RANGE];
                        req_set[thread_id]     = store_buffer_pop_info.addr[`DCACHE_SET_ADDR_RANGE]; 
                        req_size[thread_id]    = store_buffer_pop_info.size;

                        req_target_pos[thread_id]  = store_buffer_pop_info.way + req_set[thread_id]*`DCACHE_WAYS_PER_SET;
                   
                        // Update D$ 
                        dCache_tag[req_target_pos[thread_id]]   = req_tag[thread_id];
                        dCache_dirty[req_target_pos[thread_id]] = 1'b1; 
               
                        if ( req_size[thread_id] == Byte)
                        begin
                            req_offset[thread_id]  = store_buffer_pop_info.addr[`DCACHE_OFFSET_ADDR_RANGE];
                            dCache_data[req_target_pos[thread_id]][`GET_LOWER_BOUND(`BYTE_BITS,req_offset[thread_id])+:`BYTE_BITS] = store_buffer_pop_info.data[`BYTE_RANGE];
                        end
                        else
                        begin
                            req_offset[thread_id]  = store_buffer_pop_info.addr[`DCACHE_OFFSET_ADDR_RANGE] >> $clog2(store_buffer_pop_info.size+1);
                            dCache_data[req_target_pos[thread_id]][`GET_LOWER_BOUND(`DWORD_BITS,req_offset[thread_id])+:`DWORD_BITS] = store_buffer_pop_info.data[`DWORD_RANGE];
                        end
                    end
                end
            end

            // This state is executed when we've sent an evict request for a D$ line,
            // so we wait until we receive the ACK signal and then we send a request
            // to get the new line.
            evict_line:
            begin
                req_valid_miss_arb[thread_id] = 1'b0;

                // Wait for response from memory ACK
                if (rsp_valid_miss && (thread_id == rsp_thread_id))
                begin
                    // Send new request to bring the new line
                    req_info_miss_arb[thread_id].addr      = pending_req_ff[thread_id].addr >> `DCACHE_ADDR_RSH_VAL;
                    req_info_miss_arb[thread_id].is_store  = 1'b0;
                    req_info_miss_arb[thread_id].thread_id = thread_id;
                    req_valid_miss_arb[thread_id]          = 1'b1;

                    dcache_state[thread_id]     = pendent_request;
                    dcache_state_aux[thread_id] = bring_line;
                end
            end

            // This state is executed when a line has been requested to main memory,
            // so we wait for response from main memory and then we respond to the
            // dcache_top
            bring_line:
            begin
                // We wait until we receive the response from main memory. 
                req_valid_miss_arb[thread_id] = 1'b0;

                // If the thread was waiting for another thread response, then
                // we can unblock and move to next stage
                if (  rsp_valid_miss 
                    & blocked_by_thread_valid_ff[rsp_thread_id]
                    & (blocked_by_thread_id_ff[thread_id] == rsp_thread_id))
                begin                  
                    dcache_ready_next[thread_id]   = 1'b1;
                    dcache_state[thread_id]        = idle;
                end

                // Update the tag, data and valid information for the position related to that
                // tag
                if (  rsp_valid_miss & (thread_id == rsp_thread_id))
                begin
                    // Clear pendent request addr since thread_id has received the data and
                    // will write it into memory
                    blocked_by_thread_valid[thread_id]  = 1'b0;

                    xcpt_bus_error = rsp_bus_error;
                    rsp_valid      = (thread_id == active_thread_id);
                    if (!rsp_bus_error)
                    begin
                        // Compute signals from the pending ST request
                        req_tag[thread_id]    = pending_req_ff[thread_id].addr[`DCACHE_TAG_ADDR_RANGE];
                        req_set[thread_id]    = pending_req_ff[thread_id].addr[`DCACHE_SET_ADDR_RANGE]; 
                        req_size[thread_id]   = pending_req_ff[thread_id].size;

                        // If it was a ST, we modify the received line and put
                        // that line as dirty
                        if (pending_req_ff[thread_id].is_store)
                        begin
                            dCache_data[req_target_pos_ff[thread_id]]  = rsp_data_miss; 
                            dCache_dirty[req_target_pos_ff[thread_id]] = 1'b1; 
                            if ( req_size[thread_id] == Byte)
                            begin
                                req_offset[thread_id] = pending_req_ff[thread_id].addr[`DCACHE_OFFSET_ADDR_RANGE];
                                dCache_data[req_target_pos_ff[thread_id]][`GET_LOWER_BOUND(`BYTE_BITS,req_offset[thread_id])+:`BYTE_BITS] =  pending_req_ff[thread_id].data[`BYTE_RANGE];
                            end
                            else
                            begin
                                req_offset[thread_id] = pending_req_ff[thread_id].addr[`DCACHE_OFFSET_ADDR_RANGE]  >> $clog2(pending_req_ff[thread_id].size+1);
                                dCache_data[req_target_pos_ff[thread_id]][`GET_LOWER_BOUND(`DWORD_BITS,req_offset[thread_id])+:`DWORD_BITS] =  pending_req_ff[thread_id].data[`DWORD_RANGE];
                            end
                        end
                        // If it was a LD, we copy the line received from memory and
                        // return valid data
                        else
                        begin
                            dCache_data[req_target_pos_ff[thread_id]]  = rsp_data_miss;

                            // Respond request from dcache_top if we are the active thread
                            if ( req_size[thread_id] == Byte)
                            begin
                                req_offset[thread_id] = pending_req_ff[thread_id].addr[`DCACHE_OFFSET_ADDR_RANGE];
                                if (thread_id == active_thread_id)
                                    rsp_data  = `ZX_BYTE(`DCACHE_MAX_ACC_SIZE,rsp_data_miss[`GET_LOWER_BOUND(`BYTE_BITS,req_offset[thread_id])+:`BYTE_BITS]);
                            end
                            else
                            begin
                                req_offset[thread_id] = pending_req_ff[thread_id].addr[`DCACHE_OFFSET_ADDR_RANGE]  >> $clog2(pending_req_ff[thread_id].size+1);
                                if (thread_id == active_thread_id)
                                    rsp_data  = `ZX_DWORD(`DCACHE_MAX_ACC_SIZE, rsp_data_miss[`GET_LOWER_BOUND(`DWORD_BITS,req_offset[thread_id])+:`DWORD_BITS]);
                            end
                        end

                        dCache_tag[req_target_pos_ff[thread_id]]   = req_tag[thread_id];
                        dCache_valid[req_target_pos_ff[thread_id]] = 1'b1;
                    end 

                    // Next stage
                    dcache_ready_next[thread_id]   = 1'b1;
                    dcache_state[thread_id]        = idle;
                end //!rsp_valid_miss
            end

            // This state is executed:
            // 1.When there is a LD request that hits and there are requests 
            //   on the store buffer that modify the targetted line, so we 
            //   have to modify the data before returning it.
            // or
            //  2. When there is a miss (either LD or ST) and there are pending
            //     stores on the store_buffer for the line we want to replace.
            write_cache_line:
            begin
                req_valid_miss_arb[thread_id] = 1'b0;
                dcache_ready_next[thread_id] = 1'b0;
                // If there is a pending ST req. on the store buffer. Then, we should modify
                // the line before responding the LD or evicting the line
                if (store_buffer_hit_tag_ff[thread_id] | store_buffer_hit_way_ff[thread_id])
                begin
                    // Compute signals from the pending ST request
                    req_tag[thread_id]    = pending_store_req_ff[thread_id].addr[`DCACHE_TAG_ADDR_RANGE];
                    req_size[thread_id]   = pending_store_req_ff[thread_id].size;

                    // Modify the D$ with the store buffer request information
                    dCache_tag[req_target_pos_ff][thread_id]   =  req_tag[thread_id];
                    dCache_dirty[req_target_pos_ff[thread_id]] = 1'b1; 
                    dCache_valid[req_target_pos_ff[thread_id]] = 1'b1;

                    if ( req_size[thread_id] == Byte)
                    begin
                        req_offset[thread_id] = pending_store_req_ff[thread_id].addr[`DCACHE_OFFSET_ADDR_RANGE];
                        dCache_data[req_target_pos_ff[thread_id]][`GET_LOWER_BOUND(`BYTE_BITS,req_offset[thread_id])+:`BYTE_BITS] = pending_store_req_ff[thread_id].data[`BYTE_RANGE];
                    end
                    else
                    begin
                        req_offset[thread_id] = pending_store_req_ff[thread_id].addr[`DCACHE_OFFSET_ADDR_RANGE] >> $clog2(pending_req_ff[thread_id].size+1);
                        dCache_data[req_target_pos_ff[thread_id]][`GET_LOWER_BOUND(`DWORD_BITS,req_offset[thread_id])+:`DWORD_BITS] = pending_store_req_ff[thread_id].data[`DWORD_RANGE];
                    end

                    `ifdef VERBOSE_WRITE_CACHE_LINE
                        $display("[WRITE CACHE LINE]");
                        $display("                   req_target_pos_ff              = %h",req_target_pos_ff );
                        $display("                   req_offset                     = %h",req_offset[thread_id] );
                        $display("                   req_size                       = %h",req_size );[thread_id]
                        $display("                   pending_store_req_ff.data      = %h",pending_store_req_ff[thread_id].data );
                        $display("                   dCache_data[req_target_pos_ff] = %h",dCache_data[req_target_pos_ff[thread_id]] );
                    `endif

                    // Check if there are more ST that affect the line on the store buffer.
                    // If there is another ST that affects the same line or TAG, we perform
                    // the write request
                    search_store_buffer[thread_id] = 1'b1;
                    search_tag[thread_id]          = pending_req_ff[thread_id].addr[`DCACHE_TAG_ADDR_RANGE];
                    if (store_buffer_hit_tag[thread_id] | store_buffer_hit_way[thread_id])
                    begin
                        dcache_state[thread_id]        = write_cache_line;
                    end
                    // Otherwise, if there are no more store_buff req. that affect this line.
                    else
                    begin
                        // If we were updating the line due to a LD hit we return the data and go to idle
                        if ( store_buffer_hit_tag_ff[thread_id])
                        begin
                            req_size    = pending_req_ff[thread_id].size;
                           
                            // If it was a LD request we return the data 
                            if (!pending_req_ff[thread_id].is_store)
                            begin
                                if ( req_size == Byte)
                                begin
                                    req_offset[thread_id]  = pending_req_ff[thread_id].addr[`DCACHE_OFFSET_ADDR_RANGE];
                                    // We only return the data if we are the active thread
                                    if (thread_id == active_thread_id)
                                    begin
                                        rsp_data  = dCache_data[req_target_pos_ff[thread_id]] >> ((req_size+1)*`BYTE_BITS*req_offset[thread_id]) ;
                                        rsp_data  = `ZX_BYTE(`DCACHE_MAX_ACC_SIZE, rsp_data[`BYTE_RANGE]);
                                    end
                                end
                                else
                                begin
                                    req_offset[thread_id]  = pending_req_ff[thread_id].addr[`DCACHE_OFFSET_ADDR_RANGE] >> $clog2(pending_req_ff[thread_id].size+1);
                                    // We only return the data if we are the active thread
                                    if (thread_id == active_thread_id)
                                    begin
                                        rsp_data  = dCache_data[req_target_pos_ff[thread_id]] >> ((req_size+1)*`BYTE_BITS*req_offset[thread_id]) ;
                                        rsp_data  = `ZX_DWORD(`DCACHE_MAX_ACC_SIZE, rsp_data[`DWORD_RANGE]);
                                    end
                                end
                                rsp_valid = (thread_id == active_thread_id);
                            end
                    
                            // Next stage 
                            dcache_ready_next[thread_id]   = 1'b1;                    
                            dcache_state[thread_id]        = idle;
                        end 
                        //Otherwise, if we were updating the line before an evict, we
                        //send the evict request
                        else
                        begin
                            // Send request to evict the line
                            req_info_miss_arb[thread_id].addr       = pending_store_req_ff[thread_id].addr >> `DCACHE_ADDR_RSH_VAL; //Evict full line
                            req_info_miss_arb[thread_id].is_store   = 1'b1;
                            req_info_miss_arb[thread_id].data       = dCache_data[req_target_pos_ff[thread_id]];
                            req_info_miss_arb[thread_id].thread_id  = thread_id;
                            req_valid_miss_arb[thread_id]           = 1'b1;
                            
                            // Invalidate the line
                            dCache_valid[req_target_pos_ff[thread_id]] = 1'b0;
                            dCache_dirty[req_target_pos_ff[thread_id]] = 1'b0;

                            dcache_state[thread_id]     = pendent_request;
                            dcache_state_aux[thread_id] = evict_line;
                        end
                    end                
                end
            end

            // This state is executed when the thread needed to perform
            // a request to memory but there was another thread with higher
            // priority. So we need to wait until the thread can send the
            // request and move to the next state
            pendent_request:
            begin
                req_valid_miss_arb[thread_id]  = 1'b1;
                if (dcache_state_aux_ff[thread_id] == evict_line)
                begin                
                    req_info_miss_arb[thread_id].addr       = pending_store_req_ff[thread_id].addr >> `DCACHE_ADDR_RSH_VAL; //Evict full line
                    req_info_miss_arb[thread_id].is_store   = 1'b1;
                    req_info_miss_arb[thread_id].data       = dCache_data[req_target_pos_ff[thread_id]];
                    req_info_miss_arb[thread_id].thread_id  = thread_id;
                end
                else //wanted to bring line
                begin                    
                    req_info_miss_arb[thread_id].addr       = pending_req_ff[thread_id].addr >> `DCACHE_ADDR_RSH_VAL;
                    req_info_miss_arb[thread_id].is_store   = 1'b0;
                    req_info_miss_arb[thread_id].thread_id  = thread_id;
                end
                            
                if (arb_winner_ff == thread_id)
                    dcache_state[thread_id] = dcache_state_aux_ff[thread_id];
                else
                    dcache_state[thread_id] = pendent_request;
            end
        endcase
    end // for (thread_id=0; thread_id < `THR_PER_CORE; thread_id++)
end

//////////////////////////////////////
// Dcache LRU logic 

logic [`ICACHE_NUM_SET_RANGE] update_set;  
logic [`DCACHE_WAYS_PER_SET_RANGE] update_way;  
logic update_dcache_lru;

logic [`ICACHE_NUM_SET_RANGE] update_set_mm;  
logic [`DCACHE_WAYS_PER_SET_RANGE] update_way_mm;  
logic update_dcache_lru_mm;

assign update_dcache_lru    = dcache_tags_hit;
assign update_dcache_lru_mm = (dcache_state_ff[rsp_thread_id] == bring_line) & rsp_valid_miss;

assign update_set    = req_set[active_thread_id] ;
assign update_set_mm = req_set[rsp_thread_id] ;

assign update_way    = hit_way;
assign update_way_mm = req_target_pos_ff[rsp_thread_id] - (req_size[rsp_thread_id]+1)*`BYTE_BITS*`DCACHE_WAYS_PER_SET; // bring new line

    // This module returns the oldest way accessed for a given set and updates the
    // the LRU logic when there's a hit on the D$ or we bring a new line                        
cache_lru_mt
#(
    .NUM_SET       ( `DCACHE_NUM_SET        ),
    .NUM_WAYS      ( `DCACHE_NUM_WAYS       ),
    .NUM_WAYS_MT   ( `DCACHE_NUM_WAYS_MT    ),
    .WAYS_PER_SET  ( `DCACHE_WAYS_PER_SET   )
)
dcache_lru
(
    // System signals
    .clock              ( clock             ),
    .reset              ( reset             ),
    .mt_mode            ( mt_mode           ),
    .thread_id          ( active_thread_id  ),

    // Info to select the victim
    .victim_req         ( !dcache_tags_hit  ),
    .victim_set         ( req_info.addr[`DCACHE_SET_ADDR_RANGE] ),
    .victim_way         ( miss_dcache_way   ),

    // Update the LRU logic in case of hit in the active thread
    .update_req         ( update_dcache_lru ),
    .update_set         ( update_set        ),
    .update_way         ( update_way        ),

    // Update the LRU logic in case of rsp from memory
    .update_req_mt      ( update_dcache_lru_mm ),
    .update_set_mt      ( update_set_mm        ),
    .update_way_mt      ( update_way_mm        ),
    .update_thread_mt   ( rsp_thread_id        )
);

//////////////////////////////////////
// Dcache Store Buffer instance

store_buffer
store_buffer
(
    // System signals
    .clock              ( clock                 ),
    .reset              ( reset                 ),

    .buffer_empty       ( store_buffer_pending  ),
    .buffer_full        ( store_buffer_full     ),

    // Get the information from the oldest store on the buffer
    .get_oldest         ( store_buffer_perform  ),
    .oldest_info        ( store_buffer_pop_info ),

    // Push a new store to the buffer Update the LRU logic
    .push_valid         ( dcache_tags_hit & 
                          req_info.is_store     ),
    .push_info          ( store_buffer_push_info), 

    // Look for hit on store buffer
    .search_valid       ( search_store_buffer   ), 
    .search_tag         ( search_tag            ), 
    .search_way         ( miss_dcache_way       ), 
    .search_rsp_hit_tag ( store_buffer_hit_tag  ),
    .search_rsp_hit_way ( store_buffer_hit_way  ),
    .search_rsp         ( pending_store_req     )
);

endmodule 
