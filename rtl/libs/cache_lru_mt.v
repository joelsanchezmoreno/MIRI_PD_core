`include "soc.vh"

module cache_lru
#(
    parameter NUM_SET        = `ICACHE_NUM_SET,
    parameter NUM_WAYS       = `ICACHE_NUM_WAYS,
    parameter NUM_WAYS_MT    = `ICACHE_NUM_WAYS_MT,
    parameter WAYS_PER_SET   = `ICACHE_WAYS_PER_SET,
    parameter NUM_SET_W      = $clog2(NUM_SET),
    parameter NUM_WAYS_W     = $clog2(NUM_WAYS),
    parameter WAYS_PER_SET_W = $clog2(WAYS_PER_SET) 
)
(
    // System signals
    input   logic                           clock,
    input   logic                           reset,
    input   multithreading_mode_t           mt_mode, //Multi-threading mode
    input   logic [`THR_PER_CORE_WIDTH-1:0] thread_id,

    // Searches for a victim on the ways reserved for each thread
    input   logic                       victim_req,
    input   logic  [NUM_SET_W-1:0]      victim_set,
    output  logic  [WAYS_PER_SET_W-1:0] victim_way,

    // Update the set LRU only when the hit is from the same thread
    input   logic                       update_req,
    input   logic  [NUM_SET_W-1:0]      update_set,
    input   logic  [WAYS_PER_SET_W-1:0] update_way
 );

logic [WAYS_PER_SET_W-1:0] victim_per_set [NUM_SET-1:0];

assign victim_way = victim_per_set[victim_set];

genvar gen_it;
generate
    for (gen_it = 0; gen_it < NUM_SET; gen_it++) 
    begin :gen_set_lru

        logic [WAYS_PER_SET-1:0][WAYS_PER_SET_W-1:0]  counter;
        logic [WAYS_PER_SET-1:0][WAYS_PER_SET_W-1:0]  counter_ff;
        logic [WAYS_PER_SET_W-1:0]                max_count;
        
        //      CLK    RST    DOUT        DIN      DEF
        `RST_FF(clock, reset, counter_ff, counter, '0 )

        integer way_num,way_num_aux;
        always_comb
        begin
            counter = counter_ff;
            // Return victim ID
            if (victim_req && victim_set == gen_it )
            begin
                max_count = '0;
                victim_per_set[gen_it] = '0;
              
                // If Single Threaded then we look all the ways
                if (mt_mode == Single_Threaded)
                begin
                    for (way_num = 0; way_num < WAYS_PER_SET ; way_num++)
                    begin                        
                        if ( max_count < counter_ff[way_num] )
                        begin
                            max_count               = counter_ff[way_num];
                            victim_per_set[gen_it]  = way_num;
                        end
                    end // for
                end // if single threaded
                // If Multi Threaded then we look only the ways belonging
                // to the thread
                else 
                begin
                    for (way_num = 0; way_num < WAYS_PER_SET ; way_num++)
                    begin                        
                        if (   (way_num >= (thread_id * `NUM_WAYS_MT)) // bottom limit
                            && (way_num <  (thread_id * `NUM_WAYS_MT +`NUM_WAYS_MT)) // upper limit
                            && (max_count < counter_ff[way_num]))
                        begin
                            max_count               = counter_ff[way_num];
                            victim_per_set[gen_it]  = way_num;
                        end
                    end // for
                end //else - multithreaded
            end
        
            // Replace victim with the new line
            if (update_req && update_set == gen_it)
            begin
                // If Single Threaded then we look all the ways
                if (mt_mode == Single_Threaded)
                begin
                    for (way_num_aux = 0; way_num_aux < WAYS_PER_SET; way_num_aux++)
                    begin
                        // we increase in one the ways as they get older
                        if (counter_ff[way_num_aux] <= counter_ff[update_way])
                            counter[way_num_aux] = counter_ff[way_num_aux] + 1'b1;
                    end
                end
                // If Multi Threaded then we look only the ways belonging
                // to the thread and increase them as they get older
                else 
                begin
                    for (way_num_aux = 0; way_num_aux < WAYS_PER_SET ; way_num_aux++)
                    begin                        
                        if (   (way_num_aux >= (thread_id * `NUM_WAYS_MT)) // bottom limit
                            && (way_num_aux <  (thread_id * `NUM_WAYS_MT +`NUM_WAYS_MT)) // upper limit
                            && (max_count < counter_ff[way_num_aux]))
                        begin
                            if (counter_ff[way_num_aux] <= counter_ff[update_way])
                               counter[way_num_aux] = counter_ff[way_num_aux] + 1'b1;
                        end
                    end // for
                end //else - multithreaded

                // We reset the counter for the new block
                counter[update_way] = '0;

            end // if (update_req)
        end // always_comb
    end // for (gen_it = 0; gen_it < NUM_SET; gen_it++)
endgenerate
endmodule
