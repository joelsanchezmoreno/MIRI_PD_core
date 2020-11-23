// This module implements an arbiter, giving priority to the highest index
module arbiter_priority
#(
    parameter NUM_ENTRIES = 8,
    parameter NUM_ENTRIES_LOG = $clog2(NUM_ENTRIES)
)
(
    // client side
    input   logic [NUM_ENTRIES-1:0]     client_valid,
    input   logic [NUM_ENTRIES_LOG-1:0] top_client,
    output  logic [NUM_ENTRIES-1:0]     client_ready,
    output  logic [NUM_ENTRIES_LOG-1:0] winner,
    
    // destination side
    output  logic                       valid,
    input   logic                       ready
);
   
   assign winner = (client_valid[top_client]) ? top_client :
                                                prio_encoder(client_valid);

   always_comb
   begin
      client_ready         = {NUM_ENTRIES{1'b0}};
      client_ready[winner] = ready;
   end

   assign valid = |client_valid;
   
   function automatic [NUM_ENTRIES_LOG-1:0] prio_encoder;
      input [NUM_ENTRIES - 1:0] v;
      begin
         prio_encoder = { (NUM_ENTRIES_LOG)  {1'b0}};
         for(int i=1;i<NUM_ENTRIES;i++) begin
            if (v[i]) prio_encoder = i[NUM_ENTRIES_LOG-1:0];
         end
      end
   endfunction 

endmodule

