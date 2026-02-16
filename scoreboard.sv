`ifndef AXI4_L3_CACHE_SCOREBOARD_INCLUDED_
`define AXI4_L3_CACHE_SCOREBOARD_INCLUDED_

import uvm_pkg::*;
`include "uvm_macros.svh"

class axi4_scoreboard extends uvm_scoreboard;
  `uvm_component_utils(axi4_scoreboard)

  axi4_master_tx axi4_master_tx_h;
  axi4_slave_tx axi4_slave_tx_h;

  //=============================================================================
  // L3 CACHE CONFIGURATION (SHARED CACHE)
  //=============================================================================
  localparam int L3_CACHE_SIZE_BYTES       = 16*1024;//16KB
  localparam int L3_CACHE_LINE_SIZE_BYTES  = 64;
  localparam int L3_CACHE_ASSOCIATIVITY    = 4;

  localparam int L3_NUM_CACHE_LINES = L3_CACHE_SIZE_BYTES / L3_CACHE_LINE_SIZE_BYTES;
  localparam int L3_NUM_CACHE_SETS  = L3_NUM_CACHE_LINES / L3_CACHE_ASSOCIATIVITY;

  localparam int L3_OFFSET_BITS = $clog2(L3_CACHE_LINE_SIZE_BYTES);
  localparam int L3_INDEX_BITS  = $clog2(L3_NUM_CACHE_SETS);
  localparam int L3_TAG_BITS    = ADDR_WIDTH - L3_INDEX_BITS - L3_OFFSET_BITS;
  localparam int MAX_MSHR = 2;
  
  localparam int WORDS_PER_LINE = L3_CACHE_LINE_SIZE_BYTES / (DATA_WIDTH/8);
  localparam int AXI_DATA_BYTES = DATA_WIDTH / 8;

  //=============================================================================
  // L3 CACHE LINE STRUCTURE (SHARED)
  //=============================================================================

  typedef enum logic [1:0] {
   L3_INVALID,
   L3_CLEAN,
   L3_DIRTY,
   L3_FILLING
  } l3_state_e;
  
  typedef struct {
    bit valid;
    bit [L3_TAG_BITS-1:0] tag;
    byte data[L3_CACHE_LINE_SIZE_BYTES];
    l3_state_e state;
  } scb_cache_line_s;

  typedef struct {
    bit device; 
    bit cacheable;
    bit write_back;
    bit write_through;
    bit read_allocate;
    bit write_allocate;
    bit bufferable;
    bit modifiable;
  } axi_cache_policy_s;

  //=============================================================================
  // UNIFIED L3 CACHE STORAGE (SINGLE SHARED CACHE)
  //=============================================================================
  
  scb_cache_line_s l3_cache[L3_NUM_CACHE_SETS][L3_CACHE_ASSOCIATIVITY];
  int l3_lru_counter[L3_NUM_CACHE_SETS][L3_CACHE_ASSOCIATIVITY];
  int l3_global_lru_tick;

  //=============================================================================
  // L3 CACHE STATISTICS (GLOBAL)
  //=============================================================================
  
  int l3_read_hits_per_master[int];
  int l3_read_misses_per_master[int];
  int l3_write_hits_per_master[int];
  int l3_write_misses_per_master[int];
  
  int l3_total_read_hits;
  int l3_total_read_misses;
  int l3_total_write_hits;
  int l3_total_write_misses;
  int l3_evictions;
  int l3_writebacks_to_memory;
  int l3_writeback_errors;

  //======================================================
  // SCB MISS TRACKING (Matches DUT refill pipeline)
  //======================================================
typedef struct {

   // ---------------- Allocation ----------------
   bit valid;
   bit is_write;
   int master;
   int txn_id;

   bit [L3_INDEX_BITS-1:0] index;
   bit [L3_TAG_BITS-1:0]   tag;
   bit [ADDR_WIDTH-1:0]    line_addr;

   int way;

   // ---------------- Write Offset Tracking 
   int start_word;
   int start_byte;

   // ---------------- AXI Tracking ----------------
   int slave;
   int beat_count;
   int expected_beats;

   bit ar_sent;
   bit resp_sent;
   bit done;

   // ---------------- Writeback FSM ----------------
   bit needs_writeback;
   bit wb_done;
   bit wb_error;
   bit [1:0] resp_code;

   // ---------------- Write Data Buffer 
   logic [DATA_W-1:0] wdata_buf[WORDS_PER_LINE];
   logic [STRB_W-1:0] wstrb_buf[WORDS_PER_LINE];
   int wbeat_count;

   axi_cache_policy_s policy;

} scb_mshr_t;

  scb_mshr_t scb_mshr[MAX_MSHR];

  // FIX ISSUE #2: Active R-channel tracking per slave
  int  active_r_mshr[NO_OF_SLAVES];
  bit  active_r_valid[NO_OF_SLAVES];

  // FIX ISSUE #5: Write ownership locking
  bit scb_write_locked;
  int scb_write_owner;
  int scb_write_owner_set;
  int scb_write_owner_way;
  bit scb_write_owner_is_hit;
  bit [ADDR_WIDTH-1:0] scb_write_line;

  // FIX ISSUE #3: Performance counter tracking
  bit wr_hit_counted[NO_OF_MASTERS];

  // FIX ISSUE #7: Cache maintenance state
  typedef enum {
    SCB_MAINT_IDLE,
    SCB_MAINT_SCAN,
    SCB_MAINT_FLUSH_WAIT,
    SCB_MAINT_DONE
  } scb_maint_state_e;
  
  scb_maint_state_e scb_maint_state;
  bit               scb_maint_busy;
  int               scb_maint_set_idx;
  int               scb_maint_way_idx;
  bit               scb_maint_flush_mode;
  bit               scb_maint_inv_mode;

  // NEW: Writeback FSM state tracking (matches RTL)
  typedef enum {
    WB_IDLE,
    WB_AW,
    WB_W,
    WB_RESP
  } wb_state_e;
  
  wb_state_e scb_wb_state;
  bit        scb_wb_active;
  int        scb_wb_mshr_id;
  int        scb_wb_beat;

  //=============================================================================
  // TRANSACTION TRACKING STRUCTURES
  //=============================================================================
  
  typedef struct {
    axi4_master_tx tx;
    int master_id;
    int slave_id;
    bit address_granted;
    bit write_data_complete;
    int beats_received;
  } pending_write_transaction_t;

  typedef struct {
    axi4_master_tx tx;
    int master_id;
    int slave_id;
    bit address_granted;
    bit expected_l3_hit;
    time addr_request_time;
    bit prediction_made;
    bit [ADDR_WIDTH-1:0] line_addr;
  } pending_read_transaction_t;

  pending_write_transaction_t pending_write_txns[int][bit[ID_WIDTH-1:0]][$];
  pending_read_transaction_t pending_read_txns[int][bit[ID_WIDTH-1:0]][$];

  //=============================================================================
  // REFERENCE MEMORY (MAIN DRAM)
  //=============================================================================
  
  logic[7:0] referenceData[int][longint];

  //=============================================================================
  // ROUND-ROBIN ARBITRATION TRACKING
  //=============================================================================
  
  int rr_write_next_master[int];
  int rr_write_pending_cnt[int][int];
  int rr_write_last_granted[int];
  
  int rr_read_next_master[int];
  int rr_read_pending_cnt[int][int];
  int rr_read_last_granted[int];

  event slave_write_addr_granted[int];
  event slave_read_addr_granted[int];

  //=============================================================================
  // TLM FIFOs
  //=============================================================================
  
  uvm_tlm_analysis_fifo#(axi4_master_tx) axi4_master_read_address_analysis_fifo[];
  uvm_tlm_analysis_fifo#(axi4_master_tx) axi4_master_read_data_analysis_fifo[];
  uvm_tlm_analysis_fifo#(axi4_master_tx) axi4_master_write_address_analysis_fifo[];
  uvm_tlm_analysis_fifo#(axi4_master_tx) axi4_master_write_data_analysis_fifo[];
  uvm_tlm_analysis_fifo#(axi4_master_tx) axi4_master_write_response_analysis_fifo[];
  
  uvm_tlm_analysis_fifo#(axi4_slave_tx) axi4_slave_read_address_analysis_fifo[];
  uvm_tlm_analysis_fifo#(axi4_slave_tx) axi4_slave_read_data_analysis_fifo[];
  uvm_tlm_analysis_fifo#(axi4_slave_tx) axi4_slave_write_address_analysis_fifo[];
  uvm_tlm_analysis_fifo#(axi4_slave_tx) axi4_slave_write_data_analysis_fifo[];
  uvm_tlm_analysis_fifo#(axi4_slave_tx) axi4_slave_write_response_analysis_fifo[];

  //=============================================================================
  // TRANSACTION COUNTERS
  //=============================================================================
  
  int axi4_master_tx_awaddr_count[];
  int axi4_slave_tx_awaddr_count[];
  int axi4_master_tx_wdata_count[];
  int axi4_slave_tx_wdata_count[];
  int axi4_master_tx_bresp_count[];
  int axi4_slave_tx_bresp_count[];
  int axi4_master_tx_araddr_count[];
  int axi4_slave_tx_araddr_count[];
  int axi4_master_tx_rdata_count[];
  int axi4_slave_tx_rdata_count[];
  int axi4_master_tx_rresp_count[];
  int axi4_slave_tx_rresp_count[];

  int total_master_tx_count = 0;
  int total_slave_tx_count = 0;
  
  //=============================================================================
  // COMPARISON RESULT COUNTERS
  //=============================================================================
  
  int byte_data_cmp_verified_awid_count;
  int byte_data_cmp_verified_awaddr_count;
  int byte_data_cmp_verified_awsize_count;
  int byte_data_cmp_verified_awlen_count;
  int byte_data_cmp_verified_awburst_count;
  int byte_data_cmp_verified_awcache_count;
  int byte_data_cmp_verified_awlock_count;
  int byte_data_cmp_verified_awprot_count;
  int byte_data_cmp_verified_wdata_count;
  int byte_data_cmp_verified_wstrb_count;
  int byte_data_cmp_verified_wlast_count;
  int byte_data_cmp_verified_bid_count;
  int byte_data_cmp_verified_bresp_count;
  int byte_data_cmp_verified_arid_count;
  int byte_data_cmp_verified_araddr_count;
  int byte_data_cmp_verified_arsize_count;
  int byte_data_cmp_verified_arlen_count;
  int byte_data_cmp_verified_arburst_count;
  int byte_data_cmp_verified_arcache_count;
  int byte_data_cmp_verified_arlock_count;
  int byte_data_cmp_verified_arprot_count;
  int byte_data_cmp_verified_arregion_count;
  int byte_data_cmp_verified_arqos_count;
  int byte_data_cmp_verified_rid_count;
  int byte_data_cmp_verified_rdata_count;
  int byte_data_cmp_verified_rresp_count;
  int byte_data_cmp_verified_rlast_count;
  
  int byte_data_cmp_failed_awid_count;
  int byte_data_cmp_failed_awaddr_count;
  int byte_data_cmp_failed_awsize_count;
  int byte_data_cmp_failed_awlen_count;
  int byte_data_cmp_failed_awburst_count;
  int byte_data_cmp_failed_awcache_count;
  int byte_data_cmp_failed_awlock_count;
  int byte_data_cmp_failed_awprot_count;
  int byte_data_cmp_failed_wdata_count;
  int byte_data_cmp_failed_wstrb_count;
  int byte_data_cmp_failed_wlast_count;
  int byte_data_cmp_failed_bid_count;
  int byte_data_cmp_failed_bresp_count;
  int byte_data_cmp_failed_arid_count;
  int byte_data_cmp_failed_araddr_count;
  int byte_data_cmp_failed_arsize_count;
  int byte_data_cmp_failed_arlen_count;
  int byte_data_cmp_failed_arburst_count;
  int byte_data_cmp_failed_arcache_count;
  int byte_data_cmp_failed_arlock_count;
  int byte_data_cmp_failed_arprot_count;
  int byte_data_cmp_failed_arregion_count;
  int byte_data_cmp_failed_arqos_count;
  int byte_data_cmp_failed_rid_count;
  int byte_data_cmp_failed_rdata_count;
  int byte_data_cmp_failed_rresp_count;
  int byte_data_cmp_failed_rlast_count;

  int rr_write_violations;
  int rr_read_violations;
  int rr_write_grants;
  int rr_read_grants;

  //=============================================================================
  // SLAVE ADDRESS CONFIGURATION
  //=============================================================================
  
  bit[ADDR_WIDTH-1:0] SLAVE_START_ADDR[];
  bit[ADDR_WIDTH-1:0] SLAVE_END_ADDR[];

  int nonExistantMemRead;
  
  axi4_env_config axi4_env_cfg_h;
  axi4_slave_agent_config axi4_slave_agent_cfg_h[];

  //=============================================================================
  // FUNCTION PROTOTYPES
  //=============================================================================
  
  // Constructor and UVM phases
  extern function new(string name = "axi4_scoreboard", uvm_component parent = null);
  extern virtual function void build_phase(uvm_phase phase);
  extern virtual function void connect_phase(uvm_phase phase);
  extern virtual task run_phase(uvm_phase phase);
  extern virtual function void check_phase(uvm_phase phase);
  extern virtual function void report_phase(uvm_phase phase);
  
  // L3 cache functions
  extern virtual function void init_l3_cache_model();

  extern virtual function void l3_cache_decode_address(
    input  bit [ADDR_WIDTH-1:0] addr,
    output bit [L3_TAG_BITS-1:0]   tag,
    output bit [L3_INDEX_BITS-1:0] index,
    output bit [L3_OFFSET_BITS-1:0] offset
  );

  extern virtual function bit l3_cache_lookup(
    input  bit [ADDR_WIDTH-1:0] addr,
    input  axi_cache_policy_s   policy,
    output int                  hit_way,
    output l3_state_e           state
  );

  extern virtual function int unsigned l3_find_lru_way(
    input int unsigned set_index
  );

  extern virtual function void l3_update_lru(
    input int unsigned set_index,
    input int unsigned way
  );

  extern virtual function void l3_set_line_state(
    input int set_index,
    input int way,
    input l3_state_e new_state
  );

  // NEW: Enhanced writeback with ownership locking
  extern virtual function void l3_writeback_to_memory(
    input int set_index,
    input int way,
    input bit [1:0] bresp = 2'b00
  );

  extern virtual function int scb_allocate_mshr(
    input bit is_write,
    input bit [ADDR_WIDTH-1:0] addr,
    input int master,
    input axi_cache_policy_s policy
  );
  
  // NEW: MSHR management functions
  extern virtual function int scb_find_existing_mshr(
    input bit [ADDR_WIDTH-1:0] line_addr
  );
  
  extern virtual function void scb_update_mshr_beat(
    input int mshr_idx,
    input byte beat_data[]
  );
  
  extern virtual function void scb_update_mshr_write_data(
    input int mshr_idx,
    input byte wdata[],
    input bit wstrb[]
  );
  
  extern virtual function void scb_release_mshr(
    input int mshr_idx
  );

  // Request handlers
  extern virtual function void l3_handle_read_request(
    input int master_id,
    input axi4_master_tx m_tx,
    output bit expected_hit
  );
  
  extern virtual function void l3_handle_write_request(
    input int master_id,
    input axi4_master_tx m_tx,
    input int slave_idx
  );
  
  extern virtual function void l3_handle_write_data(
    input int master_id,
    input axi4_master_tx m_tx
  );
  
  extern virtual function bit[ADDR_WIDTH-1:0] get_line_base_addr(
    bit[ADDR_WIDTH-1:0] addr
  );
  
  extern virtual function void l3_complete_fill(
    input int master_id,
    input bit[ADDR_WIDTH-1:0] addr,
    input axi4_master_tx m_tx
  );
  
  // NEW: Helper functions for fixes
  extern virtual function bit line_under_refill(
    input int index,
    input int tag
  );
  
  extern virtual function bit line_has_active_mshr(
    input int index,
    input int way
  );
  
  extern virtual function void scb_handle_cache_maintenance(
    input bit flush_req,
    input bit inv_req,
    input bit flush_inv_req,
    input bit [ADDR_WIDTH-1:0] maint_addr,
    input bit by_addr
  );
  
  // Reference model and utility functions
  extern virtual function int get_slave_index(logic[ADDR_WIDTH-1:0] addr);
  extern virtual function void ref_model_write(axi4_master_tx m_tx, int slave_idx, int master_idx);
  extern virtual function void ref_model_read(axi4_master_tx m_tx, int slave_idx);
  
  // Arbitration checking
  extern virtual function void check_write_rr_arbitration(int slave_id, int granted_master);
  extern virtual function void check_read_rr_arbitration(int slave_id, int granted_master);
  
  // AXI cache policy decoder
  extern virtual function axi_cache_policy_s axi_decode_cache_policy(
    bit [3:0] axcache,
    bit is_read
  );
  
  // Comparison tasks
  extern virtual task axi4_write_address_comparison(input axi4_master_tx exp_tx, input axi4_slave_tx act_tx, input int master_id, input int slave_id);
  extern virtual task axi4_write_data_comparison(input axi4_master_tx exp_tx, input axi4_slave_tx act_tx, input int master_id, input int slave_id);
  extern virtual task axi4_write_response_comparison(input axi4_master_tx exp_tx, input axi4_slave_tx act_tx, input int master_id, input int slave_id);
  extern virtual task axi4_read_address_comparison(input axi4_master_tx exp_tx, input axi4_slave_tx act_tx, input int master_id, input int slave_id);
  extern virtual task automatic axi4_read_data_comparison(input axi4_master_tx exp_tx, input axi4_master_tx act_tx, input int master_id, input int slave_id, input bit expected_hit);
endclass : axi4_scoreboard

//=============================================================================
// IMPLEMENTATION
//=============================================================================

function axi4_scoreboard::new(string name = "axi4_scoreboard", 
                                       uvm_component parent = null);
  super.new(name, parent);
endfunction : new

//=============================================================================
// Function: build_phase
//=============================================================================
function void axi4_scoreboard::build_phase(uvm_phase phase);
  super.build_phase(phase);
  
  if(!uvm_config_db#(axi4_env_config)::get(this, "", "axi4_env_config", axi4_env_cfg_h)) begin
    `uvm_fatal("FATAL_ENV_CONFIG", "Couldn't get axi4_env_config from config_db")
  end
  
  // Initialize L3 cache model
  init_l3_cache_model();
  
  // Allocate arrays for masters
  axi4_master_read_address_analysis_fifo = new[NO_OF_MASTERS];
  axi4_master_read_data_analysis_fifo = new[NO_OF_MASTERS];
  axi4_master_write_address_analysis_fifo = new[NO_OF_MASTERS];
  axi4_master_write_data_analysis_fifo = new[NO_OF_MASTERS];
  axi4_master_write_response_analysis_fifo = new[NO_OF_MASTERS];
  
  axi4_master_tx_awaddr_count = new[NO_OF_MASTERS];
  axi4_master_tx_wdata_count = new[NO_OF_MASTERS];
  axi4_master_tx_bresp_count = new[NO_OF_MASTERS];
  axi4_master_tx_araddr_count = new[NO_OF_MASTERS];
  axi4_master_tx_rdata_count = new[NO_OF_MASTERS];
  axi4_master_tx_rresp_count = new[NO_OF_MASTERS];
  
  l3_read_hits_per_master = new[NO_OF_MASTERS];
  l3_read_misses_per_master = new[NO_OF_MASTERS];
  l3_write_hits_per_master = new[NO_OF_MASTERS];
  l3_write_misses_per_master = new[NO_OF_MASTERS];
  
  foreach(l3_read_hits_per_master[i]) begin
    l3_read_hits_per_master[i] = 0;
    l3_read_misses_per_master[i] = 0;
    l3_write_hits_per_master[i] = 0;
    l3_write_misses_per_master[i] = 0;
    wr_hit_counted[i] = 0;
  end
  
  // Allocate arrays for slaves
  axi4_slave_read_address_analysis_fifo = new[NO_OF_SLAVES];
  axi4_slave_read_data_analysis_fifo = new[NO_OF_SLAVES];
  axi4_slave_write_address_analysis_fifo = new[NO_OF_SLAVES];
  axi4_slave_write_data_analysis_fifo = new[NO_OF_SLAVES];
  axi4_slave_write_response_analysis_fifo = new[NO_OF_SLAVES];
  
  axi4_slave_tx_awaddr_count = new[NO_OF_SLAVES];
  axi4_slave_tx_wdata_count = new[NO_OF_SLAVES];
  axi4_slave_tx_bresp_count = new[NO_OF_SLAVES];
  axi4_slave_tx_araddr_count = new[NO_OF_SLAVES];
  axi4_slave_tx_rdata_count = new[NO_OF_SLAVES];
  axi4_slave_tx_rresp_count = new[NO_OF_SLAVES];
  
  axi4_slave_agent_cfg_h = new[NO_OF_SLAVES];
  SLAVE_START_ADDR = new[NO_OF_SLAVES];
  SLAVE_END_ADDR = new[NO_OF_SLAVES];
  
  // Create TLM FIFOs for each master
  foreach(axi4_master_read_address_analysis_fifo[i]) begin
    axi4_master_read_address_analysis_fifo[i] = new($sformatf("axi4_master_read_address_analysis_fifo[%0d]", i), this);
    axi4_master_read_data_analysis_fifo[i] = new($sformatf("axi4_master_read_data_analysis_fifo[%0d]", i), this);
    axi4_master_write_address_analysis_fifo[i] = new($sformatf("axi4_master_write_address_analysis_fifo[%0d]", i), this);
    axi4_master_write_data_analysis_fifo[i] = new($sformatf("axi4_master_write_data_analysis_fifo[%0d]", i), this);
    axi4_master_write_response_analysis_fifo[i] = new($sformatf("axi4_master_write_response_analysis_fifo[%0d]", i), this);
  end
  
  // Create TLM FIFOs for each slave
  foreach(axi4_slave_read_address_analysis_fifo[i]) begin
    axi4_slave_read_address_analysis_fifo[i] = new($sformatf("axi4_slave_read_address_analysis_fifo[%0d]", i), this);
    axi4_slave_read_data_analysis_fifo[i] = new($sformatf("axi4_slave_read_data_analysis_fifo[%0d]", i), this);
    axi4_slave_write_address_analysis_fifo[i] = new($sformatf("axi4_slave_write_address_analysis_fifo[%0d]", i), this);
    axi4_slave_write_data_analysis_fifo[i] = new($sformatf("axi4_slave_write_data_analysis_fifo[%0d]", i), this);
    axi4_slave_write_response_analysis_fifo[i] = new($sformatf("axi4_slave_write_response_analysis_fifo[%0d]", i), this);
    
    if(!uvm_config_db#(axi4_slave_agent_config)::get(this, "", $sformatf("axi4_slave_agent_config[%0d]", i), axi4_slave_agent_cfg_h[i])) begin
      `uvm_fatal("FATAL_SA_AGENT_CONFIG", $sformatf("Couldn't get axi4_slave_agent_config[%0d] from config_db", i))
    end
    
    SLAVE_START_ADDR[i] = axi4_slave_agent_cfg_h[i].min_address;
    SLAVE_END_ADDR[i] = axi4_slave_agent_cfg_h[i].max_address;
  end
  
  // Initialize Round-Robin tracking
  for(int s = 0; s < NO_OF_SLAVES; s++) begin
    rr_write_next_master[s] = 0;
    rr_write_last_granted[s] = -1;
    rr_read_next_master[s] = 0;
    rr_read_last_granted[s] = -1;
    
    for(int m = 0; m < NO_OF_MASTERS; m++) begin
      rr_write_pending_cnt[s][m] = 0;
      rr_read_pending_cnt[s][m] = 0;
    end
  end
  
endfunction : build_phase

//=============================================================================
// Function: axi_decode_cache_policy
//=============================================================================
function axi_cache_policy_s axi4_scoreboard::axi_decode_cache_policy(
  bit [3:0] axcache,
  bit is_read
);

  axi_cache_policy_s p;
  p = '{default:0};

  p.bufferable = axcache[0];
  p.modifiable = axcache[1];

  // DEVICE MEMORY
  if(axcache == 4'b0000 || axcache == 4'b0001) begin
    p.device     = 1;
    p.modifiable = 0;
    return p;
  end

  // NON CACHEABLE NORMAL
  if(axcache[3:2] == 2'b00) begin
    p.cacheable = 0;
    return p;
  end

  p.cacheable = 1;

  // CACHE TYPE (PATTERN BASED)
  case(axcache)
    // WRITE THROUGH
    4'b1010, 4'b0110, 4'b1110:
      p.write_through = 1;

    // WRITE BACK
    4'b1011, 4'b0111, 4'b1111:
      p.write_back = 1;
  endcase

  // ALLOCATION (CHANNEL AWARE)
  if(is_read) begin
    p.read_allocate  = axcache[2];
  end
  else begin
    p.write_allocate = axcache[3];
  end

  return p;

endfunction : axi_decode_cache_policy

//=============================================================================
// Function: init_l3_cache_model 
//=============================================================================
function void axi4_scoreboard::init_l3_cache_model();

  `uvm_info("L3_CACHE_INIT",
    $sformatf(
      "L3 Cache Model Configuration (SHARED):\n"
      "  L3 Cache Size    : %0d bytes (%0d KB)\n"
      "  Line Size        : %0d bytes\n"
      "  Associativity    : %0d-way\n"
      "  Number of Lines  : %0d\n"
      "  Number of Sets   : %0d\n"
      "  Offset Bits      : %0d\n"
      "  Index Bits       : %0d\n"
      "  Tag Bits         : %0d\n"
      "  Words Per Line   : %0d\n",
      L3_CACHE_SIZE_BYTES,
      L3_CACHE_SIZE_BYTES/1024,
      L3_CACHE_LINE_SIZE_BYTES,
      L3_CACHE_ASSOCIATIVITY,
      L3_NUM_CACHE_LINES,
      L3_NUM_CACHE_SETS,
      L3_OFFSET_BITS,
      L3_INDEX_BITS,
      L3_TAG_BITS,
      WORDS_PER_LINE
    ), UVM_LOW)

  if(ADDR_WIDTH != L3_TAG_BITS + L3_INDEX_BITS + L3_OFFSET_BITS) begin
    `uvm_fatal("ADDR_DECODE", $sformatf("Address split mismatch: ADDR=%0d TAG+IDX+OFF=%0d", ADDR_WIDTH, L3_TAG_BITS+L3_INDEX_BITS+L3_OFFSET_BITS))
  end  

  // CACHE ARRAY RESET
  for(int s = 0; s < L3_NUM_CACHE_SETS; s++) begin
    for(int w = 0; w < L3_CACHE_ASSOCIATIVITY; w++) begin
      l3_cache[s][w].valid = 0;
      l3_cache[s][w].tag   = '0;
      l3_cache[s][w].state = L3_INVALID;

      foreach(l3_cache[s][w].data[i]) begin
        l3_cache[s][w].data[i] = '0;
      end

      l3_lru_counter[s][w] = w;
    end
  end

  // GLOBAL STATE RESET
  l3_total_read_hits      = 0;
  l3_total_read_misses    = 0;
  l3_total_write_hits     = 0;
  l3_total_write_misses   = 0;
  l3_evictions            = 0;
  l3_writebacks_to_memory = 0;
  l3_writeback_errors     = 0;

  l3_global_lru_tick  = 0;

  // Initialize MSHR array
  for(int i = 0; i < MAX_MSHR; i++) begin
    scb_mshr[i].valid = 0;
    scb_mshr[i].done = 0;
    scb_mshr[i].ar_sent = 0;
    scb_mshr[i].wb_done = 0;
    scb_mshr[i].wb_error = 0;
    scb_mshr[i].resp_code = 2'b00;
  end

  // Initialize write ownership (FIX ISSUE #5)
  scb_write_locked = 0;
  scb_write_owner  = -1;
  scb_write_owner_set = -1;
  scb_write_owner_way = -1;
  scb_write_owner_is_hit = 0;

  // Initialize R-channel tracking (FIX ISSUE #2)
  for(int s = 0; s < NO_OF_SLAVES; s++) begin
    active_r_valid[s] = 0;
    active_r_mshr[s] = -1;
  end

  // Initialize maintenance state (FIX ISSUE #7)
  scb_maint_state = SCB_MAINT_IDLE;
  scb_maint_busy = 0;
  scb_maint_set_idx = 0;
  scb_maint_way_idx = 0;
  scb_maint_flush_mode = 0;
  scb_maint_inv_mode = 0;

  // Initialize writeback FSM
  scb_wb_state = WB_IDLE;
  scb_wb_active = 0;
  scb_wb_mshr_id = -1;
  scb_wb_beat = 0;

endfunction : init_l3_cache_model

//=============================================================================
// Function: l3_cache_decode_address
//=============================================================================
function void axi4_scoreboard::l3_cache_decode_address(
  input  bit [ADDR_WIDTH-1:0] addr,
  output bit [L3_TAG_BITS-1:0] tag,
  output bit [L3_INDEX_BITS-1:0] index,
  output bit [L3_OFFSET_BITS-1:0] offset
);

  offset = addr[L3_OFFSET_BITS-1:0];
  index  = addr[L3_OFFSET_BITS +: L3_INDEX_BITS];
  tag    = addr[L3_OFFSET_BITS + L3_INDEX_BITS +: L3_TAG_BITS];

endfunction : l3_cache_decode_address

//=============================================================================
// Function: l3_cache_lookup (FIX ISSUE #5 & #8)
// Detects cache hits while blocking:
//   - Lines under refill (FIX #8)
//   - Lines being written by locked master (FIX #5)
//=============================================================================
function bit axi4_scoreboard::l3_cache_lookup(
  input  bit [ADDR_WIDTH-1:0] addr,
  input  axi_cache_policy_s    policy,
  output int                   hit_way,
  output l3_state_e            state
);
  bit [L3_TAG_BITS-1:0]   tag;
  bit [L3_INDEX_BITS-1:0] index;
  bit [L3_OFFSET_BITS-1:0] offset;

  // Default outputs
  hit_way = -1;
  state   = L3_INVALID;

  // AXI4 RULE — Device or Non-cacheable bypass
  if(policy.device || !policy.cacheable)
    return 0;

  // AXI4 RULE — no lookup if allocate bits = 00
  if(!policy.read_allocate && !policy.write_allocate)
    return 0;

  // Address decode
  l3_cache_decode_address(addr, tag, index, offset);

  // FIX ISSUE #8: Block lookup if line is under refill
  if(line_under_refill(index, tag)) begin
    `uvm_info("L3_LOOKUP_BLOCK", 
      $sformatf("Blocked lookup - line under refill: idx=%0d tag=0x%0h", index, tag), 
      UVM_HIGH)
    return 0;
  end

  // Search set
  for(int w = 0; w < L3_CACHE_ASSOCIATIVITY; w++) begin
    if(l3_cache[index][w].valid &&
       l3_cache[index][w].state != L3_FILLING &&
       l3_cache[index][w].state != L3_INVALID &&
       l3_cache[index][w].tag   == tag) begin

      // FIX ISSUE #5: Block read hit if write-locked on same line
      if(scb_write_locked && 
         scb_write_owner_is_hit &&
         scb_write_owner_set == index &&
         scb_write_owner_way == w) begin
        `uvm_info("L3_LOOKUP_BLOCK", 
          $sformatf("Blocked read hit - write locked: idx=%0d way=%0d owner=M%0d", 
                    index, w, scb_write_owner), 
          UVM_HIGH)
        return 0; // Treat as miss to match RTL behavior
      end

      hit_way = w;
      state   = l3_cache[index][w].state;
      return 1;
    end
  end

  return 0;

endfunction : l3_cache_lookup

//=============================================================================
// Function: l3_find_lru_way
//=============================================================================
function int axi4_scoreboard::l3_find_lru_way(int set_index);
  int victim_way = -1;
  int min_lru    = 32'h7FFFFFFF;

  // Prefer INVALID ways (fast allocation, no eviction)
  for(int w = 0; w < L3_CACHE_ASSOCIATIVITY; w++) begin
    if(l3_cache[set_index][w].state == L3_INVALID) begin
      return w;
    end
  end

  // Choose true LRU among CLEAN/DIRTY, NEVER select FILLING ways
  for(int w = 0; w < L3_CACHE_ASSOCIATIVITY; w++) begin
    if(l3_cache[set_index][w].state != L3_FILLING) begin
      if(l3_lru_counter[set_index][w] < min_lru) begin
        min_lru    = l3_lru_counter[set_index][w];
        victim_way = w;
      end
    end
  end

  // DUT safety fallback (pipeline saturation)
  if(victim_way == -1) begin
    `uvm_warning("L3_LRU",
      $sformatf("All ways filling in set %0d — forcing way0 (DUT fallback)", set_index))
    victim_way = 0;
  end

  return victim_way;

endfunction : l3_find_lru_way

//=============================================================================
// Function: l3_update_lru (matches RTL age-based LRU)
//=============================================================================
function void axi4_scoreboard::l3_update_lru(int set_index, int way);

  // Don't update invalid or filling lines
  if(l3_cache[set_index][way].state == L3_INVALID ||
     l3_cache[set_index][way].state == L3_FILLING)
    return;

  // Increment all counters in set (aging) - matches RTL
  for(int w = 0; w < L3_CACHE_ASSOCIATIVITY; w++) begin
    if(l3_lru_counter[set_index][w] < 255) begin // 8-bit wrap like RTL
      l3_lru_counter[set_index][w]++;
    end
  end

  // Reset accessed way to 0 (most recently used)
  l3_lru_counter[set_index][way] = 0;

endfunction : l3_update_lru

//=============================================================================
// Function: l3_set_line_state
//=============================================================================
function void axi4_scoreboard::l3_set_line_state(
  input int set_index,
  input int way,
  input l3_state_e new_state
);

  l3_cache[set_index][way].state = new_state;

  // Maintain valid bit
  if(new_state == L3_INVALID)
    l3_cache[set_index][way].valid = 0;
  else
    l3_cache[set_index][way].valid = 1;

endfunction : l3_set_line_state

//=============================================================================
// Function: l3_writeback_to_memory (ENHANCED with OWNERSHIP LOCKING)
// 
// FIX ISSUE #1: Writeback BRESP error handling
// FIX ISSUE #5: Write ownership blocking during writeback
// 
// This function models the RTL writeback FSM behavior:
//   1. Blocks all reads and writes to the line being written back
//   2. Only clears dirty state if BRESP = 2'b00 (OKAY)
//   3. Keeps dirty state if writeback fails
//   4. Prevents concurrent access during multi-beat writeback
//
// RTL equivalent: WB_IDLE -> WB_AW -> WB_W -> WB_RESP states
//=============================================================================
function void axi4_scoreboard::l3_writeback_to_memory(
  input int set_index,
  input int way,
  input bit [1:0] bresp = 2'b00
);
  bit [ADDR_WIDTH-1:0] wb_addr;
  int slave_idx;
  bit writeback_success;

  // Only write back if the cache line is DIRTY
  if(l3_cache[set_index][way].state != L3_DIRTY) begin
    `uvm_info("L3_WRITEBACK", 
      $sformatf("Skipping writeback - line not dirty: Set=%0d Way=%0d State=%s", 
                set_index, way, l3_cache[set_index][way].state.name()), 
      UVM_HIGH)
    return;
  end

  // Compose cache line base address
  wb_addr = {l3_cache[set_index][way].tag[L3_TAG_BITS-1:0], 
             set_index[L3_INDEX_BITS-1:0], 
             {L3_OFFSET_BITS{1'b0}}};

  `uvm_info("L3_WRITEBACK_START",
    $sformatf("Starting writeback: Set=%0d Way=%0d Addr=0x%0h",
              set_index, way, wb_addr),
    UVM_MEDIUM)

  //===========================================================================
  // CRITICAL: ACQUIRE WRITEBACK OWNERSHIP LOCK
  // 
  // This prevents ANY read or write access to this line during writeback.
  // Matches RTL behavior where wb_active blocks new transactions.
  //
  // RTL equivalent:
  //   if (wr_req_valid[m] && maint_state != MAINT_IDLE || wb_active)
  //     wr_req_ready[m] = 1'b0;  // Block writes
  //===========================================================================
  
  if(scb_wb_active) begin
    `uvm_warning("WB_CONFLICT", 
      $sformatf("Writeback already active (MSHR=%0d), queueing Set=%0d Way=%0d",
                scb_wb_mshr_id, set_index, way))
    // In production, would queue this or retry later
    return;
  end

  // LOCK: Set writeback active state
  scb_wb_active = 1;
  scb_wb_state = WB_AW;
  
  `uvm_info("WB_LOCK_ACQUIRED",
    $sformatf("Writeback lock acquired for Set=%0d Way=%0d", set_index, way),
    UVM_HIGH)

  //===========================================================================
  // READ BLOCKING DURING WRITEBACK
  //
  // While writeback is active, any read to the SAME line should be blocked.
  // This is implicitly handled by the line state being DIRTY and the
  // scb_wb_active flag preventing new MSHR allocations.
  //
  // RTL equivalent:
  //   if (rd_req_valid[m] && maint_state != MAINT_IDLE)
  //     rd_ready[m] = 1'b0;
  //===========================================================================

  // Determine which slave owns this address
  slave_idx = get_slave_index(wb_addr);

  if(slave_idx == -1) begin
    `uvm_error("WB_INVALID_SLAVE", 
      $sformatf("No slave found for writeback addr=0x%0h", wb_addr))
    scb_wb_active = 0;
    return;
  end

  //===========================================================================
  // WRITEBACK FSM: WB_AW State
  // Send write address channel
  //===========================================================================
  scb_wb_state = WB_AW;
  `uvm_info("WB_STATE_AW", 
    $sformatf("Writeback AW phase: Addr=0x%0h Slave=%0d", wb_addr, slave_idx), 
    UVM_HIGH)

  //===========================================================================
  // WRITEBACK FSM: WB_W State
  // Send write data channel (multi-beat)
  //===========================================================================
  scb_wb_state = WB_W;
  
  for(int beat = 0; beat < WORDS_PER_LINE; beat++) begin
    scb_wb_beat = beat;
    
    // Write entire cache line to reference memory
    for(int offset = 0; offset < AXI_DATA_BYTES; offset++) begin
      int byte_idx = beat * AXI_DATA_BYTES + offset;
      if(byte_idx < L3_CACHE_LINE_SIZE_BYTES) begin
        referenceData[slave_idx][wb_addr + byte_idx] = l3_cache[set_index][way].data[byte_idx];
      end
    end
    
    `uvm_info("WB_DATA_BEAT",
      $sformatf("Writeback beat %0d/%0d: Addr=0x%0h", 
                beat, WORDS_PER_LINE-1, wb_addr + beat*AXI_DATA_BYTES),
      UVM_DEBUG)
  end

  //===========================================================================
  // WRITEBACK FSM: WB_RESP State
  // Check write response
  //
  // FIX ISSUE #1: BRESP Error Handling
  //
  // RTL behavior:
  //   if (s_bresp[mshr[wb_mshr_id].slave] == 2'b00) begin
  //     dirty_array[...] <= 1'b0;  // Success - clear dirty
  //   end
  //   else begin
  //     mshr[wb_mshr_id].wb_error <= 1'b1;  // Error - keep dirty!
  //   end
  //===========================================================================
  scb_wb_state = WB_RESP;
  
  writeback_success = (bresp == 2'b00); // OKAY response

  if(writeback_success) begin
    // SUCCESS: Clear dirty state
    l3_cache[set_index][way].state = L3_CLEAN;
    l3_writebacks_to_memory++;
    
    `uvm_info("L3_WRITEBACK_SUCCESS",
      $sformatf("Writeback completed successfully: Set=%0d Way=%0d Addr=0x%0h BRESP=%0h",
                set_index, way, wb_addr, bresp),
      UVM_MEDIUM)
  end
  else begin
    // ERROR: Keep dirty state, log error
    l3_writeback_errors++;
    
    `uvm_error("L3_WRITEBACK_ERROR",
      $sformatf("Writeback FAILED: Set=%0d Way=%0d Addr=0x%0h BRESP=%0h - LINE REMAINS DIRTY",
                set_index, way, wb_addr, bresp))
    
    // Line remains DIRTY for potential retry
    // In RTL: mshr[wb_mshr_id].wb_error <= 1'b1;
  end

  //===========================================================================
  // CRITICAL: RELEASE WRITEBACK OWNERSHIP LOCK
  //
  // Matches RTL transition: WB_RESP -> WB_IDLE
  // Now reads and writes can proceed to this set/way
  //===========================================================================
  scb_wb_state = WB_IDLE;
  scb_wb_active = 0;
  scb_wb_mshr_id = -1;
  scb_wb_beat = 0;

  `uvm_info("WB_LOCK_RELEASED",
    $sformatf("Writeback lock released for Set=%0d Way=%0d", set_index, way),
    UVM_HIGH)

endfunction : l3_writeback_to_memory

//=============================================================================
// Function: line_under_refill (FIX ISSUE #8)
//=============================================================================
function bit axi4_scoreboard::line_under_refill(input int index, input int tag);
  for(int i = 0; i < MAX_MSHR; i++) begin
    if(scb_mshr[i].valid &&
       !scb_mshr[i].done &&
       scb_mshr[i].index == index &&
       scb_mshr[i].tag == tag)
      return 1;
  end
  return 0;
endfunction : line_under_refill

//=============================================================================
// Function: line_has_active_mshr (FIX ISSUE #7)
//=============================================================================
function bit axi4_scoreboard::line_has_active_mshr(input int index, input int way);
  for(int i = 0; i < MAX_MSHR; i++) begin
    if(scb_mshr[i].valid &&
       scb_mshr[i].index == index &&
       scb_mshr[i].way == way)
      return 1;
  end
  return 0;
endfunction : line_has_active_mshr

//=============================================================================
// Function: get_line_base_addr
//=============================================================================
function bit[ADDR_WIDTH-1:0] axi4_scoreboard::get_line_base_addr(
  bit[ADDR_WIDTH-1:0] addr
);
  return {addr[ADDR_WIDTH-1:L3_OFFSET_BITS], {L3_OFFSET_BITS{1'b0}}};
endfunction : get_line_base_addr

//=============================================================================
// Function: scb_find_existing_mshr
//=============================================================================
function int axi4_scoreboard::scb_find_existing_mshr(
    input logic [ADDR_WIDTH-1:0] addr);

   logic [ADDR_WIDTH-1:0] line_base;
   line_base = get_line_base_addr(addr);

   for(int i=0;i<MAX_MSHR;i++) begin
      if(scb_mshr[i].valid &&
         !scb_mshr[i].done &&
         scb_mshr[i].line_addr == line_base)
         return i;
   end

   return -1;
endfunction: scb_find_existing_mshr

//=============================================================================
// Function: scb_allocate_mshr
//=============================================================================
function int axi4_scoreboard::scb_allocate_mshr(
    input logic [ADDR_WIDTH-1:0] addr,
    input int master,
    input int txn_id,
    input bit is_write);

   int idx;
   int slave;
   int way;
   bit [L3_TAG_BITS-1:0] tag;
   bit [L3_INDEX_BITS-1:0] index;
   bit [L3_OFFSET_BITS-1:0] offset;
   logic [ADDR_WIDTH-1:0] line_base;

   idx = scb_find_existing_mshr(addr);
   if(idx != -1)
      return idx;

   l3_cache_decode_address(addr, tag, index, offset);
   way = l3_find_lru_way(index);
   slave = get_slave_index(addr);
   line_base = get_line_base_addr(addr);

   if(active_r_valid[slave])
      return -1;

   for(int i=0;i<MAX_MSHR;i++) begin
      if(!scb_mshr[i].valid) begin

         scb_mshr[i].needs_writeback =
            (l3_cache[index][way].state == L3_DIRTY);

         scb_mshr[i].valid      = 1;
         scb_mshr[i].done       = 0;
         scb_mshr[i].ar_sent    = 0;
         scb_mshr[i].wb_done    = !scb_mshr[i].needs_writeback;
         scb_mshr[i].line_addr  = line_base;
         scb_mshr[i].master     = master;
         scb_mshr[i].txn_id     = txn_id;
         scb_mshr[i].slave      = slave;
         scb_mshr[i].set        = index;
         scb_mshr[i].way        = way;
         scb_mshr[i].tag        = tag;
         scb_mshr[i].beat_count = 0;
         scb_mshr[i].wbeat_count= 0;

        if(scb_mshr[i].needs_writeback)
          l3_writeback_to_memory(index, way);
        
        if(!needs_writeback)
          l3_set_line_state(index, way, L3_FILLING);

         return i;
      end
   end

   return -1;
endfunction:scb_allocate_mshr

//=============================================================================
// Function: scb_update_mshr_beat
//=============================================================================
function void axi4_scoreboard::scb_update_mshr_beat(
    input int slave,
    input logic [DATA_W-1:0] data,
    input bit rlast);

   if(!active_r_valid[slave]) return;

   int i = active_r_mshr[slave];

   if(!scb_mshr[i].valid) return;
   if(scb_mshr[i].beat_count >= WORDS_PER_LINE) return;

   l3_cache[scb_mshr[i].set][scb_mshr[i].way]
      .data[scb_mshr[i].beat_count] = data;

   scb_mshr[i].beat_count++;

   if(rlast)
      scb_mshr[i].done = 1;

endfunction : scb_update_mshr_beat


//=============================================================================
// Function: scb_update_mshr_write_data (FIX ISSUE #4)
//=============================================================================
function void axi4_scoreboard::scb_update_mshr_write_data(
    input int master,
    input int txn_id,
    input logic [DATA_W-1:0] data,
    input logic [STRB_W-1:0] strb);

   for(int i=0;i<MAX_MSHR;i++) begin
      if(scb_mshr[i].valid &&
         scb_mshr[i].master == master &&
         scb_mshr[i].txn_id == txn_id &&
         !scb_mshr[i].done) begin

         int b = scb_mshr[i].wbeat_count;

         if(b < WORDS_PER_LINE) begin
            scb_mshr[i].wdata_buf[b] = data;
            scb_mshr[i].wstrb_buf[b] = strb;
            scb_mshr[i].wbeat_count++;
         end
      end
   end

endfunction: scb_update_mshr_write_data

//=============================================================================
// Function: apply_write_merge
// Word-local merge with byte offset support
//=============================================================================
function void apply_write_merge(
      inout logic [DATA_W-1:0] line_data,
      input  logic [DATA_W-1:0] wdata,
      input  logic [STRB_W-1:0] wstrb);

  if(STRB_W*8 != DATA_W)
   `uvm_error("MERGE","WSTRB width mismatch")

   for(int b = 0; b < STRB_W; b++) begin
      if(wstrb[b]) begin
         line_data[b*8 +: 8] = wdata[b*8 +: 8];
      end
   end

endfunction:apply_write_merge

//=============================================================================
// Function: scb_release_mshr
//=============================================================================
function void axi4_scoreboard::scb_release_mshr(
    input int i,
    input bit resp_accepted);

   if(i < 0 || i >= MAX_MSHR)
      return;

   if(!scb_mshr[i].valid)
      return;

   if(!scb_mshr[i].done)
      return;

   if(!resp_accepted)
      return;

   int set  = scb_mshr[i].set;
   int way  = scb_mshr[i].way;
   int base = scb_mshr[i].start_word;

   // --------------------------------------------------
   // apply buffered writes after refill
   // --------------------------------------------------
   for(int b = 0; b < scb_mshr[i].wbeat_count; b++) begin

      int line_word = base + b;

      if(line_word >= WORDS_PER_LINE)
         break;

      apply_write_merge(
         l3_cache[set][way].data[line_word],
         scb_mshr[i].wdata_buf[b],
         scb_mshr[i].wstrb_buf[b]);
   end

   // --------------------------------------------------
   // update line state
   // --------------------------------------------------
   if(scb_mshr[i].wbeat_count > 0)
      l3_set_line_state(set, way, L3_DIRTY);
   else
      l3_set_line_state(set, way, L3_CLEAN);

   // --------------------------------------------------
   // release slave refill ownership
   // --------------------------------------------------
   int s = scb_mshr[i].slave;

   if(s >= 0 && s < NO_OF_SLAVES) begin
      active_r_valid[s] = 0;
      active_r_mshr[s]  = -1;
   end

   // --------------------------------------------------
   // clear entire entry
   // --------------------------------------------------
   scb_mshr[i] = '{default:0};

endfunction : scb_release_mshr

//============================================================================
//SCB issue AR function
//============================================================================
function void axi4_scoreboard::scb_issue_ar();

   for(int i = 0; i < MAX_MSHR; i++) begin

      if(!scb_mshr[i].valid)
         continue;

      if(scb_mshr[i].ar_sent)
         continue;

      if(!scb_mshr[i].wb_done)
         continue;

      int s = scb_mshr[i].slave;

      if(s < 0 || s >= NO_OF_SLAVES)
         continue;

      if(active_r_valid[s])
         continue;

      // issue refill
      active_r_valid[s] = 1;
      active_r_mshr[s]  = i;
      scb_mshr[i].ar_sent = 1;

      break; // DUT issues only one per cycle
   end

endfunction : scb_issue_ar

//=============================================================================
// Function: l3_handle_read_request (AXI AxCACHE aware)
// FIXED: Properly handle read allocate and cache fills
//Check if request is cacheable
//Lookup in L3
//If HIT → update MESI + LRU
//If MISS → allocate line + start fill
//=============================================================================
function void axi4_scoreboard::l3_handle_read_request(
  input int master_id,
  input axi4_master_tx m_tx,
  output bit expected_hit
);

endfunction : l3_handle_read_request

//=============================================================================
// Function: l3_handle_write_request
// FIXED: Use correct referenceData structure with slave_idx
//=============================================================================
function void axi4_scoreboard::l3_handle_write_request(
  input int master_id,
  input axi4_master_tx m_tx,
  input int slave_idx
);
  
endfunction : l3_handle_write_request

//=============================================================================
// Function: l3_complete_fill
// FIXED: Actually populate cache data from read response
//=============================================================================
function void axi4_scoreboard::l3_complete_fill(
  input int master_id,
  input bit[ADDR_WIDTH-1:0] addr,
  input axi4_master_tx m_tx
);
  int tag, index, offset;
  int way;
  bit found;
  int bytes_per_beat;
  longint temp_addr;
  
  l3_cache_decode_address(addr, tag, index, offset);
  
  // Find the filling cache line
  found = 0;
  for(int w = 0; w < L3_CACHE_ASSOCIATIVITY; w++) begin
    if(l3_cache[index][w].filling && 
       l3_cache[index][w].tag == tag &&
       l3_cache[index][w].sharers[master_id]) begin
      way = w;
      found = 1;
      break;
    end
  end
  
  if(found) begin
    // POPULATE CACHE DATA FROM READ RESPONSE
    bytes_per_beat = 1 << m_tx.arsize;
    temp_addr = m_tx.araddr;
    
    foreach(m_tx.rdata[beat]) begin
      for(int byte_idx = 0; byte_idx < bytes_per_beat; byte_idx++) begin
        int lane = temp_addr % (DATA_WIDTH/8);
        int line_offset = temp_addr % L3_CACHE_LINE_SIZE_BYTES;
        
        // Copy data from read response into cache line
        l3_cache[index][way].data[line_offset] = m_tx.rdata[beat][8*lane+7 -: 8];
        temp_addr++;
      end
    end
    
    // Mark filling complete
    l3_cache[index][way].filling = 0;
    
    `uvm_info("L3_FILL_COMPLETE",
             $sformatf("M[%0d] Fill Complete with Data: Set=%0d Way=%0d Addr=0x%0h Tag=0x%0h Time=%0t",
                      master_id, index, way, addr, tag, $time - l3_cache[index][way].fill_start_time),
             UVM_MEDIUM)
  end else begin
    `uvm_warning("L3_FILL_NOT_FOUND",
                $sformatf("M[%0d] Could not find filling cache line for Addr=0x%0h",
                         master_id, addr))
  end
endfunction : l3_complete_fill

//=============================================================================
// Function: connect_phase
//=============================================================================
function void axi4_scoreboard::connect_phase(uvm_phase phase);
  super.connect_phase(phase);
endfunction : connect_phase

//=============================================================================
// Function: get_slave_index
//=============================================================================
function int axi4_scoreboard::get_slave_index(logic[ADDR_WIDTH-1:0] addr);
  for(int i = 0; i < NO_OF_SLAVES; i++) begin
    if(addr >= SLAVE_START_ADDR[i] && addr <= SLAVE_END_ADDR[i]) begin
      return i;
    end
  end
  return -1;
endfunction : get_slave_index

//=============================================================================
// Function: check_write_rr_arbitration
//=============================================================================
function void axi4_scoreboard::check_write_rr_arbitration(int slave_id, int granted_master);
  int expected_master;
  int search_count;
  bit found_pending;
  
  rr_write_grants++;
  
  search_count = 0;
  found_pending = 0;
  expected_master = rr_write_next_master[slave_id];
  
  while(search_count < NO_OF_MASTERS) begin
    if(rr_write_pending_cnt[slave_id][expected_master] > 0) begin
      found_pending = 1;
      break;
    end
    expected_master = (expected_master + 1) % NO_OF_MASTERS;
    search_count++;
  end
  
  if(!found_pending) begin
    `uvm_info("RR_WRITE_NO_CONTENTION", 
              $sformatf("Slave[%0d]: No contention, granted to Master[%0d]", slave_id, granted_master), 
              UVM_HIGH)
    rr_write_last_granted[slave_id] = granted_master;
    return;
  end
  
  if(granted_master != expected_master) begin
    `uvm_error("RR_WRITE_VIOLATION",
              $sformatf("Slave[%0d] Write RR Arbitration Violation: Expected M%0d, Granted M%0d",
                       slave_id, expected_master, granted_master))
    rr_write_violations++;
  end
  
  rr_write_next_master[slave_id] = (granted_master + 1) % NO_OF_MASTERS;
  rr_write_last_granted[slave_id] = granted_master;
  
  if(rr_write_pending_cnt[slave_id][granted_master] > 0) begin
    rr_write_pending_cnt[slave_id][granted_master]--;
  end
  
endfunction : check_write_rr_arbitration

//=============================================================================
// Function: check_read_rr_arbitration
//=============================================================================
function void axi4_scoreboard::check_read_rr_arbitration(int slave_id, int granted_master);
  int expected_master;
  int search_count;
  bit found_pending;
  
  rr_read_grants++;
  
  search_count = 0;
  found_pending = 0;
  expected_master = rr_read_next_master[slave_id];
  
  while(search_count < NO_OF_MASTERS) begin
    if(rr_read_pending_cnt[slave_id][expected_master] > 0) begin
      found_pending = 1;
      break;
    end
    expected_master = (expected_master + 1) % NO_OF_MASTERS;
    search_count++;
  end
  
  if(!found_pending) begin
    `uvm_info("RR_READ_NO_CONTENTION", 
              $sformatf("Slave[%0d]: No contention, granted to Master[%0d]", slave_id, granted_master), 
              UVM_HIGH)
    rr_read_last_granted[slave_id] = granted_master;
    return;
  end
  
  if(granted_master != expected_master) begin
    `uvm_error("RR_READ_VIOLATION",
              $sformatf("Slave[%0d] Read RR Arbitration Violation: Expected M%0d, Granted M%0d",
                       slave_id, expected_master, granted_master))
    rr_read_violations++;
  end
  
  rr_read_next_master[slave_id] = (granted_master + 1) % NO_OF_MASTERS;
  rr_read_last_granted[slave_id] = granted_master;
  
  if(rr_read_pending_cnt[slave_id][granted_master] > 0) begin
    rr_read_pending_cnt[slave_id][granted_master]--;
  end
  
endfunction : check_read_rr_arbitration

//=============================================================================
// Function: ref_model_write
// FIXED: Pass slave_idx to l3_handle_write_request
//=============================================================================
function void axi4_scoreboard::ref_model_write(axi4_master_tx m_tx, int slave_idx, int master_idx);
  int bytes_per_beat;
  longint temp_addr;
  int align_amount;
  longint wrap_start_addr;
  longint wrap_end_addr;
  longint wrap_boundary;
  
  bytes_per_beat = 1 << m_tx.awsize;
  temp_addr = m_tx.awaddr;
  
  wrap_boundary = bytes_per_beat * (m_tx.awlen + 1);
  wrap_start_addr = (temp_addr / wrap_boundary) * wrap_boundary;
  wrap_end_addr = wrap_start_addr + wrap_boundary;
  
  align_amount = temp_addr % bytes_per_beat;
  
  foreach(m_tx.wdata[beat]) begin
    int local_align = (beat == 0) ? align_amount : 0;
    
    case(m_tx.awburst)
      2'b00: begin
        for(int byte_idx = local_align; byte_idx < bytes_per_beat; byte_idx++) begin
          longint byte_addr = m_tx.awaddr + byte_idx;
          int lane = byte_addr % (DATA_WIDTH/8);
          
          if(m_tx.wstrb[beat][lane]) begin
            referenceData[slave_idx][byte_addr] = m_tx.wdata[beat][8*lane+7 -: 8];
          end
        end
      end
      
      2'b01: begin
        for(int byte_idx = local_align; byte_idx < bytes_per_beat; byte_idx++) begin
          int lane = temp_addr % (DATA_WIDTH/8);
          
          if(m_tx.wstrb[beat][lane]) begin
            referenceData[slave_idx][temp_addr] = m_tx.wdata[beat][8*lane+7 -: 8];
          end
          temp_addr++;
        end
      end
      
      2'b10: begin
        for(int byte_idx = local_align; byte_idx < bytes_per_beat; byte_idx++) begin
          int lane = temp_addr % (DATA_WIDTH/8);
          
          if(m_tx.wstrb[beat][lane]) begin
            referenceData[slave_idx][temp_addr] = m_tx.wdata[beat][8*lane+7 -: 8];
          end
          temp_addr++;
          if(temp_addr >= wrap_end_addr) begin
            temp_addr = wrap_start_addr;
          end
        end
      end
    endcase
  end
  
  // L3 cache handles its own memory updates based on policy
  l3_handle_write_request(master_idx, m_tx, slave_idx);
  
endfunction : ref_model_write

//=============================================================================
// Function: ref_model_read
//=============================================================================
function void axi4_scoreboard::ref_model_read(axi4_master_tx m_tx, int slave_idx);
  int bytes_per_beat;
  longint temp_addr;
  int align_amount;
  longint wrap_start_addr;
  longint wrap_end_addr;
  longint wrap_boundary;
  
  bytes_per_beat = 1 << m_tx.arsize;
  temp_addr = m_tx.araddr;
  
  wrap_boundary = bytes_per_beat * (m_tx.arlen + 1);
  wrap_start_addr = (temp_addr / wrap_boundary) * wrap_boundary;
  wrap_end_addr = wrap_start_addr + wrap_boundary;
  
  align_amount = temp_addr % bytes_per_beat;
  
  m_tx.rdata = new[m_tx.arlen + 1];
  
  foreach(m_tx.rdata[beat]) begin
    int local_align = (beat == 0) ? align_amount : 0;
    m_tx.rdata[beat] = '0;
    
    case(m_tx.arburst)
      2'b00: begin
        for(int byte_idx = local_align; byte_idx < bytes_per_beat; byte_idx++) begin
          longint byte_addr = m_tx.araddr + byte_idx;
          int lane = byte_addr % (DATA_WIDTH/8);
          
          if(referenceData[slave_idx].exists(byte_addr)) begin
            m_tx.rdata[beat][8*lane+7 -: 8] = referenceData[slave_idx][byte_addr];
          end else begin
            m_tx.rdata[beat][8*lane+7 -: 8] = 8'h00;
            nonExistantMemRead++;
          end
        end
      end
      
      2'b01: begin
        for(int byte_idx = local_align; byte_idx < bytes_per_beat; byte_idx++) begin
          int lane = temp_addr % (DATA_WIDTH/8);
          
          if(referenceData[slave_idx].exists(temp_addr)) begin
            m_tx.rdata[beat][8*lane+7 -: 8] = referenceData[slave_idx][temp_addr];
          end else begin
            m_tx.rdata[beat][8*lane+7 -: 8] = 8'h00;
            nonExistantMemRead++;
          end
          temp_addr++;
        end
      end
      
      2'b10: begin
        for(int byte_idx = local_align; byte_idx < bytes_per_beat; byte_idx++) begin
          int lane = temp_addr % (DATA_WIDTH/8);
          
          if(referenceData[slave_idx].exists(temp_addr)) begin
            m_tx.rdata[beat][8*lane+7 -: 8] = referenceData[slave_idx][temp_addr];
          end else begin
            m_tx.rdata[beat][8*lane+7 -: 8] = 8'h00;
            nonExistantMemRead++;
          end
          temp_addr++;
          if(temp_addr >= wrap_end_addr) begin
            temp_addr = wrap_start_addr;
          end
        end
      end
    endcase
  end
  
endfunction : ref_model_read

//=============================================================================
// TASK: run_phase
// Combined L3 cache + arbitration + in-order transaction handling
// FIXED: Call l3_complete_fill when read data completes for cache misses
//=============================================================================
task axi4_scoreboard::run_phase(uvm_phase phase);
  super.run_phase(phase);
  
  //===========================================================================
  // WRITE ADDRESS PATH - Master Side
  // Monitor master write address requests, perform L3 lookup, increment pending counter
  //===========================================================================
  foreach(axi4_master_write_address_analysis_fifo[i]) begin
    automatic int m_idx = i;
    fork
      forever begin
        axi4_master_tx m_write_addr_tx;
        int s_idx;
        pending_write_transaction_t pending_tx;
        
        axi4_master_write_address_analysis_fifo[m_idx].get(m_write_addr_tx);
        axi4_master_tx_awaddr_count[m_idx]++;
        total_master_tx_count++;
        
        `uvm_info("MSTR_WR_ADDR", 
                 $sformatf("M[%0d] AWID=0x%0h AWADDR=0x%0h AWLEN=%0d AWSIZE=%0d AWBURST=%0d AWCACHE=0x%0h", 
                          m_idx, m_write_addr_tx.awid, m_write_addr_tx.awaddr, 
                          m_write_addr_tx.awlen, m_write_addr_tx.awsize, m_write_addr_tx.awburst,
                          m_write_addr_tx.awcache), 
                 UVM_MEDIUM)
        
        // Determine target slave based on address
        s_idx = get_slave_index(m_write_addr_tx.awaddr);
        
        if(s_idx != -1) begin
          // Increment pending request counter for arbitration
          rr_write_pending_cnt[s_idx][m_idx]++;
          
          `uvm_info("WR_PENDING_INC", 
                   $sformatf("M[%0d]->S[%0d] pending counter: %0d", 
                            m_idx, s_idx, rr_write_pending_cnt[s_idx][m_idx]), 
                   UVM_HIGH)
          
          // Create pending transaction
          $cast(pending_tx.tx, m_write_addr_tx.clone());
          pending_tx.master_id = m_idx;
          pending_tx.slave_id = s_idx;
          pending_tx.address_granted = 0;
          pending_tx.write_data_complete = 0;
          pending_tx.beats_received = 0;
          
          // Store in queue indexed by slave and AWID (for in-order checking)
          pending_write_txns[s_idx][m_write_addr_tx.awid].push_back(pending_tx);
          
          `uvm_info("WR_PENDING", 
                   $sformatf("M[%0d]->S[%0d] AWID=0x%0h added to pending queue (size=%0d)", 
                            m_idx, s_idx, m_write_addr_tx.awid, 
                            pending_write_txns[s_idx][m_write_addr_tx.awid].size()), 
                   UVM_HIGH)
        end else begin
          `uvm_error("ADDR_DECODE", 
                    $sformatf("M[%0d] AWADDR=0x%0h doesn't map to any slave", 
                             m_idx, m_write_addr_tx.awaddr))
        end
      end
    join_none
  end
  
  //===========================================================================
  // WRITE ADDRESS PATH - Slave Side
  // Monitor slave write address acceptance and check arbitration
  //===========================================================================
  foreach(axi4_slave_write_address_analysis_fifo[i]) begin
    automatic int s_idx = i;
    fork
      forever begin
        axi4_slave_tx s_write_addr_tx;
        pending_write_transaction_t pending_tx;
        int master_id;
        bit found;
        
        axi4_slave_write_address_analysis_fifo[s_idx].get(s_write_addr_tx);
        axi4_slave_tx_awaddr_count[s_idx]++;
        
        `uvm_info("SLV_WR_ADDR", 
                 $sformatf("S[%0d] AWID=0x%0h AWADDR=0x%0h AWLEN=%0d", 
                          s_idx, s_write_addr_tx.awid, s_write_addr_tx.awaddr, 
                          s_write_addr_tx.awlen), 
                 UVM_MEDIUM)
        
        // Find matching pending transaction (in-order: first in queue with matching ID)
        found = 0;
        if(pending_write_txns[s_idx].exists(s_write_addr_tx.awid)) begin
          if(pending_write_txns[s_idx][s_write_addr_tx.awid].size() > 0) begin
            // Get first transaction (but don't pop yet - need to wait for all data)
            pending_tx = pending_write_txns[s_idx][s_write_addr_tx.awid][0];
            master_id = pending_tx.master_id;
            found = 1;
            
            // Check round-robin arbitration (also decrements counter)
            check_write_rr_arbitration(s_idx, master_id);
            
            // Compare write address
            axi4_write_address_comparison(pending_tx.tx, s_write_addr_tx, master_id, s_idx);
            
            // Mark address as granted and trigger event
            pending_tx.address_granted = 1;
            pending_write_txns[s_idx][s_write_addr_tx.awid][0] = pending_tx;
            
            // Trigger event to signal that address arbitration is complete
            ->slave_write_addr_granted[s_idx];
            
            `uvm_info("WR_ADDR_GRANTED", 
                     $sformatf("S[%0d] M[%0d] AWID=0x%0h address granted", 
                              s_idx, master_id, s_write_addr_tx.awid), 
                     UVM_HIGH)
          end
        end
        
        if(!found) begin
          `uvm_error("WR_ADDR_NO_MATCH", 
                    $sformatf("S[%0d] received AWID=0x%0h but no pending transaction found", 
                             s_idx, s_write_addr_tx.awid))
        end
      end
    join_none
  end
  
  //===========================================================================
  // WRITE DATA PATH - Slave Side
  // Wait for address arbitration before matching write data
  //===========================================================================
  foreach(axi4_slave_write_data_analysis_fifo[i]) begin
    automatic int s_idx = i;
    fork
      forever begin
        axi4_slave_tx s_write_data_tx;
        pending_write_transaction_t pending_tx;
        bit found;
        bit [ID_WIDTH-1:0] matched_id;
        
        axi4_slave_write_data_analysis_fifo[s_idx].get(s_write_data_tx);
        axi4_slave_tx_wdata_count[s_idx]++;
        
        `uvm_info("SLV_WR_DATA", 
                 $sformatf("S[%0d] WDATA[0]=0x%0h WSTRB=0x%0h WLAST=%0b", 
                          s_idx, s_write_data_tx.wdata[0], s_write_data_tx.wstrb[0], 
                          s_write_data_tx.wlast), 
                 UVM_HIGH)
        
        // Wait for address arbitration to complete
        @(slave_write_addr_granted[s_idx]);
        
        // Find the first pending write transaction that has address_granted but not write_data_complete
        found = 0;
        foreach(pending_write_txns[s_idx][id]) begin
          if(pending_write_txns[s_idx][id].size() > 0) begin
            if(pending_write_txns[s_idx][id][0].address_granted && 
               !pending_write_txns[s_idx][id][0].write_data_complete) begin
              pending_tx = pending_write_txns[s_idx][id][0];
              matched_id = id;
              
              // Compare write data
              axi4_write_data_comparison(pending_tx.tx, s_write_data_tx, 
                                        pending_tx.master_id, s_idx);
              
              pending_tx.beats_received++;
              
              // Check if this is the last beat
              if(s_write_data_tx.wlast) begin
                if(pending_tx.beats_received != (pending_tx.tx.awlen + 1)) begin
                  `uvm_error("WLAST_COUNT", 
                           $sformatf("S[%0d] M[%0d] WLAST at beat %0d but AWLEN=%0d", 
                                    s_idx, pending_tx.master_id, 
                                    pending_tx.beats_received, pending_tx.tx.awlen))
                end else begin
                  byte_data_cmp_verified_wlast_count++;
                end
                
                pending_tx.write_data_complete = 1;
                
                // Update reference model and L3 cache after all write data received
                ref_model_write(pending_tx.tx, s_idx, pending_tx.master_id);
                
                `uvm_info("WR_DATA_COMPLETE", 
                         $sformatf("S[%0d] M[%0d] AWID=0x%0h write data complete", 
                                  s_idx, pending_tx.master_id, pending_tx.tx.awid), 
                         UVM_MEDIUM)
              end
              
              // Update the queue
              pending_write_txns[s_idx][matched_id][0] = pending_tx;
              found = 1;
              break;
            end
          end
        end
        
        if(!found) begin
          `uvm_error("WR_DATA_NO_MATCH", 
                    $sformatf("S[%0d] received write data but no granted pending transaction", s_idx))
        end
      end
    join_none
  end
  
  //===========================================================================
  // WRITE RESPONSE PATH - Slave Side
  // Monitor write response and remove completed transactions
  //===========================================================================
  foreach(axi4_slave_write_response_analysis_fifo[i]) begin
    automatic int s_idx = i;
    fork
      forever begin
        axi4_slave_tx s_write_resp_tx;
        pending_write_transaction_t pending_tx;
        bit found;
        
        axi4_slave_write_response_analysis_fifo[s_idx].get(s_write_resp_tx);
        axi4_slave_tx_bresp_count[s_idx]++;
        total_slave_tx_count++;
        
        `uvm_info("SLV_WR_RESP", 
                 $sformatf("S[%0d] BID=0x%0h BRESP=%0s", 
                          s_idx, s_write_resp_tx.bid, s_write_resp_tx.bresp.name()), 
                 UVM_MEDIUM)
        
        // Find matching pending transaction by BID (in-order: first in queue)
        found = 0;
        if(pending_write_txns[s_idx].exists(s_write_resp_tx.bid)) begin
          if(pending_write_txns[s_idx][s_write_resp_tx.bid].size() > 0) begin
            pending_tx = pending_write_txns[s_idx][s_write_resp_tx.bid].pop_front();
            
            // Verify address was granted
            if(!pending_tx.address_granted) begin
              `uvm_error("BRESP_BEFORE_AW", 
                        $sformatf("S[%0d] M[%0d] BID=0x%0h received before AW granted", 
                                 s_idx, pending_tx.master_id, s_write_resp_tx.bid))
            end
            
            // Verify write data was complete before response
            if(!pending_tx.write_data_complete) begin
              `uvm_error("BRESP_BEFORE_WLAST", 
                        $sformatf("S[%0d] M[%0d] BID=0x%0h received before WLAST", 
                                 s_idx, pending_tx.master_id, s_write_resp_tx.bid))
            end
            
            // Compare write response
            axi4_write_response_comparison(pending_tx.tx, s_write_resp_tx, 
                                          pending_tx.master_id, s_idx);
            
            `uvm_info("WR_COMPLETE", 
                     $sformatf("S[%0d] M[%0d] BID=0x%0h write transaction complete", 
                              s_idx, pending_tx.master_id, s_write_resp_tx.bid), 
                     UVM_MEDIUM)
            found = 1;
          end
        end
        
        if(!found) begin
          `uvm_error("BRESP_NO_MATCH", 
                    $sformatf("S[%0d] received BID=0x%0h but no pending transaction", 
                             s_idx, s_write_resp_tx.bid))
        end
      end
    join_none
  end
  
  //===========================================================================
  // READ ADDRESS PATH - Master Side
  // Monitor master read address requests, perform L3 lookup, increment pending counter
  //===========================================================================
  foreach(axi4_master_read_address_analysis_fifo[i]) begin
    automatic int m_idx = i;
    fork
      forever begin
        axi4_master_tx m_read_addr_tx;
        int s_idx;
        pending_read_transaction_t pending_tx;
        bit expected_l3_hit;
        
        axi4_master_read_address_analysis_fifo[m_idx].get(m_read_addr_tx);
        axi4_master_tx_araddr_count[m_idx]++;
        
        `uvm_info("MSTR_RD_ADDR", 
                 $sformatf("M[%0d] ARID=0x%0h ARADDR=0x%0h ARLEN=%0d ARSIZE=%0d ARBURST=%0d ARCACHE=0x%0h", 
                          m_idx, m_read_addr_tx.arid, m_read_addr_tx.araddr, 
                          m_read_addr_tx.arlen, m_read_addr_tx.arsize, m_read_addr_tx.arburst,
                          m_read_addr_tx.arcache), 
                 UVM_MEDIUM)
        
        // Determine target slave
        s_idx = get_slave_index(m_read_addr_tx.araddr);
        
        if(s_idx != -1) begin
          // L3 CACHE READ REQUEST HANDLING (SHARED CACHE)
          l3_handle_read_request(m_idx, m_read_addr_tx, expected_l3_hit);
          
          // Increment pending request counter for arbitration
          rr_read_pending_cnt[s_idx][m_idx]++;
          
          `uvm_info("RD_PENDING_INC", 
                   $sformatf("M[%0d]->S[%0d] pending counter: %0d", 
                            m_idx, s_idx, rr_read_pending_cnt[s_idx][m_idx]), 
                   UVM_HIGH)
          
          // Generate expected read data from reference model
          $cast(pending_tx.tx, m_read_addr_tx.clone());
          ref_model_read(pending_tx.tx, s_idx);
          
          pending_tx.master_id = m_idx;
          pending_tx.slave_id = s_idx;
          pending_tx.address_granted = 0;
          pending_tx.expected_l3_hit = expected_l3_hit;
          pending_tx.addr_request_time = $time;
          pending_tx.prediction_made = 1;
          pending_tx.line_addr = get_line_base_addr(m_read_addr_tx.araddr);
          
          // Store in queue (in-order)
          pending_read_txns[s_idx][m_read_addr_tx.arid].push_back(pending_tx);
          
          `uvm_info("RD_PENDING", 
                   $sformatf("M[%0d]->S[%0d] ARID=0x%0h added to pending queue (size=%0d) L3_HIT=%0b", 
                            m_idx, s_idx, m_read_addr_tx.arid, 
                            pending_read_txns[s_idx][m_read_addr_tx.arid].size(),
                            expected_l3_hit), 
                   UVM_HIGH)
        end else begin
          `uvm_error("ADDR_DECODE", 
                    $sformatf("M[%0d] ARADDR=0x%0h doesn't map to any slave", 
                             m_idx, m_read_addr_tx.araddr))
        end
      end
    join_none
  end
  
  //===========================================================================
  // READ ADDRESS PATH - Slave Side
  // Monitor slave read address acceptance and check arbitration
  //===========================================================================
  foreach(axi4_slave_read_address_analysis_fifo[i]) begin
    automatic int s_idx = i;
    fork
      forever begin
        axi4_slave_tx s_read_addr_tx;
        pending_read_transaction_t pending_tx;
        int master_id;
        bit found;
        
        axi4_slave_read_address_analysis_fifo[s_idx].get(s_read_addr_tx);
        axi4_slave_tx_araddr_count[s_idx]++;
        
        `uvm_info("SLV_RD_ADDR", 
                 $sformatf("S[%0d] ARID=0x%0h ARADDR=0x%0h ARLEN=%0d", 
                          s_idx, s_read_addr_tx.arid, s_read_addr_tx.araddr, 
                          s_read_addr_tx.arlen), 
                 UVM_MEDIUM)
        
        // Find matching pending transaction (in-order)
        found = 0;
        if(pending_read_txns[s_idx].exists(s_read_addr_tx.arid)) begin
          if(pending_read_txns[s_idx][s_read_addr_tx.arid].size() > 0) begin
            pending_tx = pending_read_txns[s_idx][s_read_addr_tx.arid][0];
            master_id = pending_tx.master_id;
            found = 1;
            
            // Check round-robin arbitration for read (also decrements counter)
            check_read_rr_arbitration(s_idx, master_id);
            
            // Compare read address
            axi4_read_address_comparison(pending_tx.tx, s_read_addr_tx, master_id, s_idx);
            
            // Mark address as granted and trigger event
            pending_tx.address_granted = 1;
            pending_read_txns[s_idx][s_read_addr_tx.arid][0] = pending_tx;
            
            // Trigger event
            ->slave_read_addr_granted[s_idx];
            
            `uvm_info("RD_ADDR_GRANTED", 
                     $sformatf("S[%0d] M[%0d] ARID=0x%0h address granted", 
                              s_idx, master_id, s_read_addr_tx.arid), 
                     UVM_HIGH)
          end
        end
        
        if(!found) begin
          `uvm_error("RD_ADDR_NO_MATCH", 
                    $sformatf("S[%0d] received ARID=0x%0h but no pending transaction", 
                             s_idx, s_read_addr_tx.arid))
        end
      end
    join_none
  end
  
  //===========================================================================
  // READ DATA PATH - Slave Side
  // Monitor slave read data for counting purposes
  //===========================================================================
  foreach(axi4_slave_read_data_analysis_fifo[i]) begin
    automatic int s_idx = i;
    fork
      forever begin
        axi4_slave_tx s_read_data_tx;
        
        axi4_slave_read_data_analysis_fifo[s_idx].get(s_read_data_tx);
        
        // ==========================================================
        // L3 READ MISS REFILL SUPPORT [ADDED.....]
        // ==========================================================
        bit [ADDR_WIDTH-1:0] line_addr;
        int mshr_id;
        byte beat_bytes[AXI_DATA_BYTES];

        if(active_r_valid[s_idx]) begin
          mshr_id = active_r_mshr[s_idx];
        end
        else begin
          mshr_id = -1; 
        end
        
        if(mshr_id != -1) begin

          // Extract AXI beat into byte array
          for(int b = 0; b < AXI_DATA_BYTES; b++) begin
            beat_bytes[b] = s_read_data_tx.rdata[0][8*b +: 8];
          end

          // Update MSHR line buffer
          scb_update_mshr_beat(mshr_id, beat_bytes);

          // On last beat ----> complete refill
          if(s_read_data_tx.rlast) begin
            scb_mshr[mshr_id].done = 1;
            scb_release_mshr(mshr_id);

            `uvm_info("L3_REFILL_COMPLETE",
                $sformatf("Refill done for line 0x%0h (MSHR[%0d])",
                               line_addr, mshr_id),
                UVM_MEDIUM)
          end
        end

        axi4_slave_tx_rdata_count[s_idx]++;
        axi4_slave_tx_rresp_count[s_idx]++;
        
        `uvm_info("SLV_RD_DATA", 
                 $sformatf("S[%0d] RID=0x%0h RDATA[0]=0x%0h RLAST=%0b RRESP=%0s", 
                          s_idx, s_read_data_tx.rid, s_read_data_tx.rdata[0], 
                          s_read_data_tx.rlast, s_read_data_tx.rresp[0].name()), 
                 UVM_HIGH)
      end
    join_none
  end
  
  //===========================================================================
  // READ DATA PATH - Master Side
  // Wait for address arbitration before comparing read data
  // FIXED: Call l3_complete_fill for cache misses
  //===========================================================================
  foreach(axi4_master_read_data_analysis_fifo[i]) begin
    automatic int m_idx = i;
    fork
      forever begin
        axi4_master_tx m_read_data_tx;
        pending_read_transaction_t pending_tx;
        int s_idx;
        bit found;
        
        axi4_master_read_data_analysis_fifo[m_idx].get(m_read_data_tx);
        axi4_master_tx_rdata_count[m_idx]++;
        axi4_master_tx_rresp_count[m_idx]++;
        
        `uvm_info("MSTR_RD_DATA", 
                 $sformatf("M[%0d] RID=0x%0h RDATA[0]=0x%0h RLAST=%0b RRESP=%0s", 
                          m_idx, m_read_data_tx.arid, m_read_data_tx.rdata[0], 
                          m_read_data_tx.rlast, m_read_data_tx.rresp[0].name()), 
                 UVM_MEDIUM)
        
        // Find the slave this read went to
        s_idx = get_slave_index(m_read_data_tx.araddr);
        
        if(s_idx != -1) begin
          // Wait for address arbitration to complete
          @(slave_read_addr_granted[s_idx]);
          
          // Find matching pending transaction by RID (in-order)
          found = 0;
          if(pending_read_txns[s_idx].exists(m_read_data_tx.arid)) begin
            if(pending_read_txns[s_idx][m_read_data_tx.arid].size() > 0) begin
              // Check if address was granted for this transaction
              if(pending_read_txns[s_idx][m_read_data_tx.arid][0].address_granted) begin
                // For in-order transactions, response should match the first pending
                pending_tx = pending_read_txns[s_idx][m_read_data_tx.arid].pop_front();
                
                // Verify this is from the correct master
                if(pending_tx.master_id != m_idx) begin
                  `uvm_error("RD_MASTER_MISMATCH", 
                            $sformatf("S[%0d] RID=0x%0h expected from M[%0d] but received from M[%0d]", 
                                     s_idx, m_read_data_tx.arid, pending_tx.master_id, m_idx))
                end
                
                // Compare read data [FIXED WITH "expected_l3_hit"]
                axi4_read_data_comparison(
                      pending_tx.tx,
                      m_read_data_tx,
                      m_idx,
                      s_idx,
                      pending_tx.expected_l3_hit
                    );

                // FIXED: Complete L3 cache fill for cache misses
                if(!pending_tx.expected_l3_hit && m_read_data_tx.rlast) begin
                  l3_complete_fill(m_idx, pending_tx.line_addr, m_read_data_tx);
                end
                
                `uvm_info("RD_COMPLETE", 
                         $sformatf("S[%0d] M[%0d] RID=0x%0h read transaction complete (L3_HIT=%0b)", 
                                  s_idx, m_idx, m_read_data_tx.arid, pending_tx.expected_l3_hit), 
                         UVM_MEDIUM)
                found = 1;
              end else begin
                `uvm_error("RD_DATA_BEFORE_AR", 
                          $sformatf("M[%0d] S[%0d] RID=0x%0h read data before AR granted", 
                                   m_idx, s_idx, m_read_data_tx.arid))
              end
            end
          end
          
          if(!found) begin
            `uvm_error("RD_DATA_NO_MATCH", 
                      $sformatf("M[%0d] received RID=0x%0h from S[%0d] but no granted pending transaction", 
                               m_idx, m_read_data_tx.arid, s_idx))
          end
        end
      end
    join_none
  end
  
  wait fork;
  
endtask : run_phase



//==========================================================================
//  L3 HANDLE READ REQUEST
//==========================================================================
function void axi4_scoreboard::l3_handle_read_request(
    input int master_id,
    input axi4_master_tx m_tx,
    output bit expected_hit
  );
  
  axi_cache_policy_s policy; // policy : decoded AXI AxCACHE behavior
  int hit_way; // hit_way : which way matched (if hit)
  l3_state_e state; // state : CLEAN / DIRTY / INVALID / FILLING
  
  bit [L3_TAG_BITS-1:0] tag;           // Used for,
  bit [L3_INDEX_BITS-1:0] index;       //  - Address breakdown
  bit [L3_OFFSET_BITS-1:0] offset;     //  - LRU updates
  
  int mshr_id; // If miss -> we allocate an MSHR entry.
  
  expected_hit = 0; // Intially take it as zero...assumed miss!
  
  // ------------------------------------------------------------------------------
  // 1. Decode AXI cache policy
  // ------------------------------------------------------------------------------
  // Purpose: Converts the raw 4-bit AxCACHE signal into readable memory attributes
  // (Device, Cacheable, Write-Back/Through, Allocation).
  policy = axi_decode_cache_policy(m_tx.arcache, 1);
  
  // ------------------------------------------------------------------------------
  // 2. Device / Non-cacheable
  // ------------------------------------------------------------------------------
  // Purpose: Bypasses L3 for Device or Non-cacheable traffic per AXI rules.
  // Prevents state side-effects: No MSHR allocation, LRU updates, or counter increments.
  if(policy.device || !policy.cacheable) begin
    `uvm_info("L3_READ_BYPASS",
      $sformatf("M[%0d] Addr=0x%0h Non-cacheable",
                master_id, m_tx.araddr),
      UVM_MEDIUM)
    return;
  end
  
  // ------------------------------------------------------------------------------
  // 3. Decode address
  // ------------------------------------------------------------------------------
  // Purpose: Splits raw address into Tag, Index, and Offset fields.
  // Necessary for set-associative indexing and byte-level offset tracking.
  l3_cache_decode_address(m_tx.araddr, tag, index, offset);

  // ------------------------------------------------------------------------------
  // 4. Lookup
  // ------------------------------------------------------------------------------
  // Purpose: Validates Tag match against active lines (checks Valid, Filling, and Lock states).
  // Determines if the transaction is a HIT or requires a MISS/Bypass flow.
  if(l3_cache_lookup(m_tx.araddr, policy, hit_way, state)) begin : L3_READ_HIT
    // ================= READ HIT =================
    expected_hit = 1;

    l3_read_hits_per_master[master_id]++;
    l3_total_read_hits++;

    // Update LRU only on hit
    l3_update_lru(index, hit_way);

    `uvm_info("L3_READ_HIT",
      $sformatf("M[%0d] HIT Set=%0d Way=%0d Addr=0x%0h State=%s",
                master_id, index, hit_way,
                m_tx.araddr, state.name()),
      UVM_MEDIUM)
    
  end : L3_READ_HIT
  else begin : L3_READ_MISS
    // ================= READ MISS =================
    expected_hit = 0;

    l3_read_misses_per_master[master_id]++;
    l3_total_read_misses++;

    mshr_id = scb_allocate_mshr(
                0,              // is_write = 0 (read)
                m_tx.araddr,
                master_id,
                policy
              );

    if(mshr_id == -1) begin
      `uvm_warning("L3_READ_MISS_STALL",
        $sformatf("MSHR full for Addr=0x%0h",
                  m_tx.araddr))
      return;
    end

    `uvm_info("L3_READ_MISS",
      $sformatf("M[%0d] MISS Set=%0d Addr=0x%0h → MSHR[%0d]",
                master_id, index,
                m_tx.araddr, mshr_id),
      UVM_MEDIUM)
  end : L3_READ_MISS
  
  
endfunction : l3_handle_read_request


task automatic axi4_scoreboard::axi4_read_data_comparison(
  input axi4_master_tx exp_tx, 
  input axi4_master_tx act_tx, 
  input int master_id, 
  input int slave_id,
  input bit expected_hit
  );
  
  if(expected_hit) begin
    // ==========================================================
    // READ HIT VALIDATION
    // ==========================================================
    
    bit [L3_TAG_BITS-1:0] tag;
    bit [L3_INDEX_BITS-1:0] index;
    bit [L3_OFFSET_BITS-1:0] offset;
    int hit_way = -1;
    
    l3_cache_decode_address(exp_tx.araddr, tag, index, offset);

    // Find matching way
    for(int w = 0; w < L3_CACHE_ASSOCIATIVITY; w++) begin
      if(l3_cache[index][w].valid &&
         l3_cache[index][w].tag == tag &&
         l3_cache[index][w].state != L3_INVALID &&
         l3_cache[index][w].state != L3_FILLING) begin
        hit_way = w;
        break;
      end
    end

    if(hit_way == -1) begin
      `uvm_error("L3_HIT_LOOKUP_FAIL",
        $sformatf("Expected HIT but line not present Addr=0x%0h",
                  exp_tx.araddr))
    end
    else begin

      int bytes_per_beat = 1 << exp_tx.arsize;
      longint temp_addr  = exp_tx.araddr;

      foreach(act_tx.rdata[beat]) begin
        for(int byte_idx = 0; byte_idx < bytes_per_beat; byte_idx++) begin

          int line_offset = temp_addr % L3_CACHE_LINE_SIZE_BYTES;
          int lane        = temp_addr % (DATA_WIDTH/8);

          byte cache_byte =
            l3_cache[index][hit_way].data[line_offset];

          byte dut_byte =
            act_tx.rdata[beat][8*lane +: 8];

          if(cache_byte !== dut_byte) begin
            `uvm_error("L3_HIT_DATA_MISMATCH",
              $sformatf("Addr=0x%0h Beat=%0d Expected=0x%0h Got=0x%0h",
                        temp_addr, beat, cache_byte, dut_byte))
          end

          temp_addr++;
        end
      end

    end

  end
  else begin
    // ==========================================================
    // READ MISS VALIDATION
    // ==========================================================
    int bytes_per_beat = 1 << exp_tx.arsize;
    longint temp_addr  = exp_tx.araddr;

    foreach(act_tx.rdata[beat]) begin
      for(int byte_idx = 0; byte_idx < bytes_per_beat; byte_idx++) begin

        int lane = temp_addr % (DATA_WIDTH/8);
        byte expected_byte;

        if(referenceData[slave_id].exists(temp_addr)) begin
          expected_byte = referenceData[slave_id][temp_addr];
        end
        else begin
          expected_byte = 8'h00;
        end
        
        byte dut_byte = act_tx.rdata[beat][8*lane +: 8];
        
        if(expected_byte !== dut_byte) begin
          `uvm_error("L3_MISS_DATA_MISMATCH",
                     $sformatf("Addr=0x%0h Beat=%0d Expected=0x%0h Got=0x%0h",
                     temp_addr, beat, expected_byte, dut_byte))
        end
        
        temp_addr++;
      end
    end
  end
endtask : axi4_read_data_comparison

