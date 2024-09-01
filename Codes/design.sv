
module memory_dual_port #(
    parameter WIDTH = 16,                            // Data width
    parameter SIZE = 4,                              // Memory size
    localparam LOGSIZE = $clog2(SIZE)                // Logarithm of memory size for address calculation
)(
    input [WIDTH-1:0] data_in,                       // Input data to write
    output logic [WIDTH-1:0] data_out,               // Output data to read
    input [LOGSIZE-1:0] wr_addr,                     // Write address
    input [LOGSIZE-1:0] rd_addr,                     // Read address
    input clk,                                       // Clock signal
    input wr_en,                                     // Write enable signal
    input full,                                       // Full flag
    input logic rd_en
);


    logic [SIZE-1:0][WIDTH-1:0] mem;                 // Memory array declaration

    always_ff @(posedge clk) begin
        data_out <= mem[rd_addr];                    // Synchronous read operation

        // Synchronous write operation with bypass
        if (wr_en & !full) begin
            mem[wr_addr] <= data_in;                 // Write data to memory
            // If read and write addresses match, update output with input data
          if ((rd_addr == wr_addr) & wr_en & rd_en) 
                data_out <= data_in;          
        end
    end
endmodule

module head_logic #(
    parameter WIDTH = 16, 
    parameter SIZE = 4,
    localparam LOGSIZE = $clog2(SIZE)          // Logarithm of memory size for address calculation
)(
    input logic clk,                           // Clock signal
    input logic reset,                         // Reset signal
    input logic wr_en,                         // Write enable signal
    input logic full,                          // Full flag
    output logic [LOGSIZE-1:0] wr_addr         // Write address
);

    always_ff @(posedge clk, posedge reset) begin
        if (reset) 
            wr_addr <= '0;                     // Reset write address to zero
        else if (wr_en & !full) 
            wr_addr <= wr_addr + 1;            // Increment write address on write enable
    end
endmodule

module tail_logic #(
    parameter WIDTH = 16, 
    parameter SIZE = 4,
    localparam LOGSIZE = $clog2(SIZE)         // Logarithm of memory size for address calculation
)(
    input logic clk,                          // Clock signal
    input logic reset,                        // Reset signal
    input logic empty,                        // Empty flag
    output logic [LOGSIZE-1:0] rd_addr,         // Read address
    input logic AXIS_TREADY,                   // Ready signal from AXIS interface
    input logic rd_en
);
    

    always_ff @(posedge clk, posedge reset) begin
        if (reset) 
            rd_addr <= 0;                     // Reset read address to zero
      else if (rd_en & !empty) 
            rd_addr <= rd_addr + 1;            // Increment read address on ready signal
    end
endmodule

module capacity_logic #(
    parameter WIDTH = 16, 
    parameter SIZE = 4,
    localparam LOGSIZE = $clog2(SIZE)               // Logarithm of memory size for address calculation
)(
    input logic clk,                             // Clock signal
    input logic reset,                           // Reset signal
    output logic empty,                          // Empty flag
    output logic full,                           // Full flag
    input logic wr_en,                           // Write enable signal
    output logic [$clog2(SIZE+1)-1:0] capacity,  // Current capacity of the memory
    input logic AXIS_TREADY,                      // Ready signal from AXIS interface
    input logic rd_en
);
  

    always_ff @(posedge clk, posedge reset) begin
        if (reset) 
            capacity <= SIZE;                  // Reset capacity to maximum size
        else if (wr_en & !full) 
            capacity <= capacity - 1;          // Decrease capacity on write
      else if (rd_en & !empty) 
            capacity <= capacity + 1;          // Increase capacity on read
    end

    // Combinational logic to determine empty and full status
    always_comb begin
        if (capacity == SIZE) begin
            empty = 1'b1;                      // Memory is empty
            full = 0;                          // Memory is not full
        end else if (capacity == 0) begin
            full = 1'b1;                       // Memory is full
            empty = 0;                         // Memory is not empty
        end else begin
            full = 0;                          // Memory is neither full nor empty
            empty = 0;
        end
    end
endmodule

module top #(
    parameter WIDTH = 16, 
    parameter SIZE = 4,
    localparam LOGSIZE = $clog2(SIZE)          // Logarithm of memory size for address calculation
)(
    input logic clk,                           // Clock signal
    input logic reset,                         // Reset signal
    input logic wr_en,                         // Write enable signal
    input logic AXIS_TREADY,                   // Ready signal from AXIS interface
    input logic [WIDTH-1:0] data_in,           // Input data for memory
    output logic [WIDTH-1:0] AXIS_TDATA,       // Output data for AXIS interface
    output logic [$clog2(SIZE+1)-1:0] capacity,  // Current capacity of the memory
    output logic full, empty,                      // Full and empty flags
    input logic rd_en

);

    logic [LOGSIZE-1:0] wr_addr, rd_addr;      // Write and read addresses
                       

    // Instantiate modules for dual-port memory, address logic, and capacity management
    memory_dual_port DUT1(
        .clk(clk),
        .data_in(data_in),
        .data_out(AXIS_TDATA),
        .rd_addr(rd_addr),
        .wr_addr(wr_addr),
        .wr_en(wr_en),
        .rd_en,
        .full(full)
    );

    head_logic DUT2(
        .clk(clk),
        .reset(reset),
        .wr_addr(wr_addr),
        .wr_en(wr_en),
        .full(full)
    );

    tail_logic DUT3(
        .clk(clk),
        .reset(reset),
        .rd_addr(rd_addr),
        .empty(empty),
        .AXIS_TREADY,
        .rd_en
    );

    capacity_logic DUT4(
        .clk(clk),
        .reset(reset),
        .wr_en(wr_en),
        .empty(empty),
        .full(full),
        .capacity(capacity),
        .AXIS_TREADY,
        .rd_en
    );
endmodule

  
/*////////////////////////////////////////////////////////////////////////////////////

                              MATRIX MEMORY MODULE
                         
////////////////////////////////////////////////////////////////////////////////////*/ 


module matrix_memory #(
  parameter INW = 16,                       // Width of the input and output data
  parameter M = 4,                          // Number of rows in the matrix
  parameter N = 4,                          // Number of columns in the matrix
  localparam SIZE = M * N,                  // Total number of elements in the matrix
  localparam ADDR_SIZE = $clog2(SIZE)       // Address width, calculated as log2 of SIZE
)
(
  input logic [INW-1:0] data_in,            // Data input for writing to the matrix
  input logic clk,                          // Clock signal
  input logic wr_en_matrix,                 // Write enable signal for matrix
  input logic wr_en,                        // Write enable signal
  input logic [ADDR_SIZE-1:0] addr,         // Address input for accessing the matrix
  output logic [INW-1:0] data_out           // Data output from the matrix
);

  // Memory array to hold matrix data
  logic [INW-1:0] mem_matrix [SIZE-1:0];

  // Always block for synchronous operations on the rising edge of the clock
  always_ff @(posedge clk) begin
    // Output data from the memory at the given address
    data_out <= mem_matrix[addr];
    
    // Write data to the memory if both write enable signals are asserted
    if (wr_en_matrix & wr_en) begin
      mem_matrix[addr] <= data_in;
    end
  end

endmodule



/*////////////////////////////////////////////////////////////////////////////////////

                                VECTOR MEMORY MODULE
                         
////////////////////////////////////////////////////////////////////////////////////*/ 



module vector_memory #(
  parameter N = 4,                       // Number of elements in the vector
  parameter INW = 16,                    // Width of each data element
  localparam WIDTH = INW + $clog2(N),    // Total width of the data input/output, combining data width and address width
  localparam ADDR_SIZE_V = $clog2(N)     // Number of bits needed for addressing each element in the vector
)
(
  input logic [WIDTH-1:0] data_in,       // Data input for writing to the vector
  input logic clk,                       // Clock signal for synchronous operations
  input logic wr_en_VECTOR,              // Write enable signal for vector memory
  input logic wr_en,                     // General write enable signal
  input logic [ADDR_SIZE_V-1:0] addr,    // Address input for accessing the vector
  output logic [WIDTH-1:0] data_out      // Data output from the vector memory
);

  // Memory array to hold vector data
  logic [N-1:0][WIDTH-1:0] mem_vector;

  // Always block for synchronous operations on the rising edge of the clock
  always_ff @(posedge clk) begin
    // Output data from the vector at the specified address
    data_out <= mem_vector[addr];
    
    // Write data to the vector if both write enable signals are asserted
    if (wr_en_VECTOR & wr_en) begin
      mem_vector[addr] <= data_in;
    end
  end

endmodule




/*////////////////////////////////////////////////////////////////////////////////////

                         INPUT MEMORIES MODULE
                         
////////////////////////////////////////////////////////////////////////////////////*/ 

module input_mems #(
  parameter INW=16,
  parameter M=4,
  parameter N=4,
  localparam SIZE=M*N,
  localparam LOGN = $clog2(N),
  localparam LOGMN = $clog2(M*N)
)
  (
    input logic clk, reset,
    input logic [INW-1:0] INPUT_TDATA,
    input logic INPUT_TVALID,
    input logic INPUT_TLAST,
    input logic [LOGN:0] INPUT_TUSER,
    output logic INPUT_TREADY,
    input logic OUTPUT_TREADY,
    output logic OUTPUT_TVALID,
    output logic [INW-1:0] OUTPUT_TDATA    
);
  logic done;
  logic [INW-1:0] matrix_data;
  logic [LOGMN-1:0] matrix_read_addr;
  logic [LOGN-1:0] vector_row;
  logic [INW-1:0] vector_val;
  logic [LOGN-1:0] vector_read_addr;
  logic input_loaded;
  logic [LOGN:0] D;
  logic wr_en_matrix,wr_en_VECTOR;
  logic incr;
  logic [INW-1:0] out;
  logic [$clog2(N+1)-1:0] capacity;
  logic delayed_input_loaded;
  logic [(INW+LOGN)-1:0] vect_data_in;
  assign vect_data_in={INPUT_TDATA,INPUT_TUSER[LOGN:1]};
  logic [(INW+LOGN)-1:0] vect_data_out;
  assign {vector_val,vector_row}=vect_data_out;
  logic wr_en;
  assign wr_en=INPUT_TVALID & INPUT_TREADY;
  assign INPUT_TREADY=~input_loaded;
  logic [LOGN-1:0] vector_read_addr_inp;
  logic [LOGMN-1:0] matrix_read_addr_inp;
  logic [LOGN-1:0] vector_read_addr_MAC;
  logic [LOGMN-1:0] matrix_read_addr_MAC;
  logic [INW-1:0] vect_data_out_ff;
  logic clear_acc;
  logic [LOGMN-1:0] matrix_base_addr;
  logic [LOGMN-1:0] matrix_base_addr_ff;
  assign matrix_read_addr_inp=input_loaded?matrix_read_addr_MAC:matrix_read_addr;
  assign vector_read_addr_inp=input_loaded?vector_read_addr_MAC:D;
  logic wr_en_FIFO;
  logic clear_acc_ff;
  logic delayed_input_3_loaded;
  logic delayed_input_2_loaded;
  logic [LOGN-1:0] count_for_MAC;
  logic full,empty;
  logic rd_en;
  assign rd_en=OUTPUT_TVALID & OUTPUT_TREADY;
  
  matrix_memory DUT1(.data_in(INPUT_TDATA),.clk,.wr_en_matrix,.wr_en,.addr(matrix_read_addr_inp),.data_out(matrix_data));       //matrix memory instantiation
  
  vector_memory DUT2(.data_in(vect_data_in),.data_out(vect_data_out),.clk,.wr_en_VECTOR,.wr_en,.addr(vector_read_addr_inp));
  
  MAC DUT3(.in0(matrix_data),.in1(vect_data_out_ff),.clk,.reset,.clear_acc(wr_en_FIFO),.delayed_input_2_loaded,.out,.input_loaded);
  
  
  top DUT4(.clk,.reset,.wr_en(wr_en_FIFO),.AXIS_TREADY(OUTPUT_TREADY),.data_in(out),.AXIS_TDATA(OUTPUT_TDATA),.capacity,.full,.empty,.rd_en);

 
  typedef enum logic [2:0] {START,INPUT_MATRIX,INPUT_VECTOR,INPUT_LOADED} state_t;                                    //states definition
  state_t present_state,next_state;
  
  assign OUTPUT_TVALID = ((present_state==START) & ~empty) ? 1:0;               // AXIS valid signal is active when memory is not empty
  
  
  always_comb
    begin
      if(input_loaded & full)
        done=1;
      else
        done=0;
    end

    
  always_ff@(posedge clk,posedge reset)
    begin
      if(reset)
        present_state<=START;                                                                                       // always_ff block for state 
      else
        present_state<=next_state;
    end
  
  
  
  always_ff@(posedge clk,posedge reset)                            // Matrix counter block
    begin
      if(reset | (matrix_read_addr==(M*N)-1))
        matrix_read_addr<='0;
      else if(wr_en_matrix & wr_en)                      
        matrix_read_addr<=matrix_read_addr+1;
    end
  
  always_ff@(posedge clk,posedge reset)                            // D counter block
    begin
      if(reset | done)
        D<='0;
      else if(wr_en_VECTOR & wr_en)                      
        D<=D+1;
    end

                     
                     
  always_comb
    begin
      unique case(present_state)
        START:
          begin
            if((wr_en & INPUT_TUSER[0] & empty)==1)
              next_state=INPUT_VECTOR;
            else if(wr_en &(!INPUT_TUSER[0]) & empty)
              next_state=INPUT_MATRIX;
            else
              next_state=START;
          end
        INPUT_MATRIX:
          begin
            if(matrix_read_addr==(M*N)-1)
              next_state=INPUT_VECTOR;
              else
                next_state=INPUT_MATRIX;
          end
        INPUT_VECTOR:
          begin
            if(INPUT_TLAST)
            next_state=INPUT_LOADED;
            else
              next_state=INPUT_VECTOR; 
          end                                                     // next state logic
        INPUT_LOADED:
          begin
            if(done)
              next_state=START;
              else
                next_state=INPUT_LOADED;
          end
        default:next_state=START;
      endcase
    end
  
                       
  always_comb
    begin
      unique case(present_state)
        START:
          begin
            input_loaded=0;
            wr_en_matrix=0;
            wr_en_VECTOR=0;
          end
        INPUT_MATRIX:
          begin
            input_loaded=0;
            wr_en_matrix=1;
            wr_en_VECTOR=0;
          end
        INPUT_VECTOR:
          begin
            input_loaded=0;
            wr_en_matrix=0;
            wr_en_VECTOR=1;
          end
        INPUT_LOADED:
           begin
             input_loaded=1;
            wr_en_matrix=0;
            wr_en_VECTOR=0;
           end
        default:
           begin
            input_loaded=0;
            wr_en_matrix=0;
            wr_en_VECTOR=0;
           end
      endcase
    end
  
  
  always_ff@(posedge clk or posedge reset)
    begin
      if(reset)
        vect_data_out_ff<='0;
      else
        vect_data_out_ff<=vector_val;
    end
  
  
  

  always_ff@(posedge clk or posedge reset)
    begin
      if(reset | (vector_read_addr_MAC==D-1) | done)                                      // always_block for generating vector_read_addr for MAC operation
        vector_read_addr_MAC<=0;
      else if(input_loaded)
        vector_read_addr_MAC<=vector_read_addr_MAC+1;
    end
  
    always_ff@(posedge clk or posedge reset)
    begin
      if(reset | (matrix_read_addr_MAC>(M*N)-2) | !input_loaded)
        begin
          matrix_base_addr<=0;
        end
      else if(incr & (matrix_read_addr_MAC < ((M*N)-1)) & input_loaded)
        begin
          matrix_base_addr<=matrix_base_addr+N;
        end
    end
  always_ff@(posedge clk or posedge reset)
    begin
      if(reset)
        matrix_base_addr_ff<=0;
      else
        matrix_base_addr_ff<=matrix_base_addr;
    end
  
  assign matrix_read_addr_MAC=matrix_base_addr_ff+vector_row;
  
  
  assign incr=(input_loaded & (vector_read_addr_inp==D-1))?1:0;
  


  logic [3:0] shift_reg;

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
            shift_reg <= 4'b0; 
        end else begin
          shift_reg <= {shift_reg[2:0], input_loaded}; 
        end
    end

  assign delayed_input_loaded = shift_reg[3];
  assign delayed_input_2_loaded = shift_reg[1];
  assign delayed_input_3_loaded = shift_reg[2];

  always_ff@(posedge clk,posedge reset)                            // Matrix counter block
    begin
      if(reset | (count_for_MAC==D-1) | (!input_loaded))
        count_for_MAC<=0;
         else if(delayed_input_3_loaded)                      
        count_for_MAC<=count_for_MAC+1;
    end


  
        
  always_ff@(posedge clk or posedge reset)
    begin
      if(reset)
        begin
          wr_en_FIFO<=0;
          clear_acc<=0;
        end
      else if(delayed_input_loaded & (count_for_MAC==D-1))
              begin
                wr_en_FIFO<=1;
                clear_acc<=1;
                clear_acc_ff<=clear_acc;
              end
              else
                begin
                  wr_en_FIFO<=0;
                  clear_acc<=0;
                  clear_acc_ff<=clear_acc;
                end
    end

      endmodule
      
      
        
module MAC #(
  parameter INW = 16,                 // Width of the input signals
  parameter M = 4,                    // Number of rows in the matrix
  parameter N = 4,                    // Number of columns in the matrix
  localparam LOGN = $clog2(N),        // Log base 2 of N, used for indexing
  localparam LOGMN = $clog2(M*N)      // Log base 2 of (M*N), used for indexing
)
(
  input logic [INW-1:0] in0,          // First input operand
  input logic [INW-1:0] in1,          // Second input operand
  input logic clk,                    // Clock signal
  input logic reset,                  // Active-high reset signal
  input logic clear_acc,              // Signal to clear the accumulator
  input logic delayed_input_2_loaded, // Signal indicating valid delayed input
  output logic [INW-1:0] out,          // Output of the MAC operation
  input logic input_loaded
);

  // Intermediate register to hold multiplication result
  logic [INW-1:0] intermediate;

  // Always block for synchronous operations
  always @(posedge clk or posedge reset) begin
    if (reset | !input_loaded) begin
      // On reset, initialize intermediate and out to zero
      intermediate <= 0;                              
      out <= 0;
    end
    else if (clear_acc) begin
      // When clear_acc is high, perform multiplication and set output to the intermediate
      intermediate <= in0 * in1;
      out <= intermediate;
    end
    else if (delayed_input_2_loaded) begin
      // When delayed_input_2_loaded is high accumulate the product
      intermediate <= in0 * in1;
      out <= out + intermediate;
    end
  end

endmodule
