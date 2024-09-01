


module tb #(
  parameter INW=16,
  parameter M=4,
  parameter N=4,
  localparam LOGN = $clog2(N),
  localparam LOGMN = $clog2(M*N)
);
    logic clk, reset;
  logic [INW-1:0] INPUT_TDATA;
    logic INPUT_TVALID;
    logic INPUT_TLAST;
  logic [LOGN:0] INPUT_TUSER;
  logic INPUT_TREADY;
  logic OUTPUT_TVALID;
  logic OUTPUT_TREADY;
  logic [INW-1:0] OUTPUT_TDATA;
  int a,b,c,d,r1,r2,r3,r4;
  int arr1 [3:0][3:0] = '{'{1, 2, 3, 4}, '{5, 6, 7, 8}, '{9, 10, 11, 12}, '{13, 14, 15, 16}};
  
 task expected_results;
  begin
    r1 = (1*a) + (2*b) + (3*c) + (4*d);
    r2 = (5*a) + (6*b) + (7*c) + (8*d);
    r3 = (9*a) + (10*b) + (11*c) + (12*d);
    r4 = (13*a) + (14*b) + (15*c) + (16*d);

    @(posedge OUTPUT_TVALID)
    begin
      @(negedge clk);
      @(negedge clk);
      if (OUTPUT_TDATA == r1)
        $display("OUTPUT %d is equal to expected_result %d", OUTPUT_TDATA, r1);
      else
        $display("Test Failed");

      @(negedge clk);
      if (OUTPUT_TDATA == r2)
        $display("OUTPUT %d is equal to expected_result %d", OUTPUT_TDATA, r2);
      else
        $display("Test Failed");

      @(negedge clk);
      if (OUTPUT_TDATA == r3)
        $display("OUTPUT %d is equal to expected_result %d", OUTPUT_TDATA, r3);
      else
        $display("Test Failed");

      @(negedge clk);
      if (OUTPUT_TDATA == r4)
        $display("OUTPUT %d is equal to expected_result %d", OUTPUT_TDATA, r4);
      else
        $display("Test Failed");

      $display("TEST PASSED");
    end
  end
endtask


     input_mems DUT(.*);
    initial begin
      clk=1'b1;
    end
      always #5 clk=~clk;
      initial begin
        $dumpfile("dump.vcd"); $dumpvars;
        reset=1'b1;
        OUTPUT_TREADY=1'b1;
        @(negedge clk) {reset,INPUT_TUSER[0]}=2'b00;
        @(negedge clk) begin
          INPUT_TVALID=1'b1;
          INPUT_TDATA=16'h0001;
        end
        @(negedge clk)
        @(negedge clk) begin
          INPUT_TDATA=16'h0002;
        end
          @(negedge clk) 
        begin
          INPUT_TDATA=16'h0003;
        end
        @(negedge clk) 
        begin
          INPUT_TDATA=16'h0004;
        end
        
          @(negedge clk)
        begin
        INPUT_TDATA=16'h0005;
        end
          @(negedge clk)
        begin
          INPUT_TDATA=16'h0006;
        end
        @(negedge clk)
        begin
        INPUT_TDATA=16'h0007;
        end
        @(negedge clk)
        begin
          INPUT_TDATA=16'h0008;
        end
        @(negedge clk) 
        begin
        INPUT_TDATA=16'h0009;
        end
        @(negedge clk)
        begin
        INPUT_TDATA=16'h000A;
        end
        @(negedge clk)
        begin
        INPUT_TDATA=16'h000B;
        end
        @(negedge clk)
        begin
          INPUT_TDATA=16'h000C;
        end
        @(negedge clk)
        begin
          INPUT_TDATA=16'h000D;
        end
        @(negedge clk)
        begin
          INPUT_TDATA=16'h000E;
        end
          @(negedge clk)
        begin
          INPUT_TDATA=16'h000F;
        end
          @(negedge clk)
        begin
        INPUT_TDATA=16'h0010;
        end
       @(negedge clk)
        begin
          INPUT_TDATA=$urandom_range(10, 1);
          a=INPUT_TDATA;
          INPUT_TUSER=3'b000;
          INPUT_TLAST=1'b0;
        end
        @(negedge clk)
        begin
          INPUT_TDATA=$urandom_range(10, 1);
          b=INPUT_TDATA;
          INPUT_TUSER=3'b010;
          INPUT_TLAST=1'b0;
        end
       @(negedge clk)
        begin
          INPUT_TDATA=$urandom_range(10, 1);
          c=INPUT_TDATA;
          INPUT_TUSER=3'b100;
          INPUT_TLAST=1'b0;
        end  
        
        @(negedge clk)
        begin
          INPUT_TDATA=$urandom_range(10, 1);
          d=INPUT_TDATA;
          INPUT_TUSER=3'b111;
          INPUT_TLAST=1'b1;
        end  



        
        repeat(10)
          begin
        expected_results;
          
         @(negedge clk)
            begin
              INPUT_TUSER[0]=1'b1;

          INPUT_TDATA=$urandom_range(10, 1);
          a=INPUT_TDATA;
          INPUT_TUSER=3'b001;
          INPUT_TLAST=1'b0;
        end
        @(negedge clk)
        begin
          INPUT_TDATA=$urandom_range(10, 1);
          b=INPUT_TDATA;
          INPUT_TUSER=3'b011;
          INPUT_TLAST=1'b0;
        end
       @(negedge clk)
        begin
          INPUT_TDATA=$urandom_range(10, 1);
          c=INPUT_TDATA;
          INPUT_TUSER=3'b100;
          INPUT_TLAST=1'b0;
        end  
        
        @(negedge clk)
        begin
          INPUT_TDATA=$urandom_range(10, 1);
          d=INPUT_TDATA;
          INPUT_TUSER=3'b111;
          INPUT_TLAST=1'b1;
        end                    
          end
 #500  
        $finish;
      end
      endmodule
      

