// Che-Hao Hsu
// 04/07/2022
// EE 469
// Lab #1, Task 3

// alu_testbench tests if alu can perform addition, subtraction, add, or operations correctly.
// Also, ALUFlags needs to return the sign of zero, negative, carry, and overflow.
// Using readmemh to read vector file(alu.tv) into testbench.
//////////////////////////////////////////////////////////////////////////////////////////////

module alu_testbench();

	logic [31:0] a, b;
   logic [1:0]  ALUControl;
   logic [31:0] Result;
   logic [3:0]  ALUFlags;
	
	logic clk;
	logic [103:0] testvectors [20:0];
	
	alu dut (a, b, ALUControl, Result, ALUFlags);
	
	parameter period = 100;
	
	initial clk = 1;
	
	always begin
		#(period/2); clk = ~clk;
	end
	
	initial begin
	
	$readmemh("alu.tv", testvectors);
	
	for(int i = 0; i < 16; i++) begin
		{ALUControl, a, b, Result, ALUFlags} = testvectors[i]; @(posedge clk);
		
	end
	
	end

endmodule 