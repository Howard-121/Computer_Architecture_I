// Che-Hao Hsu
// 04/07/2022
// EE 469
// Lab #1, Task 2

// reg_file_testbench tests if the data are accessed from correct addresses
// at correct clock cycle, and if the data is written after wr_en is asserted.
///////////////////////////////////////////////////////////////////////////////

module reg_file_testbench();

	logic clk, wr_en;
	logic [31:0] write_data;
	logic [3:0]  write_addr;
	logic [3:0]  read_addr1, read_addr2;
	logic [31:0] read_data1, read_data2;
	
	reg_file dut (clk, wr_en, write_data, write_addr, 
	read_addr1, read_addr2, read_data1, read_data2);
	
	
	
	always 
	
	begin
	clk <= 0; #10; 
	clk <= 1; #10;
	
	end
	
	

	initial begin
	
	wr_en <= 0; write_data <= 10; write_addr <= 1; #20;
	wr_en <= 1; read_addr1 <= 0; read_addr2 <= 1; #20;
	wr_en <= 0; #20;
	wr_en <= 1; write_data <= 13; write_addr <= 3; #20;
	wr_en <= 0; #20;
	wr_en <= 1; write_data <= 4; write_addr <= 0; #20;
	wr_en <= 0; #20;
	read_addr1 <= 3; read_addr2 <= 0; #20;
	wr_en <= 1; write_data <= 100; write_addr <= 3; #20;
	wr_en <= 0; #20;
	
	end

endmodule 