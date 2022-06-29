// Che-Hao Hsu
// 04/07/2022
// EE 469
// Lab #1, Task 2

// reg_file is a 16*32, 2 read ports, 1 write port, asynchronous register file,
// operating as a sequential circuit with an 1-bit input clk.
// It takes 32-bit write_data and 4-bit write_addr as inputs,
// which denote the value we want to write and the location it should be written in.
// reg_file is controlled by a 1-bit input, wr_en, to decide whether the write_data
// need to be written during this clock cycle.
// reg_file also has two 4-bit iputs, read_addr1 and read_addr2, 
// showing which locations we need to read data from. 
// It finally returns two 32-bit read_data1 and read_data2 as outputs,
// representing the data we get from reg_file.
/////////////////////////////////////////////////////////////////////////////////////

module reg_file(input  logic        clk, wr_en, 
					 input  logic [31:0] write_data, 
					 input  logic [3:0]  write_addr, 
                input  logic [3:0]  read_addr1, read_addr2, 
                output logic [31:0] read_data1, read_data2);


// Create a 16*32 memory (16 addresses, 32 bit word for each)
logic [15:0][31:0] memory;


// The write_data is written in the memory at write_addr,
// only when clk is at posedge and wr_en equals one.
always_ff@ (negedge clk) begin

if (wr_en) 
	memory[write_addr] <= write_data;

end


// Since the reads are asychronous, read_data1 and read_data2 are directly driven
// by memory at addresses read_addr1 and read_addr2.
assign read_data1 = memory[read_addr1];
assign read_data2 = memory[read_addr2];

endmodule 