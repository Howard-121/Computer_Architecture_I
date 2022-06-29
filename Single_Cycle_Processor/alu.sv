// Che-Hao Hsu
// 04/07/2022
// EE 469
// Lab #1, Task 3

// alu is a combinational circuit, which takes 32-bit a and b as inputs, 
// and 2-bit ALUControl as a control signal to decide the operation.
// alu returns 32-bit Result as an output and a 4-bit output, ALUFlags. 
// This alu supports 4 operations. (addition, subtraction, and, or)
// ALUFlags also signifies zero, negative, carry, or overflow based on the Result.
//////////////////////////////////////////////////////////////////////////////////

module alu(input  logic [31:0] a, b, 
           input  logic [1:0]  ALUControl, 
           output logic [31:0] Result, 
           output logic [3:0]  ALUFlags);

			  // 1-bit logic records carry_out
			  logic carry_out; 
			  
			  logic [31:0] temp; // temporary result of add and sub
			  logic [31:0] andtemp; // temporary result of and
			  logic [31:0] ortemp; // temporary result of or

			  // 32-bit logic holds the inverse of b for subtraction
			  logic [31:0] inverse_b;
			  assign inverse_b = ~b;

			  // combinational circuit calculates either addition 
			  // or subtraction based on ALUControl[0]
			  always_comb begin
			  
				if (ALUControl[0] == 1'b0) begin
				
					{carry_out, temp} = a + b + 1'b0;
				end
				else begin
				
					{carry_out, temp} = a + inverse_b + 1'b1;
				end
				
			  end
			  
			  // combinational circuit calculates and operation
			  always_comb begin
			  
				andtemp = a & b;
			  end

			  // combinational circuit calculates or operation
			  always_comb begin
			  
				ortemp = a | b;
			  end
			  
			  // Put temporary results, temp, andtemp, ortemp, into MUX. 
			  // Result finally equals one of them, selected by ALUControl
			  always_comb begin
			  
				case(ALUControl)
				
				2'd0: // add
				Result = temp;
				
				2'd1: // subtract
				Result = temp;
				
				2'd2: // and
				Result = andtemp;
				
				2'd3: // or
				Result = ortemp;
				
				endcase
				
			  end
			  
 			 // check if Result is zero
			 // the whole bits of Result should be zero
			 always_comb begin
			 
			 if (Result == 32'd0)
			 ALUFlags[2] = 1'b1;
			 else
			 ALUFlags[2] = 1'b0;
			 
			 end

			 // check if Result is negative
			 // MSB of Result should be one
			 always_comb begin
			 
			 if (Result[31] == 1'b1)
			 ALUFlags[3] = 1'b1;
			 else
			 ALUFlags[3] = 1'b0;
			 
			 end

			 // check if Result produces carry
			 // alu should perform addition or subtraction,
			 // and carry_out should equal one
			 always_comb begin
			 
			 if (ALUControl[1] == 0 && carry_out == 1'b1)
			 ALUFlags[1] = 1'b1;
			 else
			 ALUFlags[1] = 1'b0;
			 
			 end

			 // check if Result causes overflow
			 // alu should perform addition or subtraction,
			 // a and Result have opposite signs,
			 // and a, b have same signs upon addition,
			 // or a and b have different signs upon subtraction
			 always_comb begin
			 
			 if (ALUControl[1] == 0 && a[31] ^ Result[31] == 1 
			 && a[31] ^ b[31] ^ ALUControl[0] == 0)
			 ALUFlags[0] = 1'b1;
			 else
			 ALUFlags[0] = 1'b0;
			 
			 end
			 
endmodule 
