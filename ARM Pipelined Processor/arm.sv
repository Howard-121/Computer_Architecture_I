// Che-Hao Hsu
// 05/06/2022
// EE 469
// Lab #3

/* arm is the spotlight of the show and contains the bulk of the datapath and control logic. This module is split into two parts, the datapath and control. 
*/

// clk - system clock
// rst - system reset
// InstrF - incoming 32 bit instruction from imem, contains opcode, condition, addresses and or immediates
// ReadDataM - data read out of the dmem
// WriteDataM - data to be written to the dmem
// MemWriteM - write enable to allowed WriteData to overwrite an existing dmem word
// PCF - the current program count value, goes to imem to fetch instruciton
// ALUOutM - result of the ALU operation, sent as address to the dmem

module arm (
    input  logic        clk, rst,
    input  logic [31:0] InstrF,
    input  logic [31:0] ReadDataM,
    output logic [31:0] WriteDataM, 
    output logic [31:0] PCF, ALUOutM,
    output logic        MemWriteM
);

    // datapath buses and signals
    logic [31:0] PCPrime, PCPrime2, PCPlus4F, PCPlus8D, PCPlus8E; // pc signals
    logic [ 3:0] RA1D, RA2D, RA1E, RA2E;                  // regfile input addresses
    logic [31:0] RD1, RD2, RD1E, RD2E;                  // raw regfile outputs
    logic [ 3:0] ALUFlags;                  // alu combinational flag outputs
    logic [31:0] ExtImmD, ExtImmE, SrcA, SrcB, SrcAE, SrcBE, SrcAEPrime, SrcBEPrime; // immediate and alu inputs 
    logic [31:0] ResultW;                    // computed or fetched value to be written into regfile or pc
	 
	 logic [31:0] InstrD;  // the instruction signal from pipeline register
	 logic [3:0] WA3W, WA3E, WA3M;  // write address for reg_file from each pipeline register
	 logic [31:0] WriteDataD, WriteDataE; // writedata from data memory from pipeline registers
	 logic [31:0] ALUResultE;  // the early bta signal for branch
	 logic [31:0] ReadDataW;  // the readdata signal from the last pipeline register
	 logic [31:0] ALUOutW; // the result of alu from the last pipeline register
	 

    // control signals
	 logic ENF, ENPC; // enable signals to control registers for stalling
	 logic CLRF, CLRD; // control signals for flushing data
	 
	 // all control signals from control unit and each pipeline register
    logic PCSrcD, MemtoRegD, ALUSrcD, RegWriteD, MemWriteD, BranchD;
	 logic PCSrcE, MemtoRegE, ALUSrcE, RegWriteE, MemWriteE, BranchE;
	 logic PCSrcEPrime, RegWriteEPrime, MemWriteEPrime;
	 logic PCSrcM, RegWriteM, MemtoRegM;
	 logic PCSrcW, RegWriteW, MemtoRegW;
    logic [1:0] RegSrcD, ImmSrcD, ALUControlE, ALUControlD;
	 
	 logic FlagWriteD, FlagWriteE; // determine if need to hold flags
	 logic [3:0] Flags, FlagsE;  // store the result of aluflags
	 logic CondExE;   // determine if branch happens or not
	 logic BranchTakenE;  // tell the mux to choose either bta or next instruction
	 logic [3:0] CondE;  // condition bits (cmd)
	 
	 logic [1:0] ForwardAE, ForwardBE;  // data forwarding control signals
	 logic StallF, StallD, FlushD, FlushE; // stalling and flushing control signals
	 logic ldrStallD;  // load stalling detection signal


    /* The datapath consists of a PC as well as a series of muxes to make decisions about which data words to pass forward and operate on. It is 
    ** noticeably missing the register file and alu, which you will fill in using the modules made in lab 1. To correctly match up signals to the 
    ** ports of the register file and alu take some time to study and understand the logic and flow of the datapath.
    */
    //-------------------------------------------------------------------------------
    //                                      DATAPATH
    //-------------------------------------------------------------------------------


    // Fetch Stage
	 
	 assign PCPrime2 = PCSrcW ? ResultW : PCPlus4F; // mux, use either default or newly computed value
	 assign PCPrime = BranchTakenE ? ALUResultE : PCPrime2;  // second mux, use either PCPrime2 or early BTA
    assign PCPlus4F = PCF + 32'd4;                  // default value to access next instruction      

    assign ENPC = ~StallF;  // enable signal for the reg before imem
	 
	 // update the PC, at rst initialize to 0
	 // if enable signal (ENPC) equals zero, stalling
    always_ff @(posedge clk) begin
        if (rst) PCF <= 32'd0;
		  else if (ENPC == 0) PCF <= PCF;
        else     PCF <= PCPrime;
    end

    // stalling (ENF) and flushing (CLRF) signals for the fetch-decode reg
	 assign ENF = ~StallD;
	 assign CLRF = FlushD;
	 
	 // Fetch Register
	 always_ff @(posedge clk) begin
		  if (rst) begin  // reset
			  InstrD <= 32'd0;
			  PCPlus8D <= 32'd0;
		  end
        else if (ENF == 0) begin  // stalling
			  InstrD <= InstrD;
			  PCPlus8D <= PCPlus8D;
		  end
		  else if (CLRF) begin  // flushing
			  InstrD <= 32'd0;
			  PCPlus8D <= 32'd0;
		  end
        else      begin
			  InstrD <= InstrF;
			  PCPlus8D <= PCPlus4F + 32'd4; // value read when reading from reg[15]
		  end
    end
	 
	 
	 // Decode Stage
	 
	 // determine the register addresses based on control signals
    // RegSrcD[0] is set if doing a branch instruction
    // RegSrcD[1] is set when doing memory instructions
    assign RA1D = RegSrcD[0] ? 4'd15        : InstrD[19:16];
    assign RA2D = RegSrcD[1] ? InstrD[15:12] : InstrD[ 3: 0];

	// reg_file is a 16*32, 2 read ports, 1 write port, asynchronous register file,
	// operating as a sequential circuit with an 1-bit input clk.
	// It takes 32-bit write_data and 4-bit write_addr connected with Instr[15:12] as inputs,
	// which denote the value we want to write and the location it should be written in.
	// reg_file is controlled by a 1-bit input, wr_en, to decide whether the write_data
	// need to be written during this clock cycle.
	// reg_file also has two 4-bit iputs, read_addr1 and read_addr2, 
	// showing which locations we need to read data from. 
	// It finally returns two 32-bit read_data1 and read_data2 as outputs,
	// representing the data we get from reg_file.
    reg_file u_reg_file (
        .clk       (clk), 
        .wr_en     (RegWriteW),
        .write_data(ResultW),
        .write_addr(WA3W),
        .read_addr1(RA1D), 
        .read_addr2(RA2D),
        .read_data1(RD1), 
        .read_data2(RD2)
    );

    // two muxes, put together into an always_comb for clarity
    // determines which set of instruction bits are used for the immediate
    always_comb begin
        if      (ImmSrcD == 'b00) ExtImmD = {{24{InstrD[7]}},InstrD[7:0]};          // 8 bit immediate - reg operations
        else if (ImmSrcD == 'b01) ExtImmD = {20'b0, InstrD[11:0]};                 // 12 bit immediate - mem operations
        else                     ExtImmD = {{6{InstrD[23]}}, InstrD[23:0], 2'b00}; // 24 bit immediate - branch operation
    end
	 
	 // flushing signal (CLRD) for the decode-execute reg
	 assign CLRD = FlushE;
	 
	 // Decode Register
	 always_ff @(posedge clk) begin
	 if (CLRD == 1 || rst == 1) begin // reset or flushing happen
		 
		 // datapath signals
		 RD1E <= 32'd0;
		 RD2E <= 32'd0;
		 WA3E <= 4'd0;
		 ExtImmE <= 32'd0;
		 PCPlus8E <= 32'd0;
		 RA2E <= 4'd0;
		 RA1E <= 4'd0;
		 
		 // control signals
		 PCSrcE <= 0;
		 RegWriteE <= 0;
		 MemtoRegE <= 0;
		 MemWriteE <= 0;
		 ALUControlE <= 0;
		 BranchE <= 0;
		 ALUSrcE <= 0;
		 FlagWriteE <= 0;
		 CondE <= 4'D0;
		 FlagsE <= 0;
		 
	 end
	 else begin
	 
		 // datapath signals
		 RD1E <= RD1;
		 RD2E <= RD2;
		 WA3E <= InstrD[15:12];
		 ExtImmE <= ExtImmD;
		 PCPlus8E <= PCPlus8D;
		 RA2E <= RA2D;
		 RA1E <= RA1D;
		 
		 // control signals
		 PCSrcE <= PCSrcD;
		 RegWriteE <= RegWriteD;
		 MemtoRegE <= MemtoRegD;
		 MemWriteE <= MemWriteD;
		 ALUControlE <= ALUControlD;
		 BranchE <= BranchD;
		 ALUSrcE <= ALUSrcD;
		 FlagWriteE <= FlagWriteD;
		 CondE <= InstrD[31:28];
		 FlagsE <= Flags;
	 
	 end
	 end
	 
	 
	 // Execute Stage
	 
	 // WriteData and SrcA are direct outputs of the register file, wheras SrcB is chosen between reg file output and the immediate
    assign SrcBEPrime = (RA2E == 'd15) ? PCPlus8E : RD2E;           // substitute the 15th regfile register for PC 
    assign SrcAEPrime = (RA1E == 'd15) ? PCPlus8E : RD1E;           // substitute the 15th regfile register for PC 
	 
	 
	 // the control signal selects forwarding signal from aluresult, forwarding signal from resultw, or the original value for SrcBA
	 always_comb begin
	 if (ForwardAE == 2'b00) SrcAE = SrcAEPrime;
	 else if (ForwardAE == 2'b01) SrcAE = ResultW;
	 else SrcAE = ALUOutM;
	 end
	 
	 // the control signal selects forwarding signal from aluresult, forwarding signal from resultw, or the original value for SrcBE
	 always_comb begin
	 if (ForwardBE == 2'b00) WriteDataE = SrcBEPrime;
	 else if (ForwardBE == 2'b01) WriteDataE = ResultW;
	 else WriteDataE = ALUOutM;
	 end
	 
	 
	 // determine alu operand to be either from reg file or from immediate
	 assign SrcBE = ALUSrcE ? ExtImmE  : WriteDataE; 
	 
	
	// alu is a combinational circuit, which takes 32-bit a and b as inputs, 
	// and 2-bit ALUControl as a control signal to decide the operation.
	// alu returns 32-bit Result as an output and a 4-bit output, ALUFlags. 
	// This alu supports 4 operations. (addition, subtraction, and, or)
	// ALUFlags also signifies zero, negative, carry, or overflow based on the Result.
    alu u_alu (
        .a          (SrcAE), 
        .b          (SrcBE),
        .ALUControl (ALUControlE),
        .Result     (ALUResultE),
        .ALUFlags   (ALUFlags)
    );
	 
	 // control signals determining if branch happens
	 assign PCSrcEPrime = PCSrcE & CondExE;
	 assign RegWriteEPrime = RegWriteE & CondExE;
	 assign MemWriteEPrime = MemWriteE & CondExE;
	 assign BranchTakenE = BranchE & CondExE;
	 
	 
	 // Execute Register
	 always_ff @(posedge clk) begin
		 if (rst) begin  // reset
		 
			 // datapath signals
			 ALUOutM <= 32'd0;
			 WriteDataM <= 32'd0;
			 WA3M <= 4'd0;
			 
			 // control signals
			 PCSrcM <= 0;
			 RegWriteM <= 0;
			 MemtoRegM <= 0;
			 MemWriteM <= 0;
		 end
		 else begin
		 
			 // datapath signals
			 ALUOutM <= ALUResultE;
			 WriteDataM <= WriteDataE;
			 WA3M <= WA3E;
			 
			 // control signals
			 PCSrcM <= PCSrcEPrime;
			 RegWriteM <= RegWriteEPrime;
			 MemtoRegM <= MemtoRegE;
			 MemWriteM <= MemWriteEPrime;
		 end
	 end
	 
	 
	 // Memory Stage
	 
	 // Memory Register
	 always_ff @(posedge clk) begin
		 if (rst) begin
			 
			 // datapath signals
			 ALUOutW <= 32'd0;
			 WA3W <= 4'd0;
			 ReadDataW <= 32'd0;
			 
			 // control signals
			 PCSrcW <= 0;
			 RegWriteW <= 0;
			 MemtoRegW <= 0;
		 end
		 else begin
			 
			 // datapath signals
			 ALUOutW <= ALUOutM;
			 WA3W <= WA3M;
			 ReadDataW <= ReadDataM;
			 
			 // control signals
			 PCSrcW <= 0;  // because branchtakenE can take care of branch case(early BTA), PCSrcW won't be used, and can be set 0
			 RegWriteW <= RegWriteM;
			 MemtoRegW <= MemtoRegM;
		 end
	 end
	 
	 
	 // Write Back Stage

    // determine the result to run back to PC or the register file based on whether we used a memory instruction
    assign ResultW = MemtoRegW ? ReadDataW : ALUOutW;    // determine whether final writeback result is from dmemory or alu


    /* The control conists of a large decoder, which evaluates the top bits of the instruction and produces the control bits 
    ** which become the select bits and write enables of the system. The write enables (RegWrite, MemWrite and PCSrc) are 
    ** especially important because they are representative of your processors current state. 
    */
    //-------------------------------------------------------------------------------
    //                                      CONTROL
    //-------------------------------------------------------------------------------
    
	 
    // In Decode Stage
	 always_comb begin
        casez (InstrD[27:20])

            // ADD (Imm or Reg)
            8'b00?_0100_0 : begin   // note that we use wildcard "?" in bit 25. That bit decides whether we use immediate or reg, but regardless we add
                PCSrcD    = 0;
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = InstrD[25]; // may use immediate
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00; 
                ALUControlD = 'b00;
					 
					 FlagWriteD = 0;
					 BranchD = 0;
            end

            // SUB (Imm or Reg) / SUBS
            8'b00?_0010_? : begin   // note that we use wildcard "?" in bit 25. That bit decides whether we use immediate or reg, but regardless we sub
                PCSrcD    = 0; 
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = InstrD[25]; // may use immediate
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00; 
                ALUControlD = 'b01;
					 
					 FlagWriteD = InstrD[20]; // condition flag (CMP) if bit 20 == 1, execute CMP
					 BranchD = 0;
            end

            // AND
            8'b000_0000_0 : begin
                PCSrcD    = 0; 
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = 0; 
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00;    // doesn't matter
                ALUControlD = 'b10;  
					 
					 FlagWriteD = 0;
					 BranchD = 0;
            end

            // ORR
            8'b000_1100_0 : begin
                PCSrcD    = 0; 
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = 0; 
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00;    // doesn't matter
                ALUControlD = 'b11;
					 
					 FlagWriteD = 0;
					 BranchD = 0;
            end

            // LDR
            8'b010_1100_1 : begin
                PCSrcD    = 0; 
                MemtoRegD = 1; 
                MemWriteD = 0; 
                ALUSrcD   = 1;
                RegWriteD = 1;
                RegSrcD   = 'b10;    // msb doesn't matter
                ImmSrcD   = 'b01; 
                ALUControlD = 'b00;  // do an add
					 
					 FlagWriteD = 0;
					 BranchD = 0;
            end

            // STR
            8'b010_1100_0 : begin
                PCSrcD    = 0; 
                MemtoRegD = 0; // doesn't matter
                MemWriteD = 1; 
                ALUSrcD   = 1;
                RegWriteD = 0;
                RegSrcD   = 'b10;    // msb doesn't matter
                ImmSrcD   = 'b01; 
                ALUControlD = 'b00;  // do an add
					 
					 FlagWriteD = 0;
					 BranchD = 0;
            end

            // B
            8'b1010_???? : begin
				  PCSrcD    = 1; 
				  MemtoRegD = 0;
				  MemWriteD = 0; 
				  ALUSrcD   = 1;
				  RegWriteD = 0;
				  RegSrcD   = 'b01;
				  ImmSrcD   = 'b10; 
				  ALUControlD = 'b00;  // do an add
				  
				  FlagWriteD = 0;
				  BranchD = 1;	
            end

			default: begin
				  PCSrcD    = 0; 
				  MemtoRegD = 0; // doesn't matter
				  MemWriteD = 0; 
				  ALUSrcD   = 0;
				  RegWriteD = 0;
				  RegSrcD   = 'b00;
				  ImmSrcD   = 'b00; 
				  ALUControlD = 'b00;  // do an add
				  
				  FlagWriteD = 0;
				  BranchD = 0;
			end
        endcase
    end

	 // flag reg keeps the aluflags
	 assign Flags = FlagWriteE ? ALUFlags : 4'D0;
	 
	 // Cond Unit
	 // CondExE equals one unless it doesn't take branch
	 always_comb begin
		casez (CondE)
					
			// unconditional
			4'b1110 : begin
			CondExE = 1;
			
			end
			
			// equal
			4'b0000 : begin
			
			if ( FlagsE[2] == 1 ) begin
			CondExE = 1;

			end
			else begin
			CondExE = 0;
			
			end
			
			end
			
			// not equal
			4'b0001 : begin
			
			if ( FlagsE[2] == 0 ) begin
			CondExE = 1;
				  
			end
			else begin
			CondExE = 0;
				  
			end
			
			end
			
			// greater or equal
			4'b1010 : begin
			
			if ( FlagsE[2] == 1 || FlagsE[3] == 0 ) begin
			CondExE = 1;  
				  
			end
			else begin
			CondExE = 0;
				  
			end
			
			end
			
			// greater
			4'b1100 : begin
			
			if ( FlagsE[3] == 0 ) begin
			CondExE = 1;
				  
			end
			else begin
			CondExE = 0;
				  
			end
			
			end
			
			// less or equal
			4'b1101 : begin
			
			if ( FlagsE[2] == 1 || FlagsE[3] == 1 ) begin
			CondExE = 1;
				  
			end
			else begin
			CondExE = 0;
				  
			end
			
			end
			
			// less
			4'b1011 : begin
			
			if ( FlagsE[3] == 1 ) begin
			CondExE = 1;
				  
			end
			else begin
			CondExE = 0;
				  
			end
			
			end
			
			// no branch
			default : begin
			CondExE = 1;

			end
			
			endcase 
			end
			
	
	
	// Hazard Unit
	// Data Forwarding Logic
	always_comb begin
		// check if the address of the value that goes to alu and the address of the write address at mem stage are the same
		// and also check if this is certainly the value that is going to be stored in the reg_file
		if ( (RA1E == WA3M) && (RegWriteM == 1))   
			ForwardAE = 2'b10;
		// check if the address of the value that goes to alu and the address of the write address at write back stage are the same
		// and also check if this is certainly the value that is going to be stored in the reg_file
		else if ( (RA1E == WA3W) && (RegWriteW == 1))
			ForwardAE = 2'b01;
		else
			ForwardAE = 2'b00;
	end
	
	// Data Forwarding Logic
	always_comb begin
		// check if the address of the value that goes to alu and the address of the write address at mem stage are the same
		// and also check if this is certainly the value that is going to be stored in the reg_file
		if ( (RA2E == WA3M) && (RegWriteM == 1))
			ForwardBE = 2'b10;
		// check if the address of the value that goes to alu and the address of the write address at write back stage are the same
		// and also check if this is certainly the value that is going to be stored in the reg_file
		else if ( (RA2E == WA3W) && (RegWriteW == 1))
			ForwardBE = 2'b01;
		else
			ForwardBE = 2'b00;
	end
	
	
	// Stalling Logic
	always_comb begin
	if (rst) begin // reset all flushing and stalling signals to zero in the beginning
	ldrStallD = 0;
	
	StallF = 0;
	FlushD = 0;
	FlushE = 0;
	StallD = 0;
	
	end
	else begin
		// the data hazard caused by load only happens as the write address is consistent with read address
		// and the memtoreg is asserted
		ldrStallD = ( (RA1D == WA3E) || (RA2D == WA3E) ) && (MemtoRegE == 1); 
		
		StallF = ldrStallD + PCSrcD; // when load or branch occurs, stall fetch stage
		FlushD = PCSrcD + PCSrcW + BranchTakenE;  // when branch occurs, flush decode stage
		FlushE = ldrStallD + BranchTakenE; // when load or branch happens, flush the exe stage
		StallD = ldrStallD; // when load happens, stall the decode stage
	
	end
	end

endmodule