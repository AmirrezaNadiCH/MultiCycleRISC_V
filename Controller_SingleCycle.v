module Controller(instruction, ZERO, pcsrc, ImmSrc, regwrite, ALUsrc, OpCode, memwrite, resultsrc);

	parameter [2:0]	ADD = 3'b000,
					SUB = 3'b001,
					AND = 3'b010,
					OR =  3'b011,
					SLT = 3'b100,
					XOR = 3'b101;

	parameter [6:0]	ADD_OPC =	7'd51,
					SUB_OPC =	7'd51,
					AND_OPC =	7'd51,
					OR_OPC =	7'd51,
					SLT_OPC =	7'd51,
					LW_OPC =	7'd3,
					ADDI_OPC =	7'd19,
					XORI_OPC =	7'd19,
					ORI_OPC =	7'd19,
					SLTI_OPC =	7'd19,
					JALR_OPC =	7'd103,
					SW_OPC =	7'd35,
					JAL_OPC =	7'd111,
					BEQ_OPC =	7'd99,
					BNE_OPC =	7'd99,
					LUI_OPC =	7'd55;

	parameter [2:0]	ADD_F3 =	3'd0,
					SUB_F3 =	3'd0,
					AND_F3 =	3'd7,
					OR_F3 =		3'd6,
					SLT_F3 =	3'd2,
					LW_F3 =		3'd2,
					ADDI_F3 =	3'd0,
					XORI_F3 =	3'd4,
					ORI_F3 =	3'd6,
					SLTI_F3 =	3'd2,
					JALR_F3 =	3'd0,
					SW_F3 =		3'd2,
					BEQ_F3 =	3'd0,
					BNE_F3 =	3'd1;

	parameter [6:0] ADD_F7 =	7'd0,
					SUB_F7 =	7'd32,
					AND_F7 =	7'd0,
					OR_F7 =		7'd0,
					SLT_F7 =	7'd0;

	parameter [2:0]	IT_IMM =	3'b000,
					ST_IMM =	3'b001,
					BT_IMM =	3'b010,
					JT_IMM =	3'b011,
					UT_IMM =	3'b100;



	input ZERO;
	input [31:0] instruction;
	output regwrite, ALUsrc, memwrite;
	output [1:0] pcsrc, resultsrc;
	output [2:0] OpCode, ImmSrc;


	reg regwrite_temp, ALUsrc_temp, memwrite_temp;
	reg [1:0] pcsrc_temp, resultsrc_temp;
	reg [2:0] OpCode_temp, ImmSrc_temp;

	wire [2:0] f3;
	wire [6:0] f7;
	wire [6:0] opc;

	assign regwrite = regwrite_temp;
	assign ALUsrc = ALUsrc_temp;
	assign memwrite = memwrite_temp;
	assign pcsrc = pcsrc_temp;
	assign ImmSrc = ImmSrc_temp;
	assign resultsrc = resultsrc_temp;
	assign OpCode = OpCode_temp;

	assign f3 = instruction[14:12];
	assign f7 = instruction[31:25];
	assign opc = instruction[6:0];

	always @(opc, f3, f7, ZERO) begin: ALU_OpCode
		OpCode_temp = ADD;
		if ((opc == ADD_OPC) & (f3 == ADD_F3) & (f7 == ADD_F7))				OpCode_temp = ADD;
		else if ((opc == SUB_OPC) & (f3 == SUB_F3) & (f7 == SUB_F7)) 		OpCode_temp = SUB;
		else if ((opc == AND_OPC) & (f3 == AND_F3) & (f7 == AND_F7)) 		OpCode_temp = AND;
		else if ((opc == OR_OPC) & (f3 == OR_F3) & (f7 == OR_F7)) 			OpCode_temp = OR;
		else if ((opc == SLT_OPC) & (f3 == SLT_F3) & (f7 == SLT_F7)) 		OpCode_temp = SLT;
		else if ((opc == LW_OPC) & (f3 == LW_F3))					 		OpCode_temp = ADD;
		else if ((opc == ADDI_OPC) & (f3 == ADDI_F3)) 						OpCode_temp = ADD;
		else if ((opc == XORI_OPC) & (f3 == XORI_F3)) 						OpCode_temp = XOR;
		else if ((opc == ORI_OPC) & (f3 == ORI_F3)) 						OpCode_temp = OR;
		else if ((opc == SLTI_OPC) & (f3 == SLTI_F3)) 						OpCode_temp = SLT;
		else if ((opc == JALR_OPC) & (f3 == JALR_F3)) 						OpCode_temp = ADD;
		else if ((opc == SW_OPC) & (f3 == SW_F3)) 							OpCode_temp = ADD;
		// else if ((opc == JAL_OPC)) 											OpCode_temp = SUB;
		else if ((opc == BEQ_OPC) & (f3 == BEQ_F3)) 						OpCode_temp = SUB;
		else if ((opc == BNE_OPC) & (f3 == BNE_F3)) 						OpCode_temp = SUB;
		// else if ((opc == LUI_OPC))					 						OpCode_temp = SUB;
	end

	always @(opc, f3, f7, ZERO) begin: PC_Source
		pcsrc_temp = 2'b10;
		if(opc == JALR_OPC)																	pcsrc_temp = 2'b00;
		else if(opc == JAL_OPC)																pcsrc_temp = 2'b01;
		else if(((opc==BEQ_OPC)&(f3==BEQ_F3)&ZERO)|((opc==BNE_OPC)&(f3==BNE_F3)&(~ZERO)))	pcsrc_temp = 2'b01;
		else 																				pcsrc_temp = 2'b10;
	end

	always @(opc, f3, f7, ZERO) begin: Immediate_Src
		ImmSrc_temp = 3'b000;
		case(opc)
			ADDI_OPC:	ImmSrc_temp = IT_IMM;
			JALR_OPC:	ImmSrc_temp = IT_IMM;
			SW_OPC:		ImmSrc_temp = ST_IMM;
			JAL_OPC:	ImmSrc_temp = JT_IMM;
			BEQ_OPC:	ImmSrc_temp = BT_IMM;
			LUI_OPC:	ImmSrc_temp = UT_IMM;
		endcase
	end

	always @(opc, f3, f7, ZERO) begin: Result_Source
		resultsrc_temp = 2'b00;
		regwrite_temp = 1'b0;
		if ((opc == ADD_OPC) & (f3 == ADD_F3) & (f7 == ADD_F7)) begin		resultsrc_temp = 2'b00; regwrite_temp = 1'b1; end
		if ((opc == SUB_OPC) & (f3 == SUB_F3) & (f7 == SUB_F7)) begin 	resultsrc_temp = 2'b00;	regwrite_temp = 1'b1; end
		if ((opc == AND_OPC) & (f3 == AND_F3) & (f7 == AND_F7)) begin 	resultsrc_temp = 2'b00;	regwrite_temp = 1'b1; end
		if ((opc == OR_OPC) & (f3 == OR_F3) & (f7 == OR_F7)) begin 	resultsrc_temp = 2'b00;	regwrite_temp = 1'b1; end
		if ((opc == SLT_OPC) & (f3 == SLT_F3) & (f7 == SLT_F7)) begin 	resultsrc_temp = 2'b00;	regwrite_temp = 1'b1; end
		if ((opc == LW_OPC) & (f3 == LW_F3)) begin					 	resultsrc_temp = 2'b01;	regwrite_temp = 1'b1; end
		if ((opc == ADDI_OPC) & (f3 == ADDI_F3)) begin 				resultsrc_temp = 2'b00;	regwrite_temp = 1'b1; end
		if ((opc == XORI_OPC) & (f3 == XORI_F3)) begin 				resultsrc_temp = 2'b00;	regwrite_temp = 1'b1; end
		if ((opc == ORI_OPC) & (f3 == ORI_F3)) begin 					resultsrc_temp = 2'b00;	regwrite_temp = 1'b1; end
		if ((opc == SLTI_OPC) & (f3 == SLTI_F3)) begin 				resultsrc_temp = 2'b00;	regwrite_temp = 1'b1; end
		if ((opc == JALR_OPC) & (f3 == JALR_F3)) begin 				resultsrc_temp = 2'b10;	regwrite_temp = 1'b1; end
		// if ((opc == SW_OPC) & (f3 == SW_F3)) begin 					resultsrc_temp = 2'b;	regwrite_temp = 1'b1; end
		if ((opc == JAL_OPC)) begin 									resultsrc_temp = 2'b10;	regwrite_temp = 1'b1; end
		// if ((opc == BEQ_OPC) & (f3 == BEQ_F3)) begin				resultsrc_temp = 2'b;	regwrite_temp = 1'b1; end
		// if ((opc == BNE_OPC) & (f3 == BNE_F3)) begin				resultsrc_temp = 2'b;	regwrite_temp = 1'b1; end
		if ((opc == LUI_OPC)) begin					 				resultsrc_temp = 2'b11;	regwrite_temp = 1'b1; end
	end

	always @(opc, f3, f7, ZERO) begin: ALU_Source
		ALUsrc_temp = 1'b0;
		if ((opc == ADD_OPC) & (f3 == ADD_F3) & (f7 == ADD_F7))				ALUsrc_temp = 1'b0;
		else if ((opc == SUB_OPC) & (f3 == SUB_F3) & (f7 == SUB_F7)) 		ALUsrc_temp = 1'b0;
		else if ((opc == AND_OPC) & (f3 == AND_F3) & (f7 == AND_F7)) 		ALUsrc_temp = 1'b0;
		else if ((opc == OR_OPC) & (f3 == OR_F3) & (f7 == OR_F7)) 			ALUsrc_temp = 1'b0;
		else if ((opc == SLT_OPC) & (f3 == SLT_F3) & (f7 == SLT_F7)) 		ALUsrc_temp = 1'b0;
		if ((opc == LW_OPC) & (f3 == LW_F3))					 			ALUsrc_temp = 1'b1;
		else if ((opc == ADDI_OPC) & (f3 == ADDI_F3)) 						ALUsrc_temp = 1'b1;
		else if ((opc == XORI_OPC) & (f3 == XORI_F3)) 						ALUsrc_temp = 1'b1;
		else if ((opc == ORI_OPC) & (f3 == ORI_F3)) 						ALUsrc_temp = 1'b1;
		else if ((opc == SLTI_OPC) & (f3 == SLTI_F3)) 						ALUsrc_temp = 1'b1;
		else if ((opc == JALR_OPC) & (f3 == JALR_F3)) 						ALUsrc_temp = 1'b1;
		else if ((opc == SW_OPC) & (f3 == SW_F3)) 							ALUsrc_temp = 1'b1;
		// else if ((opc == JAL_OPC)) 											ALUsrc_temp = 1'b0;
		else if ((opc == BEQ_OPC) & (f3 == BEQ_F3)) 						ALUsrc_temp = 1'b0;
		else if ((opc == BNE_OPC) & (f3 == BNE_F3)) 						ALUsrc_temp = 1'b0;
		// else if ((opc == LUI_OPC))					 						ALUsrc_temp = 1'b0;
	end

	assign memwrite_temp = ((opc == SW_OPC)&(f3 == SW_F3)) ? 1'b1 : 1'b0;


endmodule
