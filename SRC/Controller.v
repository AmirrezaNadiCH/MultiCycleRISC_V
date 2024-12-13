module ALU_Controller(ALUOp, OpCode, f3, f7, ALUControl);
    input [1:0] ALUOp;
    input [2:0] f3;
    input [6:0] f7;
    input [6:0] OpCode;
    output [2:0] ALUControl;
    reg    [2:0] ALUControl_temp;
    assign ALUControl = ALUControl_temp;

    parameter [2:0]	ADD = 3'b000,
					SUB = 3'b001,
					AND = 3'b010,
					OR =  3'b011,
					SLT = 3'b100,
					XOR = 3'b101,
                    PASS= 3'b110;

    parameter [1:0] ADD_OP     = 2'b00,
                    SUB_OP     = 2'b01,
                    CHECK_F_OP = 2'b10,
                    PASS_OP  = 2'b11;

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

    always @(ALUOp, f3, f7) begin
        ALUControl_temp = ADD;
        case(ALUOp)
            ADD_OP: begin ALUControl_temp = ADD; end
            SUB_OP: begin ALUControl_temp = SUB; end
            PASS_OP:begin ALUControl_temp = PASS; end
            CHECK_F_OP: begin
                if ( (OpCode ==  ADD_OPC) & (f3 == ADD_F3) & (f7 == ADD_F7) ) begin ALUControl_temp = ADD ; end
				else if ( (OpCode ==  SUB_OPC) & (f3 == SUB_F3) & (f7 == SUB_F7) ) begin ALUControl_temp = SUB ; end
				else if ( (OpCode ==  AND_OPC) & (f3 == AND_F3) & (f7 == AND_F7) ) begin ALUControl_temp = AND ; end
				else if ( (OpCode ==  OR_OPC) & (f3 == OR_F3) & (f7 == OR_F7) ) begin ALUControl_temp = OR ; end
				else if ( (OpCode ==  SLT_OPC) & (f3 == SLT_F3) & (f7 == SLT_F7) ) begin ALUControl_temp = SLT ; end
				else if ( (OpCode ==  LW_OPC) & (f3 == LW_F3) ) begin ALUControl_temp = ADD ; end
				else if ( (OpCode ==  ADDI_OPC) & (f3 == ADDI_F3) ) begin ALUControl_temp = ADD ; end
				else if ( (OpCode ==  XORI_OPC) & (f3 == XORI_F3) ) begin ALUControl_temp = XOR ; end
				else if ( (OpCode ==  ORI_OPC) & (f3 == ORI_F3) ) begin ALUControl_temp = OR ; end
				else if ( (OpCode ==  SLTI_OPC) & (f3 == SLTI_F3) ) begin ALUControl_temp = SLT ; end
				else if ( (OpCode ==  JALR_OPC) & (f3 == JALR_F3) ) begin ALUControl_temp = ADD ; end
				else if ( (OpCode ==  SW_OPC) & (f3 == SW_F3) ) begin ALUControl_temp = ADD ; end
				else if ( (OpCode ==  JAL_OPC) ) begin ALUControl_temp = ADD ; end
				else if ( (OpCode ==  BEQ_OPC) & (f3 == BEQ_F3) ) begin ALUControl_temp = SUB ; end
				else if ( (OpCode ==  BNE_OPC) & (f3 == BNE_F3) ) begin ALUControl_temp = SUB ; end
				else if ( (OpCode ==  LUI_OPC) ) begin ALUControl_temp = PASS ; end
            end
        endcase
    end
endmodule


module PCController(PCUpdate, ZERO, BrEQ, BrNE, PCWrite);
	input PCUpdate;
	input ZERO;
	input BrEQ, BrNE;
	output PCWrite;
	reg PCWrite_temp;
	assign PCWrite = PCWrite_temp;

	always @(PCUpdate, ZERO, BrEQ, BrNE) begin
		PCWrite_temp = 1'b0;
		if(PCUpdate)	PCWrite_temp = 1'b1;
		else if(BrEQ)	PCWrite_temp = ZERO ? 1'b1 : 1'b0;
		else if(BrNE)	PCWrite_temp = ZERO ? 1'b0 : 1'b1;
	end
endmodule


module Controller(ZERO, OpCode, f3, f7, clk,rst, PCWrite, AdrSrc, MemWrite, IRWrite, ResultSrc, ALUControl,
				 ALUSrcA, ALUSrcB, ImmSrc, RegWrite);

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
	input [6:0] OpCode;
	input [2:0] f3;
	input [6:0] f7;
	input clk,rst;

	output PCWrite, AdrSrc, MemWrite, IRWrite;
	output [1:0] ResultSrc;
	output [2:0] ALUControl;
	output [1:0] ALUSrcA, ALUSrcB;
	output [2:0] ImmSrc;
	output RegWrite;

	reg AdrSrc_temp, MemWrite_temp, IRWrite_temp;
	reg [1:0] ResultSrc_temp;
	reg [1:0] ALUSrcA_temp, ALUSrcB_temp, ImmSrc_temp;
	reg RegWrite_temp;

	assign AdrSrc = AdrSrc_temp;
	assign MemWrite = MemWrite_temp;
	assign IRWrite = IRWrite_temp;
	assign ResultSrc = ResultSrc_temp;
	assign ALUSrcA = ALUSrcA_temp;
	assign ALUSrcB = ALUSrcB_temp;
	assign ImmSrc = ImmSrc_temp;
	assign RegWrite = RegWrite_temp;

	parameter [3:0]	FETCH		= 4'd0,
					DECODE		= 4'd1,
					MEM_ADR		= 4'd2,
					EXECUTE_R	= 4'd3,
					EXECUTE_I	= 4'd4,
					JAL			= 4'd5,
					LUI			= 4'd6,
					BRANCH		= 4'd7,
					JALR_NPC	= 4'd8,
					MEM_READ	= 4'd9,
					ALU_WB		= 4'd10,
					JALR_RA		= 4'd11,
					MEM_WRITE	= 4'd12,
					MEM_WB		= 4'd13;

	reg [1:0] ALUOp;
	ALU_Controller ALU_Controller__(ALUOp, OpCode, f3, f7, ALUControl);

	reg PCUpdate, BrEQ, BrNE;
	PCController PCController__(PCUpdate, ZERO, BrEQ, BrNE, PCWrite);



	reg [3:0] ps, ns;

	always @(posedge clk, posedge rst) begin
		if(rst)	ps <= FETCH;
		else	ps <= ns;
	end

	always @(ps, ZERO, OpCode, f3, f7) begin
		ns = FETCH;
		case(ps)
			FETCH: begin ns = DECODE; end
			DECODE: begin
					if (((OpCode==LW_OPC)&(f3==LW_F3)) | ((OpCode==SW_OPC)&(f3==SW_F3))) ns = MEM_ADR;
					else if(OpCode == ADD_OPC) ns = EXECUTE_R;
					else if(OpCode == ADDI_OPC) ns = EXECUTE_I;
					else if(OpCode == JAL_OPC) ns = JAL;
					else if(OpCode == LUI_OPC) ns = LUI;
					else if(((OpCode==BEQ_OPC)&(f3==BEQ_F3))|((OpCode==BNE_OPC)&(f3==BNE_F3))) ns = BRANCH;
					else if(OpCode == JALR_OPC & f3 == JALR_F3) ns = JALR_NPC;
			end
			MEM_ADR: begin
				if((OpCode==LW_OPC)&(f3==LW_F3)) ns = MEM_READ;
				else if((OpCode==SW_OPC)&(f3==SW_F3)) ns = MEM_WRITE;
			end
			EXECUTE_R: begin ns = ALU_WB; end
			EXECUTE_I: begin ns = ALU_WB; end
			JAL: begin ns = ALU_WB; end
			LUI: begin ns = ALU_WB; end
			BRANCH: begin ns = FETCH; end
			JALR_NPC: begin ns = JALR_RA; end
			MEM_READ: begin ns = MEM_WB; end
			ALU_WB: begin ns = FETCH; end
			JALR_RA: begin ns = ALU_WB; end
			MEM_WRITE: begin ns = FETCH; end
			MEM_WB: begin ns = FETCH; end
		endcase
	end

	always @(ps, ZERO, OpCode, f3, f7) begin
		AdrSrc_temp = 1'b0;
		IRWrite_temp = 1'b0;
		ALUSrcA_temp = 2'b00;
		ALUSrcB_temp = 2'b00;
		ResultSrc_temp = 2'b00;
		ALUOp = 2'b00;
		PCUpdate = 1'b0;
		BrEQ = 1'b0;
		BrNE = 1'b0;
		RegWrite_temp = 1'b0;
		MemWrite_temp = 1'b0;

		case(ps)
			FETCH: begin
				AdrSrc_temp = 1'b0;
				IRWrite_temp = 1'b1;
				ALUSrcA_temp = 2'b00;
				ALUSrcB_temp = 2'b10;
				ResultSrc_temp = 2'b10;
				ALUOp = 2'b00;
				PCUpdate = 1'b1;
			end
			DECODE: begin
				ALUSrcA_temp = 2'b01;
				ALUSrcB_temp = 2'b01;
				ALUOp = 2'b00;
			end
			MEM_ADR: begin
				ALUSrcA_temp = 2'b10;
				ALUSrcB_temp = 2'b01;
				ALUOp = 2'b00;
			end
			EXECUTE_R: begin
				ALUSrcA_temp = 2'b10;
				ALUSrcB_temp = 2'b00;
				ALUOp = 2'b10;
			end
			EXECUTE_I: begin
				ALUSrcA_temp = 2'b10;
				ALUSrcB_temp = 2'b01;
				ALUOp = 2'b10;
			end
			JAL: begin
				ALUSrcA_temp = 2'b01;
				ALUSrcB_temp = 2'b10;
				ALUOp = 2'b00;
				ResultSrc_temp = 2'b00;
				PCUpdate = 1'b1;
			end
			LUI: begin
				ALUSrcB_temp = 2'b01;
				ALUOp = 2'b11;
			end
			BRANCH: begin
				ALUSrcA_temp = 2'b10;
				ALUSrcB_temp = 2'b00;
				ALUOp = 2'b01;
				ResultSrc_temp = 2'b00;
				if(OpCode == BEQ_OPC && f3 == BEQ_F3) BrEQ = 1'b1;
				else if(OpCode == BNE_OPC && f3 == BNE_F3) BrNE = 1'b1;
			end
			JALR_NPC: begin
				ALUSrcA_temp = 2'b10;
				ALUSrcB_temp = 2'b01;
				ALUOp = 2'b00;
			end
			MEM_READ: begin
				ResultSrc_temp = 2'b00;
				AdrSrc_temp = 1'b1;
			end
			ALU_WB: begin
				ResultSrc_temp = 2'b00;
				RegWrite_temp = 1'b1;
			end
			JALR_RA: begin
				ResultSrc_temp = 2'b00;
				PCUpdate = 1'b1;
				ALUSrcA_temp = 2'b01;
				ALUSrcB_temp = 2'b10;
				ALUOp = 2'b00;
			end
			MEM_WRITE: begin
				ResultSrc_temp = 2'b00;
				AdrSrc_temp = 1'b1;
				MemWrite_temp = 1'b1;
			end
			MEM_WB: begin
				ResultSrc_temp = 2'b01;
				RegWrite_temp = 1'b1;
			end
		endcase
	end

	always @(ALUSrcB, OpCode) begin
		ImmSrc_temp = IT_IMM;
		if(ALUSrcB == 2'b01) begin
			case(OpCode)
			ADDI_OPC:	ImmSrc_temp = IT_IMM;
			JALR_OPC:	ImmSrc_temp = IT_IMM;
			SW_OPC:		ImmSrc_temp = ST_IMM;
			JAL_OPC:	ImmSrc_temp = JT_IMM;
			BEQ_OPC:	ImmSrc_temp = BT_IMM;
			LUI_OPC:	ImmSrc_temp = UT_IMM;
			endcase
		end
	end


endmodule