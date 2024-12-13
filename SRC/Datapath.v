module Register #(parameter SIZE = 32) (dataIN, clk,rst,ldEN, dataOUT);
	input [SIZE-1:0] dataIN;
	input clk,rst,ldEN;
	output [SIZE-1:0] dataOUT;

	reg [SIZE-1:0] rg;
	assign dataOUT = rg;

	always @(posedge clk, posedge rst) begin
		if(rst)			rg <= {(SIZE){1'b0}};
		else if(ldEN)	rg <= dataIN;
	end

endmodule

module MUX #(parameter SIZE = 2, parameter WL = 32) (dataIN, select, dataOUT);

	parameter SELECT_SIZE = $clog2(SIZE);

	input [SIZE-1:0][WL-1:0] dataIN;
	input [SELECT_SIZE-1:0] select;

	output [WL-1:0] dataOUT;

	assign dataOUT = dataIN[select];

endmodule

module Adder #(parameter WL = 32) (A, B, S);

	input [WL-1:0] A;
	input [WL-1:0] B;
	output [WL-1:0] S;

	assign S = A + B;
endmodule



module ALU #(parameter WL = 32) (OpCode, operand1, operand2, result, ZERO);

	parameter[2:0]	ADD = 3'b000,
					SUB = 3'b001,
					AND = 3'b010,
					OR =  3'b011,
					SLT = 3'b100,
					XOR = 3'b101,
					PASS= 3'b110;

	input [2:0] OpCode;
	input signed [WL-1:0] operand1, operand2;
	output signed [WL-1:0] result;
	output ZERO;

	reg signed [WL-1:0] result_temp;
	assign result = result_temp;

	always @(OpCode, operand1, operand2) begin
		result_temp = {(WL){1'b0}};
		case(OpCode)
			ADD: begin result_temp = operand1 + operand2; end
			SUB: begin result_temp = operand1 - operand2; end
			AND: begin result_temp = operand1 & operand2; end
			OR:  begin result_temp = operand1 | operand2; end
			SLT: begin result_temp = (operand1 < operand2) ? {{(WL-1){1'b0}}, 1'b1} : {(WL){1'b0}}; end
			XOR: begin result_temp = operand1 ^ operand2; end
			PASS:begin result_temp = operand2; end
			default: result_temp = operand2;
		endcase
	end

	assign ZERO = ~|{result};
endmodule

module ImmediateExtension (ImmSrc, Inst, Imm);

	parameter[2:0]	I_TYPE =	2'b000,
					S_TYPE =	2'b001,
					B_TYPE =	2'b010,
					J_TYPE =	2'b011,
					U_Type =	2'b100;

	input	[2:0]	ImmSrc;
	input	[31:0]	Inst;
	output	[31:0]	Imm;

	reg [31:0] Imm_temp;
	assign Imm = Imm_temp;

	always @(ImmSrc, Inst) begin
		Imm_temp = 32'b0;
		case(ImmSrc)
			I_TYPE: begin Imm_temp = {{20{Inst[31]}}, Inst[31:20]}; end
			S_TYPE: begin Imm_temp = {{20{Inst[31]}}, Inst[31:25], Inst[11:7]}; end
			B_TYPE: begin Imm_temp = {{20{Inst[31]}}, Inst[7], Inst[30:25], Inst[11:8], 1'b0}; end
			J_TYPE: begin Imm_temp = {{12{Inst[31]}}, Inst[19:12], Inst[20], Inst[30:21], 1'b0}; end
			U_Type: begin Imm_temp = {Inst[31:12], 12'b0}; end
			default: Imm_temp = 32'b0;
		endcase
	end

endmodule

module ProgramCounter #(parameter SIZE = 32) (parallelIN, clk,rst,ldEN, parallelOUT);

	input [SIZE-1:0] parallelIN;
	input clk,rst, ldEN;
	output [SIZE-1:0] parallelOUT;

	reg [SIZE-1:0] Storage;
	assign parallelOUT = Storage;

	always @(posedge rst, posedge clk) begin
		if(rst) Storage <= {(SIZE){1'b0}};
		else if (ldEN)	Storage <= parallelIN;
	end

endmodule


module RegisterFile (rs1, rs2, rd, WriteData, RegWrite, clk,rst, data1, data2);

	input [4:0] rs1, rs2, rd;
	input [31:0] WriteData;
	input RegWrite;
	input clk,rst;
	output [31:0] data1, data2;

	reg [31:0][31:0] RegFile;

	always @(posedge clk, posedge rst) begin
		if(rst) RegFile = 0;
		else begin
			if(RegWrite & (rd != 5'b00000))	RegFile[rd] <= WriteData;
		end
	end

	assign data1 = RegFile[rs1];
	assign data2 = RegFile[rs2];

endmodule


module Memory (addressIN, dataIN, writeEN, clk,rst, dataOUT);

	input [31:0] addressIN;
	input [31:0] dataIN;
	input clk,rst, writeEN;
	output [31:0] dataOUT;


	reg [7:0] dataMem [$pow(2, 16)-1:0];

	wire [31:0] adr;
    assign adr = {addressIN[31:2], 2'b00};

	always @(negedge rst) begin
		$readmemh("Data.mem", dataMem);
	end

	initial $readmemh("Data.mem", dataMem);

	integer i;
	always @(posedge clk, posedge rst) begin
        if(rst)	begin
			for(i = 0 ; i < $pow(2, 16); i = i + 1) dataMem[i] = 0;
		end
		else begin
			if (writeEN)
				{dataMem[adr + 3], dataMem[adr + 2], dataMem[adr + 1], dataMem[adr]} <= dataIN;
		end
	end


	assign dataOUT = {dataMem[adr + 3], dataMem[adr + 2], dataMem[adr + 1], dataMem[adr]};

endmodule


module Datapath (PCWrite, AdrSrc, MemWrite, IRWrite, ResultSrc, ALUControl,
				 ALUSrcA, ALUSrcB, ImmSrc, RegWrite, clk,rst, ZERO, OpCode, f3, f7);
	input PCWrite, AdrSrc, MemWrite, IRWrite;
	input [1:0] ResultSrc;
	input [2:0] ALUControl;
	input [1:0] ALUSrcA, ALUSrcB;
	input [2:0] ImmSrc;
	input RegWrite;
	input clk,rst;
	output ZERO;
	output [6:0] OpCode;
	output [2:0] f3;
	output [6:0] f7;

	supply1 VDD;
	supply0 GND;

	wire [31:0] PCNext, PC, Adr, OldPC, Instr, ReadData, MemData, ImmExt, WriteData,
						A, RD1, RD2, Result, SrcA, SrcB, ALUResult, ALUOut;
	wire [4:0] RS1, RS2, RD;

	assign PCNext = Result;

	assign OpCode = Instr[6:0];
	assign f3 = Instr[14:12];
	assign f7 = Instr[31:25];

	assign RS1 = Instr[19:15];
	assign RS2 = Instr[24:20];
	assign RD = Instr[11:7];

	ProgramCounter #(32) PrgCnt (.parallelIN(PCNext), .clk(clk),.rst(rst),.ldEN(PCWrite), .parallelOUT(PC));
	MUX #(2,32) AdrMUX (.dataIN({Result,PC}), .select(AdrSrc), .dataOUT(Adr));
	Memory IDMem (.addressIN(Adr), .dataIN(WriteData), .writeEN(MemWrite),
				  .clk(clk),.rst(rst), .dataOUT(ReadData));
	Register #(32) OldPC_Reg (.dataIN(PC), .clk(clk),.rst(rst),.ldEN(IRWrite), .dataOUT(OldPC));
	Register #(32) Instr_Reg (.dataIN(ReadData), .clk(clk),.rst(rst),.ldEN(IRWrite), .dataOUT(Instr));
	Register #(32) MemData_Reg (.dataIN(ReadData), .clk(clk),.rst(rst),.ldEN(VDD), .dataOUT(MemData));
	RegisterFile RegFile (.rs1(RS1), .rs2(RS2), .rd(RD), .WriteData(Result), .RegWrite(RegWrite),
						  .clk(clk),.rst(rst), .data1(RD1), .data2(RD2));
	Register #(32) RD1_Reg (.dataIN(RD1), .clk(clk),.rst(rst),.ldEN(VDD), .dataOUT(A));
	Register #(32) RD2_Reg (.dataIN(RD2), .clk(clk),.rst(rst),.ldEN(VDD), .dataOUT(WriteData));
	ImmediateExtension ImmExtender (.ImmSrc(ImmSrc), .Inst(Instr), .Imm(ImmExt));
	MUX #(3, 32) ALUSrcA_MUX (.dataIN({A,OldPC,PC}), .select(ALUSrcA), .dataOUT(SrcA));
	MUX #(3, 32) ALUSrcB_MUX (.dataIN({32'd4, ImmExt, WriteData}), .select(ALUSrcB), .dataOUT(SrcB));
	ALU #(32) ALU__ (.OpCode(ALUControl), .operand1(SrcA), .operand2(SrcB), .result(ALUResult), .ZERO(ZERO));
	Register #(32) ALUOut_Reg (.dataIN(ALUResult), .clk(clk),.rst(rst),.ldEN(VDD), .dataOUT(ALUOut));
	MUX #(3, 32) Result_MUX (.dataIN({ALUResult, MemData, ALUOut}), .select(ResultSrc), .dataOUT(Result));

endmodule