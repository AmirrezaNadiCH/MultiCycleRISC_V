module RISC_V (clk, rst);

	input clk,rst;
    wire PCWrite, AdrSrc, MemWrite, IRWrite;
	wire [1:0] ResultSrc;
	wire [2:0] ALUControl;
	wire [1:0] ALUSrcA, ALUSrcB;
    wire [2:0] ImmSrc;
	wire RegWrite;
	wire ZERO;
	wire [6:0] OpCode;
	wire [2:0] f3;
	wire [6:0] f7;

    Datapath DPTH (PCWrite, AdrSrc, MemWrite, IRWrite, ResultSrc, ALUControl,
				 ALUSrcA, ALUSrcB, ImmSrc, RegWrite, clk,rst, ZERO, OpCode, f3, f7);

    Controller CNTR (ZERO, OpCode, f3, f7, clk,rst, PCWrite, AdrSrc, MemWrite, IRWrite, ResultSrc, ALUControl,
				 ALUSrcA, ALUSrcB, ImmSrc, RegWrite);

endmodule