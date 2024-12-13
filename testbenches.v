`timescale 1ns/1ns

module MUX_TB();

	parameter WL = 32, SIZE = 4;
	parameter SELECT_SIZE = $clog2(SIZE);

	reg [SIZE-1:0][WL-1:0] dataIN;
	reg [SELECT_SIZE-1:0] select;

	wire [WL-1:0] dataOUT;

	MUX #(SIZE, WL) UUT (dataIN, select, dataOUT);

	always #30 dataIN = {$random, $random, $random, $random, $random};

	always #13 select = $random;

	initial begin
		#500 $stop;
	end

endmodule


module Adder_TB();

	parameter SIZE = 32;

	reg [SIZE-1:0] A;
	reg [SIZE-1:0] B;
	wire [SIZE-1:0] S;

	Adder #(SIZE) UUT (A, B, S);

	always #20 {A, B} = {$random, $random};
	initial begin #500 $stop; end

endmodule

module ALU_TB();

	parameter WL = 32;

	parameter	ADD = 3'b000,
				SUB = 3'b001,
				AND = 3'b010,
				OR = 3'b011,
				SLT = 3'b100,
				XOR = 3'b101;

	reg [2:0] OpCode;
	reg signed [WL-1:0] operand1, operand2;
	wire signed [WL-1:0] result;
	wire ZERO;
	ALU #(WL) UUT (OpCode, operand1, operand2, result, ZERO);

	always #20 {operand1, operand2} = {$random, $random};

	integer i;
	initial begin
		OpCode = ADD;
		{operand1, operand2} = {$random, $random};
		for (i = ADD ; i <= XOR ; i = i + 1) begin
			OpCode = i;
			#100;
		end
		#1
		operand1 = 32'd25;
		operand2 = -32'd25;
		OpCode = ADD;
		#20 $stop;
	end

endmodule


module InstructionMemory_TB ();

	reg [31:0] pc;
	wire [31:0] instruction;

	InstructionMemory UUT (pc, instruction);

	initial begin pc = 0; #30 $stop; end
endmodule


module DataMemory_TB();


	reg [31:0] addressIN;
	reg [31:0] dataIN;
	reg clk,rst, writeEN;
	wire [31:0] dataOUT;

	DataMemory UUT (addressIN, dataIN, writeEN, clk,rst, dataOUT);

	initial begin clk = 1; rst = 1; addressIN = 0; dataIN = 5; writeEN = 0; end

	always #5 clk = ~clk;

	initial begin
		#25 rst = 0;
		#10 writeEN = 1;
		dataIN = 15;
		#10 writeEN = 0;
		#30 $stop;
	end


endmodule


module RISC_V_TB();

	reg clk,rst;

	RISC_V UUT (clk, rst);

	initial begin clk = 1 ; rst = 1 ;#25 rst = 1'b0; end
	always #5 clk = ~clk;
	initial begin #2610 $stop; end

endmodule