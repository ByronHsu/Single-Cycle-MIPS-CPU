`timescale 1 ns/10 ps
`define CYCLE 14.0
`define SDFFILE     "./SingleCycleMIPS_syn.sdf"   // Modify your sdf file name
`include "HSs18n_128x32.v"



`define IGLD_INIT "golden/inst_golden.txt"

`ifdef Baseline
	`define IMEM_INIT "inst/inst.txt"
	`define DMEM_INIT "data/mem_data.txt"
	`define GOLD_INIT "golden/golden.txt"
`endif
`ifdef FPU
	`define DMEM_INIT "data/fp_mem_data.txt"
	`ifdef Single
		`define IMEM_INIT "inst/fp_inst.txt"
		`define GOLD_INIT "golden/fp_golden.txt"
	`endif
	`ifdef Double
		`define IMEM_INIT "inst/fp_inst_double.txt"
		`define GOLD_INIT "golden/fp_golden_double.txt"	
	`endif
`endif
module SingleCycle_tb;

	reg clk;
	reg rst_n;
	wire [31:0] IR_addr;
	wire [31:0] IR;
	wire [31:0] ReadDataMem;
	wire CEN;
	wire WEN;
	wire [6:0] A;
	wire [31:0] Data2Mem;
	wire OEN;

	reg	 [31:0] ans[0:100];
	reg  [7:0]  inst_ans[0:2700];
	integer error;
	integer i, check_count;

	// Instruction memory
	ROM128x32 i_rom(
		.addr(IR_addr[8:2]),
		.data(IR)
	);

	SingleCycleMIPS SingleCycleMIPS(
		clk,
		rst_n,
		IR_addr,
		IR,
		ReadDataMem,
		CEN,
		WEN,
		A,
		Data2Mem,
		OEN
	);

	HSs18n_128x32 Data_memory(
		ReadDataMem,
		~clk,
		CEN,
		WEN,
		A,
		Data2Mem,
		OEN
	);

`ifdef SDF
	initial $sdf_annotate(`SDFFILE, SingleCycleMIPS);
`endif

// Initialize the data memory
	initial begin
		$readmemh (`DMEM_INIT, Data_memory.mem);
		$readmemh (`GOLD_INIT, ans);
		$readmemb (`IGLD_INIT, inst_ans);
	end

// Dump waveform file
	initial begin
		$fsdbDumpfile("MIPS.fsdb");
		$fsdbDumpvars(0,SingleCycle_tb,"+mda"); //This command is for dumping 2D array
		$fsdbDumpvars;								
	end
	
//initial rst_n, value
	initial begin
		clk = 0;
		error = 0;
		i = 0;

		rst_n = 1'b1;
		#(`CYCLE*0.2) rst_n = 1'b0;
		#(`CYCLE*2) rst_n = 1'b1;
	end

// clk generation
	always begin
		#(`CYCLE*0.5) clk = ~clk;
	end
	
// check correctness
`ifdef Baseline
	always@(negedge clk)begin
		if (IR_addr >= 8'd4 && rst_n == 1)begin
			if (IR_addr[7:0] !== inst_ans[i]) begin
				error = 1;
            end
			
			i <= i+1;
		end		
	
	end
		
`endif
	
	always@(*)begin
		if (ReadDataMem == 32'h7fffffff && IR_addr == 8'd8)begin
			if (error==1) 
				$display("\norder of Single cycle MIPS instructions error!!!\n");
			`ifdef SDF
				$display("using SDF File %s  for this simulation.", `SDFFILE);
			`endif
			
			for (i=1; i<=ans[0]; i=i+1)begin
				if (Data_memory.mem[i] !== ans[i])begin
					$display("\nerror in memory%d, expected: %h, your: %h", i, ans[i], Data_memory.mem[i]);
					error = error + 1;
				end
			end
			if (error == 0)begin
				$display("=======================The test result is ..... PASS=========================");
				$display("\n");
				`ifdef FPU
					$display("				FPU Version \n");
				`endif
				$display("        *****************************************************              ");
				$display("        **                                                 **      /|__/|");
				$display("        **               Congratulations !!                **     / O,O  \\");
				$display("        **                                                 **    /_____   \\");
				$display("        **   All instructions have been done successfully! **   /^ ^ ^ \\  |");
				$display("        **                                                 **  |^ ^ ^ ^ |w|");
				$display("        *****************************************************   \\m___m__|_|");
				$display("\n");
				$display("============================================================================");
			end
			else begin
				$display("-----------------------------------------------------\n");
				$display("                 Error!!!                   \n");
				$display("      There are %d error with your code!       \n", error);
				$display("--------The test result is .....FAIL ----------------\n");
				$display("-----------------------------------------------------\n");			
			end
			$finish;
			
		end
	
	end
	
	
initial begin
	#(`CYCLE*5000)
	$display("-----------------------------------------------------\n");
	$display("      Error!!! Running out of time!                  \n");
	$display("      There is something wrong with your code!       \n");
 	$display("--------The test result is .....FAIL ----------------\n");
 	$display("-----------------------------------------------------\n");
	$finish;
end
	
endmodule

module ROM128x32 (
	addr,
	data
);
	input [6:0] addr;
	output [31:0] data;
	reg [31:0] data;
	reg [31:0] mem [0:127];
		
	integer i;
	initial begin
		// Initialize the instruction memory
		$readmemh (`IMEM_INIT, mem);
		$display("Reading instruction memory......");
		

	end	
	
	always @(addr) data = mem[addr];
	
endmodule