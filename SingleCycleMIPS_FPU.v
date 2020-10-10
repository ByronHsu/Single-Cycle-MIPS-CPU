module SingleCycleMIPS( 
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

//==== in/out declaration =================================
    //-------- processor ----------------------------------
    input         clk, rst_n;
    input  [31:0] IR;
    output [31:0] IR_addr;
    //-------- data memory --------------------------------
    input  [31:0] ReadDataMem;  
    output        CEN;  
    output        WEN;  
    output  [6:0] A;  
    output [31:0] Data2Mem;
    output        OEN;  

//==== reg/wire declaration ===============================
    // control signals
    wire        RegDst;
    wire        Jump;
    wire        Branch;
    wire        MemRead;
    wire        MemtoReg;
    wire  [2:0] ALUOp;
    wire        MemWrite;
    wire        ALUSrc;
    wire  [1:0] RegWrite;
    wire        Jr;
    wire        Jal;
    wire        OpDouble;
    wire        RegReadType;
    wire        FPCond;

    // wiring
    wire  [1:0] reg_WEN;
    wire  [4:0] reg_x;
    wire  [4:0] reg_y;
    wire  [4:0] reg_w;
    wire [63:0] alu_y;
    wire        alu_zero;
    wire [63:0] reg_bus_w;
    wire [63:0] reg_bus_x;
    wire [63:0] reg_bus_y;
    wire [63:0] alu_out;
    wire [31:0] branch_addr;
    wire [31:0] jump_addr;
    wire [31:0] jr_addr;
    wire [63:0] sign_extend;
    wire  [3:0] alu_ctrl;

    // alu
    wire  [5:0] opcode;
    wire  [5:0] funct;
    wire  [4:0] shamt;
    wire  [5:0] fmt;

    // program
    wire [31:0] pc_w, pc_4, pc_8;
    wire        pc_stall_w;
    reg  [31:0] pc_r;
    reg         pc_stall_r;

//==== instance declarations ==============================
    ctrl_unit ctrl_unit_inst(
        .opcode(opcode),
        .funct(funct),
        .fmt(fmt),
        .RegDst(RegDst),
        .Jump(Jump),
        .Branch(Branch),
        .MemRead(MemRead),
        .MemtoReg(MemtoReg),
        .ALUOp(ALUOp),
        .MemWrite(MemWrite),
        .ALUSrc(ALUSrc),
        .RegWrite(RegWrite),
        .Jr(Jr),
        .Jal(Jal),
        .OpDouble(OpDouble),
        .RegReadType(RegReadType),
        .FPCond(FPCond)
    );

    register_file register_file_inst(
        .clk(clk),
        .rst_n(rst_n),
        .FPCond(FPCond),
        .OpDouble(OpDouble),
        .WEN(reg_WEN),
        .RW(reg_w),
        .busW(reg_bus_w),
        .RX(reg_x),
        .RY(reg_y),
        .busX(reg_bus_x),
        .busY(reg_bus_y)
    );

    alu_ctrl_unit alu_ctrl_unit_inst(
        .funct(funct),
        .ALUOp(ALUOp),
        .alu_ctrl(alu_ctrl)
    );

    alu alu_inst(
        .ctrl(alu_ctrl),
        .aug_x(reg_bus_x),
        .aug_y(alu_y),
        .shamt(shamt),
        .aug_out(alu_out),
        .zero(alu_zero)
    );
//==== combinational part =================================
    
    // Registers
    assign reg_WEN = RegWrite[1:0];
    assign reg_x = RegReadType? IR[15:11] : IR[25:21];
    assign reg_y = pc_stall_r? IR[20:16]+1 : IR[20:16];
    assign reg_w = Jal? 5'd31 : pc_stall_r? IR[20:16]+1: RegReadType? IR[10:6] : RegDst ? IR[15:11] : IR[20:16];
    assign reg_bus_w = Jal? {2'b0, pc_4[31:2], 32'b0} : (RegWrite[0] & RegWrite[1] & ~MemtoReg)? reg_bus_y: MemtoReg ? {ReadDataMem, 32'b0} : alu_out;
    // ALU
    assign alu_y = ALUSrc ? sign_extend: reg_bus_y;
    assign sign_extend = pc_stall_r? {{16{IR[15] + 4}}, IR[15:0] + 4, 32'b0} : {{16{IR[15]}}, IR[15:0], 32'b0};
    assign shamt = IR[10:6];
    assign opcode = IR[31:26];
    assign funct = IR[5:0];
    assign fmt = IR[25:21];

    // Data Memory
    assign CEN = ~(MemWrite | MemRead);
    assign WEN = ~MemWrite;
    assign OEN = ~WEN;
    assign A = alu_out[40:34]; // HSs18n_128x32 is word addr
    assign Data2Mem = reg_bus_y[63:32];

    // PC constant create
    assign pc_4 = pc_r + 4;
    assign pc_8 = pc_r + 8;

    // PC selection
    assign branch_addr = (Branch & alu_zero)? pc_4 + ({sign_extend[61:32], 2'b0}): pc_4;
    assign jump_addr = Jump ? {pc_4[31:28], {IR[25:0], 2'b0}} : branch_addr;
    assign jr_addr = Jr ? {reg_bus_x[61:32], 2'b0} : jump_addr;
    assign pc_stall_w = (OpDouble & ~pc_stall_r & ~ALUOp[2]);
    assign pc_w = pc_stall_w ? pc_r : jr_addr;

    // PC output
    assign IR_addr = pc_r;

//==== sequential part =================================
    always @ (posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            pc_r <= 32'b0;
            pc_stall_r <= 1'b0;
        end
        else begin
            pc_r <= pc_w;
            pc_stall_r <= pc_stall_w;
        end
    end

endmodule

// recommend you to use submodule for easier debugging 
//=========================================================
//Example:
//	module ctrl(
//		clk,
//		rst_n, ....


//	);

//	endmodule

//==== control unit =================================
module ctrl_unit(
    opcode,
    funct,
    fmt,
    RegDst,
    Jump,
    Branch,
    MemRead,
    MemtoReg,
    ALUOp,
    MemWrite,
    ALUSrc,
    RegWrite,
    Jr,
    Jal,
    OpDouble,
    RegReadType,
    FPCond
);
    input         [5:0] opcode;
    input         [5:0] funct;
    input         [5:0] fmt;
    output reg          RegDst;
    output reg          Jump;
    output reg          Branch;
    output reg          MemRead;
    output reg          MemtoReg;
    output reg    [2:0] ALUOp;
    output reg          MemWrite;
    output reg          ALUSrc;
    output reg    [1:0] RegWrite;
    output reg          Jr;
    output reg          Jal;
    output reg          OpDouble;
    output reg          RegReadType;
    output reg          FPCond;
    
    always @ (*) begin
        // R-type
        if (opcode == 6'h0) begin
            // jr
            if (funct == 6'h8) begin
                RegDst = 0;
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                ALUOp[2] = 0;
                ALUOp[1] = 0;
                ALUOp[0] = 0;
                Jr = 1;
                Jump = 0;
                Jal = 0;
                OpDouble = 0;
                RegReadType = 0;
                FPCond = 0;
            end
            // others
            else begin
                RegDst = 1;
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite = 1;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                ALUOp[2] = 0;
                ALUOp[1] = 1;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 0;
                Jal = 0;
                OpDouble = 0;
                RegReadType = 0;
                FPCond = 0;               
            end
        end
        // F-type
        else if(opcode == 6'h11) begin
            // Single
            if (fmt == 6'h10) begin
                RegDst = 1;
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite[1] = 1;
                RegWrite[0] = 0;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                ALUOp[2] = 1;
                ALUOp[1] = 0;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 0;
                Jal = 0;
                OpDouble = 0;
                RegReadType = 1;
                FPCond = (funct == 6'h32) ? 1 : 0;
            end
            // Double
            else if (fmt == 6'h11) begin
                RegDst = 1;
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite[1] = 1;
                RegWrite[0] = 0;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                ALUOp[2] = 1;
                ALUOp[1] = 1;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 0;
                Jal = 0;
                OpDouble = 1;
                RegReadType = 1;
                FPCond = 0;
            end
            // bclt
            else begin
                RegDst = 1;
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite[1] = 0;
                RegWrite[0] = 0;
                MemRead = 0;
                MemWrite = 0;
                Branch = 1;
                ALUOp[2] = 0;
                ALUOp[1] = 0;
                ALUOp[0] = 1;
                Jr = 0;
                Jump = 0;
                Jal = 0;
                OpDouble = 0;
                RegReadType = 1;
                FPCond = 1;
            end
        end
        // I, J-type
        else begin
        case (opcode)
            // addi
            6'h8: begin
                RegDst = 0;
                ALUSrc = 1;
                MemtoReg = 0;
                RegWrite[1] = 0;
                RegWrite[0] = 1;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                ALUOp[2] = 0;
                ALUOp[1] = 0;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 0;
                Jal = 0;
                OpDouble = 0;
                RegReadType = 0;
                FPCond = 0;
            end
            // lw
            6'h23: begin
                RegDst = 0;
                ALUSrc = 1;
                MemtoReg = 1;
                RegWrite[1] = 0;
                RegWrite[0] = 1;
                MemRead = 1;
                MemWrite = 0;
                Branch = 0;
                ALUOp[2] = 0;
                ALUOp[1] = 0;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 0;
                Jal = 0; 
                OpDouble = 0;
                RegReadType = 0;
                FPCond = 0;           
            end
            // sw
            6'h2b: begin
                RegDst = 0;
                ALUSrc = 1;
                MemtoReg = 0;
                RegWrite[1] = 0;
                RegWrite[0] = 0;
                MemRead = 0;
                MemWrite = 1;
                Branch = 0;
                ALUOp[2] = 0;
                ALUOp[1] = 0;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 0;
                Jal = 0;
                OpDouble = 0;
                RegReadType = 0;
                FPCond = 0;
            end
            // lwcl, ldcl
            6'h31: begin
                RegDst = 0;
                ALUSrc = 1;
                MemtoReg = 1;
                RegWrite[1] = 1;
                RegWrite[0] = 1;
                MemRead = 1;
                MemWrite = 0;
                Branch = 0;
                ALUOp[2] = 0;
                ALUOp[1] = 0;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 0;
                Jal = 0; 
                OpDouble = 0;
                RegReadType = 0;
                FPCond = 0;          
            end
            6'h35: begin
                RegDst = 0;
                ALUSrc = 1;
                MemtoReg = 1;
                RegWrite[1] = 1;
                RegWrite[0] = 1;
                MemRead = 1;
                MemWrite = 0;
                Branch = 0;
                ALUOp[2] = 0;
                ALUOp[1] = 0;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 0;
                Jal = 0; 
                OpDouble = 1;
                RegReadType = 0;
                FPCond = 0;          
            end
            // swcl, sdcl
            6'h39: begin
                RegDst = 0;
                ALUSrc = 1;
                MemtoReg = 0;
                RegWrite[1] = 1;
                RegWrite[0] = 1;
                MemRead = 0;
                MemWrite = 1;
                Branch = 0;
                ALUOp[2] = 0;
                ALUOp[1] = 0;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 0;
                Jal = 0;
                OpDouble = 0;
                RegReadType = 0;
                FPCond = 0;
            end
            6'h3D: begin
                RegDst = 0;
                ALUSrc = 1;
                MemtoReg = 0;
                RegWrite[1] = 1;
                RegWrite[0] = 1;
                MemRead = 0;
                MemWrite = 1;
                Branch = 0;
                ALUOp[2] = 0;
                ALUOp[1] = 0;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 0;
                Jal = 0;
                OpDouble = 1;
                RegReadType = 0;
                FPCond = 0;
            end
            // beq
            6'h4: begin
                RegDst = 0;
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite[1] = 0;
                RegWrite[0] = 0;
                MemRead = 0;
                MemWrite = 0;
                Branch = 1;
                ALUOp[2] = 0;
                ALUOp[1] = 0;
                ALUOp[0] = 1;
                Jr = 0;
                Jump = 0;
                Jal = 0;
                OpDouble = 0;
                RegReadType = 0;
                FPCond = 0;
            end
            // bne
            6'h5: begin
                RegDst = 0;
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite[1] = 0;
                RegWrite[0] = 0;
                MemRead = 0;
                MemWrite = 0;
                Branch = 1;
                ALUOp[2] = 0;
                ALUOp[1] = 1;
                ALUOp[0] = 1;
                Jr = 0;
                Jump = 0;
                Jal = 0;
                OpDouble = 0;
                RegReadType = 0;
                FPCond = 0;
            end
            // j
            6'h2: begin
                RegDst = 0;
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite[1] = 0;
                RegWrite[0] = 0;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                ALUOp[2] = 0;
                ALUOp[1] = 0;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 1;
                Jal = 0;
                OpDouble = 0;
                RegReadType = 0;
                FPCond = 0;
            end
            // jal
            6'h3: begin
                RegDst = 0;
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite[1] = 0;
                RegWrite[0] = 1;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                ALUOp[2] = 0;
                ALUOp[1] = 0;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 1;
                Jal = 1;
                OpDouble = 0;
                RegReadType = 0;
                FPCond = 0;
            end
            default: begin
                RegDst = 0;
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite[1] = 0;
                RegWrite[0] = 0;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                ALUOp[2] = 0;
                ALUOp[1] = 0;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 0;
                Jal = 0;
                OpDouble = 0;
                RegReadType = 0;
                FPCond = 0;
            end
        endcase
        end
    end
endmodule

//==== register file =================================
module register_file(
    clk,
    rst_n,
    FPCond,
    OpDouble,
    WEN,
    RW,
    busW,
    RX,
    RY,
    busX,
    busY
);
    input               clk, rst_n, FPCond, OpDouble;
    input [1:0]         WEN;
    input [4:0]         RW, RX, RY;
    input [63:0]        busW;
    output reg [63:0]   busX, busY;

    reg [31:0]          r_r [31:0], r_w [31:0];
    reg [31:0]          f_r [31:0], f_w [31:0];
    reg [31:0]          fc_r, fc_w;


    integer i;
    // combinational part
    always @(*) begin

        case (WEN)
            2'b11: begin
                busX = {r_r[RX], 32'b0};
                busY = {f_r[RY], 32'b0};
            end
            2'b10: begin
                if(OpDouble == 0) begin
                    busX = {f_r[RX], 32'b0};
                    busY = {f_r[RY], 32'b0};
                end
                else begin
                    busX = {f_r[RX], f_r[RX+1]};
                    busY = {f_r[RY], f_r[RY+1]};
                end
            end
            2'b01: begin
                busX = {r_r[RX], 32'b0};
                busY = {r_r[RY], 32'b0};
            end
            2'b00: begin
                // Check bclt
                busX = FPCond ? {32'b1, 32'b0}: {r_r[RX], 32'b0};
                busY = FPCond ? {fc_r, 32'b0}: {r_r[RY], 32'b0};
            end
        endcase

        for(i = 0; i <= 5'd31; i = i + 1) begin
            if(i == 0) begin
                r_w[0] = 32'b0;
                f_w[0] = f_r[0];
            end
            else begin
                r_w[i] = r_r[i];
                f_w[i] = f_r[i];
            end
        end
        fc_w = fc_r;

        case (WEN)
            2'b11: begin
                f_w[RW] = busW[63:32];
            end
            2'b10: begin
                if(FPCond) begin
                    fc_w = busW[63:32];
                end
                else if(OpDouble) begin
                    f_w[RW] = busW[63:32];
                    f_w[RW+1] = busW[31:0];
                end
                else begin
                    f_w[RW] = busW[63:32];
                end
            end
            2'b01: begin
                r_w[RW] = busW[63:32];
            end
            2'b00: begin
            end
        endcase
    end

    // sequential part
    always @ (posedge clk or negedge rst_n) begin
        // update r_r
        if(~rst_n) begin
            for(i = 0; i <= 5'd31; i = i + 1) begin
                r_r[i] <= 32'b0;
                f_r[i] <= 32'b0;
            end
            fc_r <= 1'b0;
        end
        else begin
            for(i = 0; i <= 5'd31; i = i + 1) begin
                r_r[i] <= r_w[i];
                f_r[i] <= f_w[i];
            end
            fc_r <= fc_w;
        end
    end

endmodule

//==== alu ctrl unit =================================
module alu_ctrl_unit(
    funct,
    ALUOp,
    alu_ctrl
);
    input [5:0] funct;
    input [2:0] ALUOp;
    output reg [3:0] alu_ctrl;

    always @ (*) begin
        // R type
        if(ALUOp == 3'b010) begin
            case(funct)
                // add
                6'b100000: alu_ctrl = 4'b0010;
                // sub
                6'b100010: alu_ctrl = 4'b0110;
                // and
                6'b100100: alu_ctrl = 4'b0000;
                // or
                6'b100101: alu_ctrl = 4'b0001;
                // slt
                6'b101010: alu_ctrl = 4'b0111;
                // sll
                6'b000000: alu_ctrl = 4'b0011;
                // srl
                6'b000010: alu_ctrl = 4'b0101;
                default: alu_ctrl = 4'b0000;
            endcase
        end
        // F type
        else if(ALUOp == 3'b100) begin
            case(funct)
                // add.s
                6'h00: alu_ctrl = 4'b1001;
                // sub.s
                6'h01: alu_ctrl = 4'b1010;
                // mul.s
                6'h02: alu_ctrl = 4'b1011;
                // div.s
                6'h03: alu_ctrl = 4'b1100;
                // c.eq.s
                6'h32: alu_ctrl = 4'b1101;
                default: alu_ctrl = 4'b0000;
            endcase
        end
        else if(ALUOp == 3'b110) begin
            case(funct)
                // add.d
                6'h00: alu_ctrl = 4'b1110;
                // sub.d
                6'h01: alu_ctrl = 4'b1111;
                default: alu_ctrl = 4'b0000;
            endcase
        end
        else begin
            case(ALUOp)
                // lw, sw
                3'b000: begin
                    alu_ctrl = 4'b0010;
                end
                // beq
                3'b001: begin
                    alu_ctrl = 4'b0110;
                end
                // bne
                3'b011: begin
                    alu_ctrl = 4'b1000;
                end
                default: begin
                    alu_ctrl = 4'b0000;
                end
            endcase
        end
    end
endmodule

//==== ALU =================================
module alu(
    ctrl,
    aug_x,
    aug_y,
    shamt,
    aug_out,
    zero
);
    input [3:0] ctrl;
    input [63:0] aug_x;
    input [63:0] aug_y;
    input [4:0] shamt;
    output reg [63:0] aug_out;
    output reg zero;

    wire [31:0] add_out;
    wire [31:0] sub_out;
    wire [31:0] mul_out;
    wire [31:0] div_out;
    wire [7:0] status;
    wire [2:0] rnd;
    wire [63:0] add_d_out;
    wire [63:0] sub_d_out;
    wire [31:0] x;
    wire [31:0] y;
    reg [31:0] out;

    assign rnd = 3'b000;
    assign x = aug_x[63:32];
    assign y = aug_y[63:32];

    DW_fp_add #(.sig_width(52), .exp_width(11), .ieee_compliance(0)) fp_add_d_inst(
        .a (aug_x),
        .b (aug_y),
        .rnd (rnd),
        .z (add_d_out),
        .status (status)
    );

    DW_fp_sub #(.sig_width(52), .exp_width(11), .ieee_compliance(0)) fp_sub_d_inst(
        .a (aug_x),
        .b (aug_y),
        .rnd (rnd),
        .z (sub_d_out),
        .status (status)
    );

    DW_fp_add fp_add_inst(
        .a (x),
        .b (y),
        .rnd (rnd),
        .z (add_out),
        .status (status)
    );

    DW_fp_sub fp_sub_inst(
        .a (x),
        .b (y),
        .rnd (rnd),
        .z (sub_out),
        .status (status)
    );

    DW_fp_mult fp_mult_inst(
        .a (x),
        .b (y),
        .rnd (rnd),
        .z (mul_out),
        .status (status)
    );

    DW_fp_div fp_div_inst(
        .a (x),
        .b (y),
        .rnd (rnd),
        .z (div_out),
        .status (status)
    );

    always @ (*) begin
        // default 0
        zero = 1'b0;

        case(ctrl)
            // and
            4'b0000: begin
                out = x & y;
                aug_out = {out, 32'b0};
            end
            // or
            4'b0001: begin
                out = x | y;
                aug_out = {out, 32'b0};
            end
            // add
            4'b0010: begin
                out = x + y;
                aug_out = {out, 32'b0};
            end
            // sub
            4'b0110: begin
                out = x - y;
                zero = (out == 32'b0) ? 1'b1: 1'b0;
                aug_out = {out, 32'b0};
            end
            // slt
            4'b0111: begin
                out = ($signed(x) < $signed(y)) ? 32'b1: 32'b0;
                aug_out = {out, 32'b0};
            end
            // shift left
            4'b0011: begin
                out = y << shamt;
                aug_out = {out, 32'b0};
            end
            // shift right
            4'b0101: begin
                out = y >> shamt;
                aug_out = {out, 32'b0};
            end
            // sub inverse
            4'b1000: begin
                out = x - y;
                zero = (out != 32'b0) ? 1'b1: 1'b0;
                aug_out = {out, 32'b0};
            end
            // add.s
            4'b1001:begin
                out = add_out;
                aug_out = {out, 32'b0};
            end
            // sub.s
            4'b1010:begin
                out = sub_out;
                aug_out = {out, 32'b0};
            end
            // mul.s
            4'b1011:begin
                out = mul_out;
                aug_out = {out, 32'b0};
            end
            // div.s
            4'b1100:begin
                out = div_out;
                aug_out = {out, 32'b0};
            end
            // c.eq.s
            4'b1101:begin
                out = (sub_out == 32'b0)? 32'b1: 32'b0;
                aug_out = {out, 32'b0};
            end
            // add.d
            4'b1110:begin
                aug_out = add_d_out;
            end
            // sub.d
            4'b1111:begin
                aug_out = sub_d_out;
            end
            default: aug_out = 64'b0;
        endcase
    end
endmodule