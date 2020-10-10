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
    wire  [1:0] ALUOp;
    wire        MemWrite;
    wire        ALUSrc;
    wire        RegWrite;
    wire        Jr;
    wire        Jal;

    // wiring
    wire  [4:0] reg_w;
    wire [31:0] alu_y;
    wire        alu_zero;
    wire [31:0] reg_bus_w;
    wire [31:0] reg_bus_x;
    wire [31:0] reg_bus_y;
    wire [31:0] alu_out;
    wire [31:0] branch_addr;
    wire [31:0] jump_addr;
    wire [31:0] jr_addr;
    wire [31:0] sign_extend;
    wire  [3:0] alu_ctrl;

    // alu
    wire  [5:0] opcode;
    wire  [5:0] funct;
    wire  [4:0] shamt;

    // program
    wire [31:0] pc_w, pc_4, pc_8;
    reg  [31:0] pc_r;

//==== instance declarations ==============================
    wire reg_WEN;
    ctrl_unit ctrl_unit_inst(
        .opcode(opcode),
        .funct(funct),
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
        .Jal(Jal)
    );

    register_file register_file_inst(
        .clk(clk),
        .rst_n(rst_n),
        .WEN(reg_WEN),
        .RW(reg_w),
        .busW(reg_bus_w),
        .RX(IR[25:21]),
        .RY(IR[20:16]),
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
        .x(reg_bus_x),
        .y(alu_y),
        .shamt(shamt),
        .out(alu_out),
        .zero(alu_zero)
    );
//==== combinational part =================================
    
    // Registers
    assign reg_WEN = RegWrite;
    assign reg_w = Jal? 5'd31 : RegDst ? IR[15:11] : IR[20:16];
    assign reg_bus_w = Jal? pc_4 : MemtoReg ? ReadDataMem : alu_out;
    
    // ALU
    assign alu_y = ALUSrc ? sign_extend: reg_bus_y;
    assign sign_extend = {{16{IR[15]}}, IR[15:0]};
    assign shamt = IR[10:6];
    assign opcode = IR[31:26];
    assign funct = IR[5:0];

    // Data Memory
    assign CEN = ~(MemWrite | MemRead);
    assign WEN = ~MemWrite;
    assign OEN = ~WEN;
    assign A = alu_out[8:2]; // HSs18n_128x32 is word addr
    assign Data2Mem = reg_bus_y;

    // PC constant create
    assign pc_4 = pc_r + 4;
    assign pc_8 = pc_r + 8;

    // PC selection
    assign branch_addr = (Branch & alu_zero)? pc_4 + ({sign_extend[29:0], 2'b0}): pc_4;
    assign jump_addr = Jump ? {pc_4[31:28], {IR[25:0], 2'b0}} : branch_addr;
    assign jr_addr = Jr ? reg_bus_x : jump_addr;
    assign pc_w = jr_addr;

    // PC output
    assign IR_addr = pc_r;

//==== sequential part =================================
    always @ (posedge clk or negedge rst_n) begin
        if (~rst_n) pc_r <= 32'b0;
        else        pc_r <= pc_w;
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
    Jal
);
    input         [5:0] opcode;
    input         [5:0] funct;
    output reg          RegDst;
    output reg          Jump;
    output reg          Branch;
    output reg          MemRead;
    output reg          MemtoReg;
    output reg    [1:0] ALUOp;
    output reg          MemWrite;
    output reg          ALUSrc;
    output reg          RegWrite;
    output reg          Jr;
    output reg          Jal;
    
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
                ALUOp[1] = 0;
                ALUOp[0] = 0;
                Jr = 1;
                Jump = 0;
                Jal = 0; 
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
                ALUOp[1] = 1;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 0;
                Jal = 0;                
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
                RegWrite = 1;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                ALUOp[1] = 0;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 0;
                Jal = 0;
            end
            // lw
            6'h23: begin
                RegDst = 0;
                ALUSrc = 1;
                MemtoReg = 1;
                RegWrite = 1;
                MemRead = 1;
                MemWrite = 0;
                Branch = 0;
                ALUOp[1] = 0;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 0;
                Jal = 0;             
            end
            // sw
            6'h2b: begin
                RegDst = 0;
                ALUSrc = 1;
                MemtoReg = 0;
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 1;
                Branch = 0;
                ALUOp[1] = 0;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 0;
                Jal = 0;
            end
            // beq
            6'h4: begin
                RegDst = 0;
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 0;
                Branch = 1;
                ALUOp[1] = 0;
                ALUOp[0] = 1;
                Jr = 0;
                Jump = 0;
                Jal = 0;
            end
            // bne
            6'h5: begin
                RegDst = 0;
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 0;
                Branch = 1;
                ALUOp[1] = 1;
                ALUOp[0] = 1;
                Jr = 0;
                Jump = 0;
                Jal = 0;
            end
            // j
            6'h2: begin
                RegDst = 0;
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                ALUOp[1] = 0;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 1;
                Jal = 0;
            end
            // jal
            6'h3: begin
                RegDst = 0;
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite = 1;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                ALUOp[1] = 0;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 1;
                Jal = 1;
            end
            default: begin
                RegDst = 0;
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                ALUOp[1] = 0;
                ALUOp[0] = 0;
                Jr = 0;
                Jump = 0;
                Jal = 0;
            end
        endcase
        end
    end
endmodule

//==== register file =================================
module register_file(
    clk,
    rst_n,
    WEN,
    RW,
    busW,
    RX,
    RY,
    busX,
    busY
);
    input               clk, WEN, rst_n;
    input [4:0]         RW, RX, RY;
    input [31:0]        busW;
    output reg [31:0]   busX, busY;

    reg [31:0]          r_r [31:0], r_w [31:0];

    integer i;
    // combinational part
    always @(*) begin
        busX = r_r[RX];
        busY = r_r[RY];
        // initialize r_w
        for(i = 0; i <= 5'd31; i = i + 1) begin
            if(i == 0) begin
                r_w[0] = 32'b0;
            end
            else begin
                r_w[i] = r_r[i];
            end
        end
        // write r_w
        if(WEN) begin
            r_w[RW] = busW;
        end
    end

    // sequential part
    always @ (posedge clk or negedge rst_n) begin
        // update r_r
        if(~rst_n) begin
            for(i = 0; i <= 5'd31; i = i + 1) begin
                r_r[i] <= 5'd0;
            end
        end
        else begin
            for(i = 0; i <= 5'd31; i = i + 1) begin
                r_r[i] <= r_w[i];
            end
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
    input [1:0] ALUOp;
    output reg [3:0] alu_ctrl;

    always @ (*) begin
        if(ALUOp == 2'b10) begin
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
                6'b000000: alu_ctrl = 4'b1000;
                // srl
                6'b000010: alu_ctrl = 4'b1001;
                default: alu_ctrl = 4'b0000;
            endcase
        end
        else begin
            case(ALUOp)
                // lw, sw
                2'b00: begin
                    alu_ctrl = 4'b0010;
                end
                // beq
                2'b01: begin
                    alu_ctrl = 4'b0110;
                end
                // bne
                2'b11: begin
                    alu_ctrl = 4'b1010;
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
    x,
    y,
    shamt,
    out,
    zero
);
    input [3:0] ctrl;
    input [31:0] x;
    input [31:0] y;
    input [4:0] shamt;
    output reg [31:0] out;
    output reg zero;

    always @ (*) begin
        // default 0
        zero = 1'b0;

        case(ctrl)
            // and
            4'b0000: begin
                out = x & y;
            end
            // or
            4'b0001: begin
                out = x | y;
            end
            // add
            4'b0010: begin
                out = x + y;
            end
            // sub
            4'b0110: begin
                out = x - y;
                zero = (out == 32'b0) ? 1'b1: 1'b0;
            end
            // slt
            4'b0111: begin
                out = ($signed(x) < $signed(y)) ? 1'b1: 1'b0;
            end
            // shift left
            4'b1000: begin
                out = y << shamt;
            end
            // shift right
            4'b1001: begin
                out = y >> shamt;
            end
            // sub inverse
            4'b1010: begin
                out = x - y;
                zero = (out != 32'b0) ? 1'b1: 1'b0;
            end
            default: out = 1'b0;
        endcase
    end
endmodule