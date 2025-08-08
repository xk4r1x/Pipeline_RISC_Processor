// Complete 5-Stage Pipelined RISC-V CPU
// All modules in one file for EDA Playground

// Program Counter Module
module PC (
    input logic clk, reset,
    input logic [31:0] next_pc,
    output logic [31:0] pc
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            pc <= 32'h00000000;
        else
            pc <= {next_pc[31:2], 2'b00};  // Word-aligned
    end
endmodule

// Instruction Memory
module Instruction_Memory (
    input logic [31:0] address,
    output logic [31:0] instruction
);
    logic [31:0] mem [0:255];
    logic [7:0] word_addr;

    assign word_addr = address[9:2];

    initial begin
        for (int i = 0; i < 256; i++) mem[i] = 32'h00000013; // NOP

        mem[0] = 32'h00100093; // ADDI x1, x0, 1
        mem[1] = 32'h00200113; // ADDI x2, x0, 2
        mem[2] = 32'h002081B3; // ADD x3, x1, x2
        mem[3] = 32'h40208233; // SUB x4, x1, x2
        mem[4] = 32'h0020F2B3; // AND x5, x1, x2
        mem[5] = 32'h0020E333; // OR x6, x1, x2
        mem[6] = 32'h00012623; // SW x0, 12(x2)
        mem[7] = 32'h00C12383; // LW x7, 12(x2)
        mem[8] = 32'h00118463; // BEQ x3, x1, +8
        mem[9] = 32'h00100393; // ADDI x7, x0, 1
        mem[10] = 32'h00000013; // NOP
        mem[11] = 32'h00100493; // ADDI x9, x0, 1
        mem[12] = 32'hFE1482E3; // BEQ x9, x1, -12
    end

    always_comb begin
        instruction = (word_addr < 256) ? mem[word_addr] : 32'h00000013;
    end
endmodule

// Register File
module Register_File (
    input logic clk,
    input logic RegWrite,
    input logic [4:0] readReg1, readReg2, writeReg,
    input logic [31:0] writeData,
    output logic [31:0] readData1, readData2
);
    logic [31:0] registers [0:31];
    
    // Initialize register file
    initial begin
        for (int i = 0; i < 32; i++) begin
            registers[i] = 32'h00000000;
        end
    end
    
    // Combinational read
    always_comb begin
        readData1 = (readReg1 == 0) ? 32'h00000000 : registers[readReg1];
        readData2 = (readReg2 == 0) ? 32'h00000000 : registers[readReg2];
    end
    
    // Synchronous write
    always_ff @(posedge clk) begin
        if (RegWrite && writeReg != 0) begin
            registers[writeReg] <= writeData;
        end
    end
endmodule

// Control Unit
module Control_Unit (
    input logic [6:0] opcode,
    input logic [2:0] funct3,
    output logic RegWrite, MemtoReg, MemRead, MemWrite, ALUSrc, Branch, Jump,
    output logic [1:0] ALUop,
    output logic [2:0] ImmSrc
);
    always_comb begin
        RegWrite = 0; MemtoReg = 0; MemRead = 0; MemWrite = 0;
        ALUSrc = 0; Branch = 0; Jump = 0; ALUop = 2'b00; ImmSrc = 3'b000;
        case(opcode)
            7'b0110011: begin RegWrite = 1; ALUop = 2'b10; end // R-type
            7'b0010011: begin RegWrite = 1; ALUop = 2'b11; ALUSrc = 1; ImmSrc = 3'b001; end // I-type
            7'b0000011: begin RegWrite = 1; MemRead = 1; MemtoReg = 1; ALUSrc = 1; ImmSrc = 3'b001; end // Load
            7'b0100011: begin MemWrite = 1; ALUSrc = 1; ImmSrc = 3'b010; end // Store
            7'b1100011: begin Branch = 1; ALUop = 2'b01; ImmSrc = 3'b011; end // Branch
            7'b1101111: begin RegWrite = 1; Jump = 1; ImmSrc = 3'b100; end // JAL
            7'b1100111: begin RegWrite = 1; Jump = 1; ALUSrc = 1; ImmSrc = 3'b001; end // JALR
            7'b0110111, 7'b0010111: begin RegWrite = 1; ALUSrc = 1; ImmSrc = 3'b101; end // LUI / AUIPC
            default: ; // NOP
        endcase
    end
endmodule

// Immediate Generator
module Immediate_Generator (
    input logic [31:0] instruction,
    input logic [2:0] ImmSrc,
    output logic [31:0] immediate
);
    always_comb begin
        case (ImmSrc)
            3'b001: immediate = {{20{instruction[31]}}, instruction[31:20]};
            3'b010: immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
            3'b011: immediate = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
            3'b100: immediate = {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};
            3'b101: immediate = {instruction[31:12], 12'b0};
            default: immediate = 32'b0;
        endcase
    end
endmodule

// ALU Control
module ALU_Control (
    input logic [2:0] funct3,
    input logic [6:0] funct7,
    input logic [1:0] ALUop,
    output logic [3:0] ALUControl
);
    always_comb begin
        case (ALUop)
            2'b00: ALUControl = 4'b0010;
            2'b01: ALUControl = 4'b0110;
            2'b10: begin
                case(funct3)
                    3'b000: ALUControl = (funct7[5]) ? 4'b0110 : 4'b0010;
                    3'b001: ALUControl = 4'b1010;
                    3'b010: ALUControl = 4'b0111;
                    3'b011: ALUControl = 4'b1000;
                    3'b100: ALUControl = 4'b1001;
                    3'b101: ALUControl = (funct7[5]) ? 4'b1100 : 4'b1011;
                    3'b110: ALUControl = 4'b0001;
                    3'b111: ALUControl = 4'b0000;
                    default: ALUControl = 4'b0010;
                endcase
            end
            2'b11: begin
                case(funct3)
                    3'b000: ALUControl = 4'b0010;
                    3'b001: ALUControl = 4'b1010;
                    3'b010: ALUControl = 4'b0111;
                    3'b011: ALUControl = 4'b1000;
                    3'b100: ALUControl = 4'b1001;
                    3'b101: ALUControl = (funct7[5]) ? 4'b1100 : 4'b1011;
                    3'b110: ALUControl = 4'b0001;
                    3'b111: ALUControl = 4'b0000;
                    default: ALUControl = 4'b0010;
                endcase
            end
            default: ALUControl = 4'b0010;
        endcase
    end
endmodule

// ALU
module ALU (
    input logic [31:0] A, B,
    input logic [3:0] ALUControl,
    output logic [31:0] ALUResult,
    output logic Zero
);
    always_comb begin
        case(ALUControl)
            4'b0000: ALUResult = A & B;
            4'b0001: ALUResult = A | B;
            4'b0010: ALUResult = A + B;
            4'b0110: ALUResult = A - B;
            4'b0111: ALUResult = ($signed(A) < $signed(B)) ? 32'h1 : 32'h0;
            4'b1000: ALUResult = (A < B) ? 32'h1 : 32'h0;
            4'b1001: ALUResult = A ^ B;
            4'b1010: ALUResult = A << B[4:0];
            4'b1011: ALUResult = A >> B[4:0];
            4'b1100: ALUResult = $signed(A) >>> B[4:0];
            4'b1101: ALUResult = ~(A | B);
            default: ALUResult = 32'b0;
        endcase
    end
    assign Zero = (ALUResult == 32'b0);
endmodule

// Data Memory
module Data_Memory (
    input logic clk, MemWrite, MemRead,
    input logic [31:0] address, writeData,
    output logic [31:0] readData
);
    logic [31:0] mem [0:1023];
    logic [31:0] word_addr;

    assign word_addr = address >> 2;

    initial begin
        for (int i = 0; i < 1024; i++) begin
            mem[i] = 32'h00000000;
        end
        mem[0] = 32'h12345678;
        mem[1] = 32'hABCDEF00;
        mem[2] = 32'h11111111;
    end

    always_ff @(posedge clk) begin
        if (MemWrite && word_addr < 1024)
            mem[word_addr] <= writeData;
    end

    always_comb begin
        if (MemRead && word_addr < 1024)
            readData = mem[word_addr];
        else
            readData = 32'b0;
    end
endmodule

// IF/ID Pipeline Register
module IF_ID_Register (
    input logic clk, reset, stall, flush,
    input logic [31:0] PC_in, instruction_in,
    output logic [31:0] PC_out, instruction_out
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset || flush) begin
            PC_out <= 32'b0;
            instruction_out <= 32'h00000013; // NOP
        end else if (!stall) begin
            PC_out <= PC_in;
            instruction_out <= instruction_in;
        end
    end
endmodule

// ID/EX Pipeline Register
module ID_EX_Register (
    input logic clk, reset, flush,
    input logic RegWrite_in, MemtoReg_in, MemRead_in, MemWrite_in, 
                ALUSrc_in, Branch_in, Jump_in,
    input logic [1:0] ALUop_in,
    input logic [3:0] ALUControl_in,
    input logic [31:0] PC_in, reg_data1_in, reg_data2_in, imm_in,
    input logic [4:0] rs1_in, rs2_in, rd_in,
    input logic [2:0] funct3_in,
    output logic RegWrite_out, MemtoReg_out, MemRead_out, MemWrite_out,
                 ALUSrc_out, Branch_out, Jump_out,
    output logic [1:0] ALUop_out,
    output logic [3:0] ALUControl_out,
    output logic [31:0] PC_out, reg_data1_out, reg_data2_out, imm_out,
    output logic [4:0] rs1_out, rs2_out, rd_out,
    output logic [2:0] funct3_out
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset || flush) begin
            RegWrite_out <= 0; MemtoReg_out <= 0; MemRead_out <= 0; 
            MemWrite_out <= 0; ALUSrc_out <= 0; Branch_out <= 0; Jump_out <= 0;
            ALUop_out <= 2'b00; ALUControl_out <= 4'b0000;
            PC_out <= 32'b0; reg_data1_out <= 32'b0; reg_data2_out <= 32'b0;
            imm_out <= 32'b0; rs1_out <= 5'b0; rs2_out <= 5'b0; rd_out <= 5'b0;
            funct3_out <= 3'b000;
        end else begin
            RegWrite_out <= RegWrite_in; MemtoReg_out <= MemtoReg_in;
            MemRead_out <= MemRead_in; MemWrite_out <= MemWrite_in;
            ALUSrc_out <= ALUSrc_in; Branch_out <= Branch_in; Jump_out <= Jump_in;
            ALUop_out <= ALUop_in; ALUControl_out <= ALUControl_in;
            PC_out <= PC_in; reg_data1_out <= reg_data1_in; 
            reg_data2_out <= reg_data2_in; imm_out <= imm_in;
            rs1_out <= rs1_in; rs2_out <= rs2_in; rd_out <= rd_in;
            funct3_out <= funct3_in;
        end
    end
endmodule

// EX/MEM Pipeline Register
module EX_MEM_Register (
    input logic clk, reset,
    input logic RegWrite_in, MemtoReg_in, MemRead_in, MemWrite_in, Jump_in,
    input logic branch_taken_in,
    input logic [31:0] ALU_result_in, reg_data2_in, branch_target_in,
    input logic [4:0] rd_in,
    output logic RegWrite_out, MemtoReg_out, MemRead_out, MemWrite_out, Jump_out,
    output logic branch_taken_out,
    output logic [31:0] ALU_result_out, reg_data2_out, branch_target_out,
    output logic [4:0] rd_out
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            RegWrite_out <= 0; MemtoReg_out <= 0; MemRead_out <= 0;
            MemWrite_out <= 0; Jump_out <= 0; branch_taken_out <= 0;
            ALU_result_out <= 32'b0; reg_data2_out <= 32'b0; 
            branch_target_out <= 32'b0; rd_out <= 5'b0;
        end else begin
            RegWrite_out <= RegWrite_in; MemtoReg_out <= MemtoReg_in;
            MemRead_out <= MemRead_in; MemWrite_out <= MemWrite_in;
            Jump_out <= Jump_in; branch_taken_out <= branch_taken_in;
            ALU_result_out <= ALU_result_in; reg_data2_out <= reg_data2_in;
            branch_target_out <= branch_target_in; rd_out <= rd_in;
        end
    end
endmodule

// MEM/WB Pipeline Register
module MEM_WB_Register (
    input logic clk, reset,
    input logic RegWrite_in, MemtoReg_in,
    input logic [31:0] ALU_result_in, memory_data_in,
    input logic [4:0] rd_in,
    output logic RegWrite_out, MemtoReg_out,
    output logic [31:0] ALU_result_out, memory_data_out,
    output logic [4:0] rd_out
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            RegWrite_out <= 0; MemtoReg_out <= 0;
            ALU_result_out <= 32'b0; memory_data_out <= 32'b0; rd_out <= 5'b0;
        end else begin
            RegWrite_out <= RegWrite_in; MemtoReg_out <= MemtoReg_in;
            ALU_result_out <= ALU_result_in; memory_data_out <= memory_data_in;
            rd_out <= rd_in;
        end
    end
endmodule

// Forwarding Unit
module Forwarding_Unit (
    input logic EX_MEM_RegWrite, MEM_WB_RegWrite,
    input logic [4:0] EX_MEM_rd, MEM_WB_rd, ID_EX_rs1, ID_EX_rs2,
    output logic [1:0] ForwardA, ForwardB
);
    always_comb begin
        ForwardA = 2'b00;
        ForwardB = 2'b00;
        
        if (EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rs1))
            ForwardA = 2'b10;
        if (EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rs2))
            ForwardB = 2'b10;
            
        if (MEM_WB_RegWrite && (MEM_WB_rd != 0) && (MEM_WB_rd == ID_EX_rs1) && 
            !(EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rs1)))
            ForwardA = 2'b01;
        if (MEM_WB_RegWrite && (MEM_WB_rd != 0) && (MEM_WB_rd == ID_EX_rs2) &&
            !(EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rs2)))
            ForwardB = 2'b01;
    end
endmodule

// Hazard Detection Unit
module Hazard_Detection (
    input logic ID_EX_MemRead,
    input logic [4:0] ID_EX_rd, IF_ID_rs1, IF_ID_rs2,
    input logic Branch, Jump, EX_MEM_branch_taken, EX_MEM_Jump,
    output logic Stall, IF_ID_flush, ID_EX_flush
);
    always_comb begin
        if (ID_EX_MemRead && ID_EX_rd != 0 && 
            ((ID_EX_rd == IF_ID_rs1) || (ID_EX_rd == IF_ID_rs2))) begin
            Stall = 1;
            IF_ID_flush = 0;
            ID_EX_flush = 1;
        end
        else if (EX_MEM_branch_taken || EX_MEM_Jump) begin
            Stall = 0;
            IF_ID_flush = 1;
            ID_EX_flush = 1;
        end
        else begin
            Stall = 0;
            IF_ID_flush = 0;
            ID_EX_flush = 0;
        end
    end
endmodule

// Main 5-Stage Pipelined CPU
module CPU (
    input logic clk, reset
);

    // IF Stage
    logic [31:0] PC, PC_plus_4;
    logic [31:0] IF_instruction;
    
    // IF/ID Pipeline Register
    logic [31:0] IF_ID_PC, IF_ID_instruction;
    logic IF_ID_flush, stall;
    
    // ID Stage 
    logic [6:0] opcode, funct7;
    logic [4:0] rs1, rs2, rd;
    logic [2:0] funct3;
    logic [31:0] reg_data1, reg_data2, immediate;
    logic RegWrite, MemtoReg, MemRead, MemWrite, ALUSrc, Branch, Jump;
    logic [1:0] ALUop;
    logic [2:0] ImmSrc;
    logic [3:0] ALUControl;
    
    // ID/EX Pipeline Register
    logic ID_EX_RegWrite, ID_EX_MemtoReg, ID_EX_MemRead, ID_EX_MemWrite;
    logic ID_EX_ALUSrc, ID_EX_Branch, ID_EX_Jump;
    logic [1:0] ID_EX_ALUop;
    logic [3:0] ID_EX_ALUControl;
    logic [31:0] ID_EX_PC, ID_EX_reg_data1, ID_EX_reg_data2, ID_EX_imm;
    logic [4:0] ID_EX_rs1, ID_EX_rs2, ID_EX_rd;
    logic [2:0] ID_EX_funct3;
    logic ID_EX_flush;
    
    // EX Stage
    logic [31:0] ALU_A, ALU_B, ALU_result, branch_target;
    logic ALU_Zero, branch_taken;
    logic [1:0] ForwardA, ForwardB;
    
    // EX/MEM Pipeline Register
    logic EX_MEM_RegWrite, EX_MEM_MemtoReg, EX_MEM_MemRead, EX_MEM_MemWrite;
    logic EX_MEM_Jump, EX_MEM_branch_taken;
    logic [31:0] EX_MEM_ALU_result, EX_MEM_reg_data2, EX_MEM_branch_target;
    logic [4:0] EX_MEM_rd;
    
    // MEM Stage
    logic [31:0] memory_data;
    
    // MEM/WB Pipeline Register
    logic MEM_WB_RegWrite, MEM_WB_MemtoReg;
    logic [31:0] MEM_WB_ALU_result, MEM_WB_memory_data;
    logic [4:0] MEM_WB_rd;
    
    // WB Stage
    logic [31:0] write_back_data;

    assign PC_plus_4 = PC + 4;
    
    // PC update logic
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            PC <= 32'h00000000;
        else if (!stall) begin
            if (EX_MEM_branch_taken || EX_MEM_Jump)
                PC <= EX_MEM_branch_target;
            else
                PC <= PC_plus_4;
        end
    end
    
    // Module Instantiations
    Instruction_Memory instruction_memory (
        .address(PC),
        .instruction(IF_instruction)
    );

    IF_ID_Register if_id_reg (
        .clk(clk), .reset(reset), .stall(stall), .flush(IF_ID_flush),
        .PC_in(PC_plus_4), .instruction_in(IF_instruction),
        .PC_out(IF_ID_PC), .instruction_out(IF_ID_instruction)
    );

    // Decode instruction fields
    assign opcode = IF_ID_instruction[6:0];
    assign rd     = IF_ID_instruction[11:7];
    assign funct3 = IF_ID_instruction[14:12];
    assign rs1    = IF_ID_instruction[19:15];
    assign rs2    = IF_ID_instruction[24:20];
    assign funct7 = IF_ID_instruction[31:25];
    
    Control_Unit control_unit (
        .opcode(opcode), .funct3(funct3),
        .RegWrite(RegWrite), .MemtoReg(MemtoReg), .MemRead(MemRead),
        .MemWrite(MemWrite), .ALUSrc(ALUSrc), .Branch(Branch), .Jump(Jump),
        .ALUop(ALUop), .ImmSrc(ImmSrc)
    );
    
    Register_File register_file (
        .clk(clk), .RegWrite(MEM_WB_RegWrite),
        .readReg1(rs1), .readReg2(rs2), .writeReg(MEM_WB_rd),
        .writeData(write_back_data), .readData1(reg_data1), .readData2(reg_data2)
    );
    
    Immediate_Generator imm_gen (
        .instruction(IF_ID_instruction), .ImmSrc(ImmSrc), .immediate(immediate)
    );
    
    ALU_Control alu_control (
        .funct3(funct3), .funct7(funct7), .ALUop(ALUop), .ALUControl(ALUControl)
    );

    ID_EX_Register id_ex_reg (
        .clk(clk), .reset(reset), .flush(ID_EX_flush),
        .RegWrite_in(RegWrite), .MemtoReg_in(MemtoReg), .MemRead_in(MemRead),
        .MemWrite_in(MemWrite), .ALUSrc_in(ALUSrc), .Branch_in(Branch), 
        .Jump_in(Jump), .ALUop_in(ALUop), .ALUControl_in(ALUControl),
        .PC_in(IF_ID_PC), .reg_data1_in(reg_data1), .reg_data2_in(reg_data2),
        .imm_in(immediate), .rs1_in(rs1), .rs2_in(rs2), .rd_in(rd), .funct3_in(funct3),
        .RegWrite_out(ID_EX_RegWrite), .MemtoReg_out(ID_EX_MemtoReg), 
        .MemRead_out(ID_EX_MemRead), .MemWrite_out(ID_EX_MemWrite),
        .ALUSrc_out(ID_EX_ALUSrc), .Branch_out(ID_EX_Branch), .Jump_out(ID_EX_Jump),
        .ALUop_out(ID_EX_ALUop), .ALUControl_out(ID_EX_ALUControl),
        .PC_out(ID_EX_PC), .reg_data1_out(ID_EX_reg_data1), 
        .reg_data2_out(ID_EX_reg_data2), .imm_out(ID_EX_imm),
        .rs1_out(ID_EX_rs1), .rs2_out(ID_EX_rs2), .rd_out(ID_EX_rd),
        .funct3_out(ID_EX_funct3)
    );

    // Forwarding multiplexers
    always_comb begin
        case (ForwardA)
            2'b00: ALU_A = ID_EX_reg_data1;
            2'b01: ALU_A = write_back_data;
            2'b10: ALU_A = EX_MEM_ALU_result;
            default: ALU_A = ID_EX_reg_data1;
        endcase
    end
    
    logic [31:0] forwarded_B;
    always_comb begin
        case (ForwardB)
            2'b00: forwarded_B = ID_EX_reg_data2;
            2'b01: forwarded_B = write_back_data;
            2'b10: forwarded_B = EX_MEM_ALU_result;
            default: forwarded_B = ID_EX_reg_data2;
        endcase
    end
    
    assign ALU_B = ID_EX_ALUSrc ? ID_EX_imm : forwarded_B;
    
    ALU alu (
        .A(ALU_A), .B(ALU_B), .ALUControl(ID_EX_ALUControl),
        .ALUResult(ALU_result), .Zero(ALU_Zero)
    );
    
    // Branch decision logic
    assign branch_taken = ID_EX_Branch & (
        (ID_EX_funct3 == 3'b000) ?  ALU_Zero     :
        (ID_EX_funct3 == 3'b001) ? ~ALU_Zero     :
        (ID_EX_funct3 == 3'b100) ?  ALU_result[0]:
        (ID_EX_funct3 == 3'b101) ? ~ALU_result[0]:
        1'b0
    );
    
    assign branch_target = (ID_EX_Jump || branch_taken) ? (ID_EX_PC + ID_EX_imm) : PC_plus_4;

    EX_MEM_Register ex_mem_reg (
        .clk(clk), .reset(reset),
        .RegWrite_in(ID_EX_RegWrite), .MemtoReg_in(ID_EX_MemtoReg), 
        .MemRead_in(ID_EX_MemRead), .MemWrite_in(ID_EX_MemWrite), 
        .Jump_in(ID_EX_Jump), .branch_taken_in(branch_taken),
        .ALU_result_in(ALU_result), .reg_data2_in(forwarded_B), 
        .branch_target_in(branch_target), .rd_in(ID_EX_rd),
        .RegWrite_out(EX_MEM_RegWrite), .MemtoReg_out(EX_MEM_MemtoReg),
        .MemRead_out(EX_MEM_MemRead), .MemWrite_out(EX_MEM_MemWrite),
        .Jump_out(EX_MEM_Jump), .branch_taken_out(EX_MEM_branch_taken),
        .ALU_result_out(EX_MEM_ALU_result), .reg_data2_out(EX_MEM_reg_data2),
        .branch_target_out(EX_MEM_branch_target), .rd_out(EX_MEM_rd)
    );

    Data_Memory data_memory (
        .clk(clk), .MemWrite(EX_MEM_MemWrite), .MemRead(EX_MEM_MemRead),
        .address(EX_MEM_ALU_result), .writeData(EX_MEM_reg_data2), .readData(memory_data)
    );

    MEM_WB_Register mem_wb_reg (
        .clk(clk), .reset(reset),
        .RegWrite_in(EX_MEM_RegWrite), .MemtoReg_in(EX_MEM_MemtoReg),
        .ALU_result_in(EX_MEM_ALU_result), .memory_data_in(memory_data), 
        .rd_in(EX_MEM_rd),
        .RegWrite_out(MEM_WB_RegWrite), .MemtoReg_out(MEM_WB_MemtoReg),
        .ALU_result_out(MEM_WB_ALU_result), .memory_data_out(MEM_WB_memory_data),
        .rd_out(MEM_WB_rd)
    );

    assign write_back_data = MEM_WB_MemtoReg ? MEM_WB_memory_data : MEM_WB_ALU_result;

    Forwarding_Unit forwarding_unit (
        .EX_MEM_RegWrite(EX_MEM_RegWrite), .MEM_WB_RegWrite(MEM_WB_RegWrite),
        .EX_MEM_rd(EX_MEM_rd), .MEM_WB_rd(MEM_WB_rd),
        .ID_EX_rs1(ID_EX_rs1), .ID_EX_rs2(ID_EX_rs2),
        .ForwardA(ForwardA), .ForwardB(ForwardB)
    );
    
    Hazard_Detection hazard_detection (
        .ID_EX_MemRead(ID_EX_MemRead), .ID_EX_rd(ID_EX_rd),
        .IF_ID_rs1(rs1), .IF_ID_rs2(rs2),
        .Branch(Branch), .Jump(Jump),
        .EX_MEM_branch_taken(EX_MEM_branch_taken), .EX_MEM_Jump(EX_MEM_Jump),
        .Stall(stall), .IF_ID_flush(IF_ID_flush), .ID_EX_flush(ID_EX_flush)
    );

endmodule