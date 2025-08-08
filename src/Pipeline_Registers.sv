/*
 * Pipeline Register Modules for 5-Stage RISC-V CPU
 * 
 * These modules implement the pipeline registers that separate the five pipeline stages:
 * 1. IF/ID: Instruction Fetch / Instruction Decode
 * 2. ID/EX: Instruction Decode / Execute  
 * 3. EX/MEM: Execute / Memory Access
 * 4. MEM/WB: Memory Access / Write Back
 * 
 * Each pipeline register stores the state needed by subsequent stages and
 * supports pipeline control signals like stall and flush for hazard handling.
 */

//==============================================================================
// IF/ID Pipeline Register
//
// Stores the fetched instruction and PC+4 value between the Instruction Fetch
// and Instruction Decode stages. Supports stall (for load-use hazards) and
// flush (for control hazards) operations.
//==============================================================================

module IF_ID_Register (
    input logic clk,                    // Clock signal
    input logic reset,                  // Reset signal (active high)
    input logic stall,                  // Stall signal (hold current values)
    input logic flush,                  // Flush signal (insert NOP)
    
    // Inputs from IF stage
    input logic [31:0] PC_in,           // PC+4 value from IF stage
    input logic [31:0] instruction_in,  // Fetched instruction from IF stage
    
    // Outputs to ID stage  
    output logic [31:0] PC_out,         // PC+4 value to ID stage
    output logic [31:0] instruction_out // Instruction to ID stage
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset || flush) begin
            // Reset or flush: Clear pipeline register (insert NOP)
            PC_out <= 32'b0;
            instruction_out <= 32'h00000013; // NOP instruction
        end else if (!stall) begin
            // Normal operation: Pass data through
            PC_out <= PC_in;
            instruction_out <= instruction_in;
        end
        // If stall is asserted: Keep current values (don't update)
    end
endmodule

//==============================================================================
// ID/EX Pipeline Register
//
// Stores decoded instruction information, control signals, register data,
// and immediate values between the Instruction Decode and Execute stages.
// This is the largest pipeline register as it carries most of the instruction state.
//==============================================================================

module ID_EX_Register (
    input logic clk,                    // Clock signal
    input logic reset,                  // Reset signal (active high)
    input logic flush,                  // Flush signal (insert bubble)
    
    // Control signal inputs from ID stage
    input logic RegWrite_in,            // Register write enable
    input logic MemtoReg_in,            // Memory to register mux control
    input logic MemRead_in,             // Memory read enable
    input logic MemWrite_in,            // Memory write enable
    input logic ALUSrc_in,              // ALU source mux control
    input logic Branch_in,              // Branch instruction flag
    input logic Jump_in,                // Jump instruction flag
    input logic [1:0] ALUop_in,         // ALU operation type
    input logic [3:0] ALUControl_in,    // ALU control signal
    
    // Data inputs from ID stage
    input logic [31:0] PC_in,           // PC+4 value
    input logic [31:0] reg_data1_in,    // Register file output 1 (rs1)
    input logic [31:0] reg_data2_in,    // Register file output 2 (rs2)
    input logic [31:0] imm_in,          // Sign-extended immediate
    input logic [4:0] rs1_in,           // Source register 1 address
    input logic [4:0] rs2_in,           // Source register 2 address
    input logic [4:0] rd_in,            // Destination register address
    input logic [2:0] funct3_in,        // Function field for branch conditions
    
    // Control signal outputs to EX stage
    output logic RegWrite_out,          // Register write enable
    output logic MemtoReg_out,          // Memory to register mux control
    output logic MemRead_out,           // Memory read enable
    output logic MemWrite_out,          // Memory write enable
    output logic ALUSrc_out,            // ALU source mux control
    output logic Branch_out,            // Branch instruction flag
    output logic Jump_out,              // Jump instruction flag
    output logic [1:0] ALUop_out,       // ALU operation type
    output logic [3:0] ALUControl_out,  // ALU control signal
    
    // Data outputs to EX stage
    output logic [31:0] PC_out,         // PC+4 value
    output logic [31:0] reg_data1_out,  // Register file output 1
    output logic [31:0] reg_data2_out,  // Register file output 2
    output logic [31:0] imm_out,        // Sign-extended immediate
    output logic [4:0] rs1_out,         // Source register 1 address
    output logic [4:0] rs2_out,         // Source register 2 address
    output logic [4:0] rd_out,          // Destination register address
    output logic [2:0] funct3_out       // Function field for branch conditions
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset || flush) begin
            // Reset or flush: Clear all control signals (creates pipeline bubble)
            RegWrite_out <= 0; MemtoReg_out <= 0; MemRead_out <= 0; 
            MemWrite_out <= 0; ALUSrc_out <= 0; Branch_out <= 0; Jump_out <= 0;
            ALUop_out <= 2'b00; ALUControl_out <= 4'b0000;
            
            // Clear data signals
            PC_out <= 32'b0; reg_data1_out <= 32'b0; reg_data2_out <= 32'b0;
            imm_out <= 32'b0; rs1_out <= 5'b0; rs2_out <= 5'b0; rd_out <= 5'b0;
            funct3_out <= 3'b000;
        end else begin
            // Normal operation: Pass all signals through
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

//==============================================================================
// EX/MEM Pipeline Register
//
// Stores ALU results, memory control signals, and branch/jump information
// between the Execute and Memory Access stages.
//==============================================================================

module EX_MEM_Register (
    input logic clk,                    // Clock signal
    input logic reset,                  // Reset signal (active high)
    
    // Control signal inputs from EX stage
    input logic RegWrite_in,            // Register write enable
    input logic MemtoReg_in,            // Memory to register mux control
    input logic MemRead_in,             // Memory read enable
    input logic MemWrite_in,            // Memory write enable
    input logic Jump_in,                // Jump instruction flag
    input logic branch_taken_in,        // Branch taken decision
    
    // Data inputs from EX stage
    input logic [31:0] ALU_result_in,   // ALU computation result
    input logic [31:0] reg_data2_in,    // Register data for store operations
    input logic [31:0] branch_target_in,// Branch/jump target address
    input logic [4:0] rd_in,            // Destination register address
    
    // Control signal outputs to MEM stage
    output logic RegWrite_out,          // Register write enable
    output logic MemtoReg_out,          // Memory to register mux control
    output logic MemRead_out,           // Memory read enable
    output logic MemWrite_out,          // Memory write enable
    output logic Jump_out,              // Jump instruction flag
    output logic branch_taken_out,      // Branch taken decision
    
    // Data outputs to MEM stage
    output logic [31:0] ALU_result_out, // ALU computation result
    output logic [31:0] reg_data2_out,  // Register data for store operations
    output logic [31:0] branch_target_out,// Branch/jump target address
    output logic [4:0] rd_out           // Destination register address
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            // Reset: Clear all signals
            RegWrite_out <= 0; MemtoReg_out <= 0; MemRead_out <= 0;
            MemWrite_out <= 0; Jump_out <= 0; branch_taken_out <= 0;
            
            ALU_result_out <= 32'b0; reg_data2_out <= 32'b0; 
            branch_target_out <= 32'b0; rd_out <= 5'b0;
        end else begin
            // Normal operation: Pass all signals through
            RegWrite_out <= RegWrite_in; MemtoReg_out <= MemtoReg_in;
            MemRead_out <= MemRead_in; MemWrite_out <= MemWrite_in;
            Jump_out <= Jump_in; branch_taken_out <= branch_taken_in;
            
            ALU_result_out <= ALU_result_in; reg_data2_out <= reg_data2_in;
            branch_target_out <= branch_target_in; rd_out <= rd_in;
        end
    end
endmodule

//==============================================================================
// MEM/WB Pipeline Register
//
// Stores memory read data and ALU results between the Memory Access and
// Write Back stages. This is the final pipeline register.
//==============================================================================

module MEM_WB_Register (
    input logic clk,                    // Clock signal
    input logic reset,                  // Reset signal (active high)
    
    // Control signal inputs from MEM stage
    input logic RegWrite_in,            // Register write enable
    input logic MemtoReg_in,            // Memory to register mux control
    
    // Data inputs from MEM stage
    input logic [31:0] ALU_result_in,   // ALU result from EX stage
    input logic [31:0] memory_data_in,  // Data read from memory
    input logic [4:0] rd_in,            // Destination register address
    
    // Control signal outputs to WB stage
    output logic RegWrite_out,          // Register write enable
    output logic MemtoReg_out,          // Memory to register mux control
    
    // Data outputs to WB stage
    output logic [31:0] ALU_result_out, // ALU result
    output logic [31:0] memory_data_out,// Data read from memory
    output logic [4:0] rd_out           // Destination register address
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            // Reset: Clear all signals
            RegWrite_out <= 0; MemtoReg_out <= 0;
            ALU_result_out <= 32'b0; memory_data_out <= 32'b0; rd_out <= 5'b0;
        end else begin
            // Normal operation: Pass all signals through
            RegWrite_out <= RegWrite_in; MemtoReg_out <= MemtoReg_in;
            ALU_result_out <= ALU_result_in; memory_data_out <= memory_data_in;
            rd_out <= rd_in;
        end
    end
endmodule