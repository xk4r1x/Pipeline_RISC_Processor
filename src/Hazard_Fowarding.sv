/*
 * Hazard Detection and Forwarding Units for Pipelined RISC-V CPU
 * 
 * These modules handle the three main types of pipeline hazards:
 * 1. Data hazards: Solved by forwarding unit
 * 2. Load-use hazards: Solved by hazard detection unit (stalling)
 * 3. Control hazards: Solved by hazard detection unit (flushing)
 */

//==============================================================================
// Forwarding Unit
//
// This unit detects data hazards and generates forwarding control signals
// to bypass ALU results from later pipeline stages to earlier stages.
// This eliminates most data hazards without requiring stalls.
//
// Data Hazard Example:
//   ADD x1, x2, x3    # x1 = x2 + x3 (writes x1 in WB stage)
//   SUB x4, x1, x5    # Needs x1 value (reads x1 in EX stage)
//
// Without forwarding: Need 2 stall cycles
// With forwarding: No stalls needed - forward result from EX/MEM or MEM/WB
//==============================================================================

module Forwarding_Unit (
    // Pipeline register write enables
    input logic EX_MEM_RegWrite,        // EX/MEM stage will write to register
    input logic MEM_WB_RegWrite,        // MEM/WB stage will write to register
    
    // Register addresses
    input logic [4:0] EX_MEM_rd,        // Destination register in EX/MEM stage
    input logic [4:0] MEM_WB_rd,        // Destination register in MEM/WB stage
    input logic [4:0] ID_EX_rs1,        // Source register 1 in ID/EX stage
    input logic [4:0] ID_EX_rs2,        // Source register 2 in ID/EX stage
    
    // Forwarding control outputs
    output logic [1:0] ForwardA,        // Forwarding control for ALU input A
    output logic [1:0] ForwardB         // Forwarding control for ALU input B
);
    always_comb begin
        // Default: No forwarding (use register file data)
        ForwardA = 2'b00;
        ForwardB = 2'b00;
        
        // EX/MEM Forwarding (highest priority - most recent result)
        // Forward ALU result from EX/MEM stage if:
        // 1. EX/MEM stage will write to a register (RegWrite = 1)
        // 2. Destination register is not x0 (can't write to x0)
        // 3. Destination register matches source register in EX stage
        
        if (EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rs1))
            ForwardA = 2'b10;  // Forward EX/MEM ALU result to ALU input A
            
        if (EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rs2))
            ForwardB = 2'b10;  // Forward EX/MEM ALU result to ALU input B
            
        // MEM/WB Forwarding (lower priority - only if EX/MEM doesn't forward)
        // Forward writeback data from MEM/WB stage if:
        // 1. MEM/WB stage will write to a register (RegWrite = 1)
        // 2. Destination register is not x0
        // 3. Destination register matches source register in EX stage
        // 4. EX/MEM stage is NOT already forwarding to the same source register
        
        if (MEM_WB_RegWrite && (MEM_WB_rd != 0) && (MEM_WB_rd == ID_EX_rs1) && 
            !(EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rs1)))
            ForwardA = 2'b01;  // Forward MEM/WB result to ALU input A
            
        if (MEM_WB_RegWrite && (MEM_WB_rd != 0) && (MEM_WB_rd == ID_EX_rs2) &&
            !(EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rs2)))
            ForwardB = 2'b01;  // Forward MEM/WB result to ALU input B
    end
endmodule

//==============================================================================
// Hazard Detection Unit
//
// This unit detects hazards that cannot be resolved by forwarding and
// generates control signals to stall the pipeline or flush instructions.
//
// Load-Use Hazard Example:
//   LW  x1, 0(x2)     # Load data into x1 (data available in MEM stage)
//   ADD x3, x1, x4    # Needs x1 value immediately (in EX stage)
//
// Control Hazard Example:
//   BEQ x1, x2, label # Branch decision made in EX stage
//   ADD x3, x4, x5    # May need to be discarded if branch taken
//==============================================================================

module Hazard_Detection (
    // Load instruction detection
    input logic ID_EX_MemRead,          // ID/EX stage is a load instruction
    input logic [4:0] ID_EX_rd,         // Destination register of load instruction
    input logic [4:0] IF_ID_rs1,        // Source register 1 of instruction in ID stage
    input logic [4:0] IF_ID_rs2,        // Source register 2 of instruction in ID stage
    
    // Control hazard detection
    input logic Branch,                  // Current instruction in ID stage is a branch
    input logic Jump,                    // Current instruction in ID stage is a jump
    input logic EX_MEM_branch_taken,     // Branch in EX/MEM stage was taken
    input logic EX_MEM_Jump,            // Jump instruction in EX/MEM stage
    
    // Hazard control outputs
    output logic Stall,                  // Stall IF and ID stages
    output logic IF_ID_flush,            // Flush IF/ID pipeline register
    output logic ID_EX_flush             // Flush ID/EX pipeline register
);
    always_comb begin
        // Load-Use Hazard Detection
        // A load-use hazard occurs when:
        // 1. Instruction in ID/EX stage is a load (MemRead = 1)
        // 2. Load destination register is not x0
        // 3. Load destination register matches a source register of instruction in ID stage
        //
        // Solution: Stall IF and ID stages, insert bubble in EX stage
        
        if (ID_EX_MemRead && ID_EX_rd != 0 && 
            ((ID_EX_rd == IF_ID_rs1) || (ID_EX_rd == IF_ID_rs2))) begin
            Stall = 1;          // Stall PC and IF/ID register
            IF_ID_flush = 0;    // Keep instruction in IF/ID (don't flush)
            ID_EX_flush = 1;    // Insert bubble in ID/EX (flush to create NOP)
        end
        
        // Control Hazard Detection  
        // A control hazard occurs when a branch or jump changes the PC
        // Instructions that were fetched after the branch/jump need to be discarded
        //
        // Solution: Flush instructions in IF/ID and ID/EX stages
        
        else if (EX_MEM_branch_taken || EX_MEM_Jump) begin
            Stall = 0;          // Don't stall (continue fetching from new PC)
            IF_ID_flush = 1;    // Flush instruction in IF/ID stage
            ID_EX_flush = 1;    // Flush instruction in ID/EX stage
        end
        
        // No Hazard Detected
        else begin
            Stall = 0;          // No stall needed
            IF_ID_flush = 0;    // No flush needed
            ID_EX_flush = 0;    // No flush needed
        end
    end
endmodule