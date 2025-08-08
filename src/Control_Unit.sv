/*
 * Main Control Unit
 * 
 * This module is the central control logic that decodes instruction opcodes
 * and generates all necessary control signals for the datapath. It determines
 * how each instruction should be executed by setting appropriate control signals.
 * 
 * Inputs:
 *   - opcode[6:0]: Instruction opcode (bits [6:0] of instruction)
 *   - funct3[2:0]: Function field 3 (bits [14:12] of instruction)
 * 
 * Outputs:
 *   - RegWrite: Enable writing to register file
 *   - MemtoReg: Select memory data for register write (0=ALU, 1=Memory)
 *   - MemRead: Enable memory read operation
 *   - MemWrite: Enable memory write operation  
 *   - ALUSrc: Select ALU second input (0=register, 1=immediate)
 *   - Branch: Indicates branch instruction
 *   - Jump: Indicates jump instruction
 *   - ALUop[1:0]: Operation type for ALU Control unit
 *   - ImmSrc[2:0]: Immediate type for Immediate Generator
 * 
 * RISC-V Instruction Types Supported:
 *   - R-type: Register-register operations (ADD, SUB, AND, OR, etc.)
 *   - I-type: Immediate operations (ADDI, ANDI, loads, JALR)
 *   - S-type: Store operations (SW, SH, SB)
 *   - B-type: Branch operations (BEQ, BNE, BLT, BGE)
 *   - U-type: Upper immediate (LUI, AUIPC)
 *   - J-type: Jump operations (JAL)
 */

module Control_Unit (
   input logic [6:0] opcode,    // Instruction opcode
   input logic [2:0] funct3,    // Function field 3 (for branch disambiguation)
   
   // Register and Memory Control
   output logic RegWrite,       // Enable register file write
   output logic MemtoReg,       // Select memory data for writeback
   output logic MemRead,        // Enable memory read
   output logic MemWrite,       // Enable memory write
   
   // ALU Control  
   output logic ALUSrc,         // Select ALU input B source
   output logic [1:0] ALUop,    // ALU operation type
   
   // Control Flow
   output logic Branch,         // Branch instruction flag
   output logic Jump,           // Jump instruction flag
   
   // Immediate Control
   output logic [2:0] ImmSrc    // Immediate type selector
);
   
   always_comb begin
      // Default control signal values (NOP behavior)
      RegWrite = 0; MemtoReg = 0; MemRead = 0; MemWrite = 0;
      ALUSrc = 0; Branch = 0; Jump = 0; ALUop = 2'b00; ImmSrc = 3'b000;
      
      case(opcode)
         // R-type instructions: ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT, SLTU
         7'b0110011: begin 
            RegWrite = 1;        // Write ALU result to register
            ALUop = 2'b10;       // R-type ALU operations
         end
         
         // I-type arithmetic: ADDI, ANDI, ORI, XORI, SLLI, SRLI, SRAI, SLTI, SLTIU  
         7'b0010011: begin 
            RegWrite = 1;        // Write ALU result to register
            ALUop = 2'b11;       // I-type ALU operations
            ALUSrc = 1;          // Use immediate as ALU input B
            ImmSrc = 3'b001;     // I-type immediate format
         end
         
         // Load instructions: LW, LH, LB, LHU, LBU
         7'b0000011: begin 
            RegWrite = 1;        // Write loaded data to register
            MemRead = 1;         // Read from memory
            MemtoReg = 1;        // Select memory data for writeback
            ALUSrc = 1;          // Use immediate for address calculation
            ImmSrc = 3'b001;     // I-type immediate format
         end
         
         // Store instructions: SW, SH, SB
         7'b0100011: begin 
            MemWrite = 1;        // Write to memory
            ALUSrc = 1;          // Use immediate for address calculation
            ImmSrc = 3'b010;     // S-type immediate format
         end
         
         // Branch instructions: BEQ, BNE, BLT, BGE, BLTU, BGEU
         7'b1100011: begin 
            Branch = 1;          // Branch instruction flag
            ALUop = 2'b01;       // Subtraction for comparison
            ImmSrc = 3'b011;     // B-type immediate format
         end
         
         // JAL: Jump and Link
         7'b1101111: begin 
            RegWrite = 1;        // Write return address (PC+4) to register
            Jump = 1;            // Jump instruction flag
            ImmSrc = 3'b100;     // J-type immediate format
         end
         
         // JALR: Jump and Link Register
         7'b1100111: begin 
            RegWrite = 1;        // Write return address (PC+4) to register
            Jump = 1;            // Jump instruction flag
            ALUSrc = 1;          // Use immediate for target calculation
            ImmSrc = 3'b001;     // I-type immediate format
         end
         
         // LUI (Load Upper Immediate) and AUIPC (Add Upper Immediate to PC)
         7'b0110111, 7'b0010111: begin 
            RegWrite = 1;        // Write result to register
            ALUSrc = 1;          // Use immediate as ALU input
            ImmSrc = 3'b101;     // U-type immediate format
         end
         
         // Default: NOP (No Operation) - all control signals remain 0
         default: ; 
      endcase
   end
endmodule