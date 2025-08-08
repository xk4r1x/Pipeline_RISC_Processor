/*
 * ALU Control Unit
 * 
 * This module generates the 4-bit ALU control signal based on the instruction type
 * and function fields. It decodes the ALUop signal from the main control unit
 * and combines it with funct3 and funct7 fields to determine the exact ALU operation.
 * 
 * Inputs:
 *   - funct3[2:0]: Function field from instruction bits [14:12]
 *   - funct7[6:0]: Function field from instruction bits [31:25] 
 *   - ALUop[1:0]: Control signal from main control unit
 *     - 00: Load/Store (always ADD for address calculation)
 *     - 01: Branch (always SUB for comparison)
 *     - 10: R-type arithmetic (decode using funct3/funct7)
 *     - 11: I-type arithmetic (decode using funct3/funct7)
 * 
 * Output:
 *   - ALUControl[3:0]: 4-bit control signal for ALU operations
 *     - 0000: AND    - 0001: OR     - 0010: ADD    - 0110: SUB
 *     - 0111: SLT    - 1000: SLTU   - 1001: XOR    - 1010: SLL
 *     - 1011: SRL    - 1100: SRA    - 1101: NOR
 */

module ALU_Control (
   input logic [2:0] funct3,    // Function field 3 from instruction
   input logic [6:0] funct7,    // Function field 7 from instruction
   input logic [1:0] ALUop,     // ALU operation type from control unit
   output logic [3:0] ALUControl // ALU control signal output
);
   always_comb begin
      case (ALUop)
         // Load/Store instructions: Always perform addition for address calculation
         2'b00: ALUControl = 4'b0010; // ADD
         
         // Branch instructions: Always perform subtraction for comparison
         2'b01: ALUControl = 4'b0110; // SUB
         
         // R-type instructions: Decode based on funct3 and funct7 fields
         2'b10: begin
            case(funct3)
               3'b000: ALUControl = (funct7[5]) ? 4'b0110 : 4'b0010; // SUB if funct7[5]=1, ADD if funct7[5]=0
               3'b001: ALUControl = 4'b1010; // SLL (Shift Left Logical)
               3'b010: ALUControl = 4'b0111; // SLT (Set Less Than)
               3'b011: ALUControl = 4'b1000; // SLTU (Set Less Than Unsigned)
               3'b100: ALUControl = 4'b1001; // XOR
               3'b101: ALUControl = (funct7[5]) ? 4'b1100 : 4'b1011; // SRA if funct7[5]=1, SRL if funct7[5]=0
               3'b110: ALUControl = 4'b0001; // OR
               3'b111: ALUControl = 4'b0000; // AND
               default: ALUControl = 4'b0010; // Default to ADD
            endcase
         end
         
         // I-type arithmetic instructions: Decode based on funct3 and funct7 fields
         2'b11: begin
            case(funct3)
               3'b000: ALUControl = 4'b0010; // ADDI (ADD Immediate)
               3'b001: ALUControl = 4'b1010; // SLLI (Shift Left Logical Immediate)
               3'b010: ALUControl = 4'b0111; // SLTI (Set Less Than Immediate)
               3'b011: ALUControl = 4'b1000; // SLTIU (Set Less Than Immediate Unsigned)
               3'b100: ALUControl = 4'b1001; // XORI (XOR Immediate)
               3'b101: ALUControl = (funct7[5]) ? 4'b1100 : 4'b1011; // SRAI if funct7[5]=1, SRLI if funct7[5]=0
               3'b110: ALUControl = 4'b0001; // ORI (OR Immediate)
               3'b111: ALUControl = 4'b0000; // ANDI (AND Immediate)
               default: ALUControl = 4'b0010; // Default to ADD
            endcase
         end
         
         // Default case: Perform addition
         default: ALUControl = 4'b0010; // ADD
      endcase
   end
endmodule