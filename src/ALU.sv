/*
 * Arithmetic Logic Unit (ALU)
 * 
 * This is the main computational unit of the processor that performs arithmetic
 * and logical operations. It supports all RISC-V integer operations including
 * addition, subtraction, logical operations, shifts, and comparisons.
 * 
 * Inputs:
 *   - A[31:0]: First operand (typically from register rs1)
 *   - B[31:0]: Second operand (from register rs2 or immediate value)
 *   - ALUControl[3:0]: Operation control signal from ALU Control unit
 * 
 * Outputs:
 *   - ALUResult[31:0]: Result of the ALU operation
 *   - Zero: Flag indicating if ALUResult is zero (used for branch conditions)
 * 
 * Supported Operations:
 *   - 0000: AND    - 0001: OR     - 0010: ADD    - 0110: SUB
 *   - 0111: SLT    - 1000: SLTU   - 1001: XOR    - 1010: SLL
 *   - 1011: SRL    - 1100: SRA    - 1101: NOR
 */

module ALU (
   input logic [31:0] A, B,          // Input operands
   input logic [3:0] ALUControl,     // Operation control signal
   output logic [31:0] ALUResult,    // Result output
   output logic Zero                 // Zero flag for branches
);
   always_comb begin
      case(ALUControl)
         // Logical Operations
         4'b0000: ALUResult = A & B;                              // AND: Bitwise AND
         4'b0001: ALUResult = A | B;                              // OR: Bitwise OR
         4'b1001: ALUResult = A ^ B;                              // XOR: Bitwise XOR
         4'b1101: ALUResult = ~(A | B);                           // NOR: Bitwise NOR
         
         // Arithmetic Operations
         4'b0010: ALUResult = A + B;                              // ADD: Addition
         4'b0110: ALUResult = A - B;                              // SUB: Subtraction
         
         // Comparison Operations
         4'b0111: ALUResult = ($signed(A) < $signed(B)) ? 32'h1 : 32'h0;  // SLT: Set if A < B (signed)
         4'b1000: ALUResult = (A < B) ? 32'h1 : 32'h0;            // SLTU: Set if A < B (unsigned)
         
         // Shift Operations
         4'b1010: ALUResult = A << B[4:0];                        // SLL: Shift Left Logical
         4'b1011: ALUResult = A >> B[4:0];                        // SRL: Shift Right Logical
         4'b1100: ALUResult = $signed(A) >>> B[4:0];              // SRA: Shift Right Arithmetic
         
         // Default case
         default: ALUResult = 32'b0;                              // Output zero for undefined operations
      endcase
   end
   
   // Zero flag: Set when ALU result is zero (used for BEQ/BNE branches)
   assign Zero = (ALUResult == 32'b0);
endmodule