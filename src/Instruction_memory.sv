/*
 * Instruction Memory Module
 * 
 * This module implements the instruction memory subsystem that stores the program.
 * It provides read-only access to 256 32-bit instruction words (1KB total).
 * The memory is initialized with a test program demonstrating various RISC-V instructions.
 * 
 * Input:
 *   - address[31:0]: Byte address for instruction fetch (word-aligned)
 * 
 * Output:  
 *   - instruction[31:0]: 32-bit instruction at the specified address
 * 
 * Memory Organization:
 *   - 256 words × 32 bits = 1024 bytes (1KB)
 *   - Word-aligned access (address bits [1:0] ignored)
 *   - Address range: 0x00000000 to 0x000003FC (word addresses)
 * 
 * Test Program:
 *   The memory is preloaded with a test program that exercises:
 *   - Arithmetic operations (ADD, SUB, AND, OR)
 *   - Immediate operations (ADDI)
 *   - Memory operations (SW, LW)
 *   - Control flow (BEQ branches)
 *   - Loop constructs
 */

module Instruction_Memory (
   input logic [31:0] address,      // Byte address for instruction fetch
   output logic [31:0] instruction  // Fetched instruction
);
   
   // Instruction memory array: 256 words of 32 bits each
   logic [31:0] mem [0:255];
   
   // Convert byte address to word address
   // Extract bits [9:2] to get word index (ignoring byte offset [1:0])
   logic [7:0] word_addr;
   assign word_addr = address[9:2];

   // Initialize instruction memory with test program
   initial begin
      // Fill all locations with NOP instructions initially
      for (int i = 0; i < 256; i++) 
         mem[i] = 32'h00000013; // NOP (ADDI x0, x0, 0)

      // Test program demonstrating various instruction types
      
      // Basic arithmetic and immediate operations
      mem[0] = 32'h00100093;   // ADDI x1, x0, 1      : x1 = 1
      mem[1] = 32'h00200113;   // ADDI x2, x0, 2      : x2 = 2
      mem[2] = 32'h002081B3;   // ADD  x3, x1, x2     : x3 = x1 + x2 = 3
      mem[3] = 32'h40208233;   // SUB  x4, x1, x2     : x4 = x1 - x2 = -1
      mem[4] = 32'h0020F2B3;   // AND  x5, x1, x2     : x5 = x1 & x2 = 0
      mem[5] = 32'h0020E333;   // OR   x6, x1, x2     : x6 = x1 | x2 = 3
      
      // Memory operations  
      mem[6] = 32'h00012623;   // SW   x0, 12(x2)     : Store 0 at address (x2 + 12)
      mem[7] = 32'h00C12383;   // LW   x7, 12(x2)     : Load from address (x2 + 12) into x7
      
      // Control flow - conditional branch
      mem[8] = 32'h00118463;   // BEQ  x3, x1, +8     : Branch to PC+8 if x3 == x1 (skip next instruction)
      mem[9] = 32'h00100393;   // ADDI x7, x0, 1      : x7 = 1 (executed if branch not taken)
      
      // More instructions for loop demonstration
      mem[10] = 32'h00000013;  // NOP                  : No operation
      mem[11] = 32'h00100493;  // ADDI x9, x0, 1      : x9 = 1
      mem[12] = 32'hFE1482E3;  // BEQ  x9, x1, -12    : Branch back to mem[6] if x9 == x1 (creates loop)
   end

   // Combinational read operation
   // Return instruction if address is within bounds, otherwise return NOP
   always_comb begin
      instruction = (word_addr < 256) ? mem[word_addr] : 32'h00000013;
   end
endmodule