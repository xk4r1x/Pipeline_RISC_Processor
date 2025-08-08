/*
 * Data Memory Module
 * 
 * This module implements the data memory subsystem for load and store operations.
 * It provides a word-addressable memory with 1024 32-bit words (4KB total).
 * The memory supports both read and write operations controlled by enable signals.
 * 
 * Inputs:
 *   - clk: Clock signal for synchronous writes
 *   - MemWrite: Enable signal for memory write operations
 *   - MemRead: Enable signal for memory read operations  
 *   - address[31:0]: Byte address for memory access
 *   - writeData[31:0]: Data to write to memory
 * 
 * Output:
 *   - readData[31:0]: Data read from memory
 * 
 * Memory Organization:
 *   - 1024 words × 32 bits = 4096 bytes (4KB)
 *   - Word-aligned access (address bits [1:0] ignored)
 *   - Address range: 0x00000000 to 0x00000FFC (word addresses)
 * 
 * Operation:
 *   - Writes: Synchronous on positive clock edge when MemWrite=1
 *   - Reads: Combinational when MemRead=1, zero when MemRead=0
 *   - Address bounds checking prevents out-of-range access
 */

module Data_Memory (
   input logic clk,                 // Clock for synchronous writes
   input logic MemWrite,            // Memory write enable
   input logic MemRead,             // Memory read enable
   input logic [31:0] address,      // Byte address (word-aligned)
   input logic [31:0] writeData,    // Data to write
   output logic [31:0] readData     // Data read from memory
);
   
   // Memory array: 1024 words of 32 bits each
   logic [31:0] mem [0:1023];
   
   // Convert byte address to word address by right-shifting by 2
   // This effectively divides by 4 since each word is 4 bytes
   logic [31:0] word_addr;
   assign word_addr = address >> 2;

   // Initialize memory with some test data
   initial begin
      // Initialize all memory locations to zero
      for (int i = 0; i < 1024; i++) begin
         mem[i] = 32'h00000000;
      end
      
      // Add some test data for verification
      mem[0] = 32'h12345678;  // Test data at word address 0
      mem[1] = 32'hABCDEF00;  // Test data at word address 1  
      mem[2] = 32'h11111111;  // Test data at word address 2
   end

   // Synchronous write operation
   // Writes occur on positive clock edge when MemWrite is asserted
   always_ff @(posedge clk) begin
      if (MemWrite && word_addr < 1024) begin
         mem[word_addr] <= writeData;
      end
   end

   // Combinational read operation  
   // Reads are immediate when MemRead is asserted
   always_comb begin
      if (MemRead && word_addr < 1024) begin
         readData = mem[word_addr];      // Return data from memory
      end else begin
         readData = 32'b0;               // Return zero if read disabled or out of bounds
      end
   end
endmodule