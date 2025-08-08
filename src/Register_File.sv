/*
 * Register File Module
 * 
 * This module implements the 32-register register file required by the RISC-V architecture.
 * It provides dual-port read access and single-port write access. Register x0 is hardwired
 * to zero and cannot be written to, as per RISC-V specification.
 * 
 * Inputs:
 *   - clk: Clock signal for synchronous writes
 *   - RegWrite: Enable signal for register write operations
 *   - readReg1[4:0]: Address of first register to read (rs1)
 *   - readReg2[4:0]: Address of second register to read (rs2)  
 *   - writeReg[4:0]: Address of register to write (rd)
 *   - writeData[31:0]: Data to write to the register
 * 
 * Outputs:
 *   - readData1[31:0]: Data from first read port
 *   - readData2[31:0]: Data from second read port
 * 
 * Register File Organization:
 *   - 32 registers (x0 through x31)
 *   - Each register is 32 bits wide
 *   - x0 is always zero (hardwired constant)
 *   - x1-x31 are general-purpose registers
 * 
 * Operation:
 *   - Reads: Combinational (immediate) from both ports
 *   - Writes: Synchronous on positive clock edge when RegWrite=1
 *   - x0 cannot be written (writes to x0 are ignored)
 */

module Register_File (
   input logic clk,                    // Clock for synchronous writes
   input logic RegWrite,               // Register write enable
   input logic [4:0] readReg1,         // First read register address (rs1)
   input logic [4:0] readReg2,         // Second read register address (rs2)
   input logic [4:0] writeReg,         // Write register address (rd)
   input logic [31:0] writeData,       // Data to write
   output logic [31:0] readData1,      // First read port output
   output logic [31:0] readData2       // Second read port output
);
   
   // Register array: 32 registers of 32 bits each
   logic [31:0] registers [0:31];

   // Initialize all registers to zero
   initial begin
      for (int i = 0; i < 32; i++) begin
         registers[i] = 32'h00000000;
      end
   end
   
   // Combinational read operations (dual-port)
   // Both read ports operate simultaneously and independently
   always_comb begin
      // First read port: Return zero if reading x0, otherwise return register content
      readData1 = (readReg1 == 5'd0) ? 32'h00000000 : registers[readReg1];
      
      // Second read port: Return zero if reading x0, otherwise return register content  
      readData2 = (readReg2 == 5'd0) ? 32'h00000000 : registers[readReg2];
   end
   
   // Synchronous write operation
   // Writes occur on positive clock edge when RegWrite is asserted
   // Note: Writes to register x0 are ignored (x0 is always zero)
   always_ff @(posedge clk) begin
      if (RegWrite && writeReg != 5'd0) begin
         registers[writeReg] <= writeData;
      end
   end
endmodule