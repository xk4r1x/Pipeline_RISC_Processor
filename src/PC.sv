/*
 * Program Counter (PC) Module
 * 
 * This module implements the program counter that tracks the address of the
 * currently executing instruction. The PC is updated on each clock cycle
 * unless stalled by the pipeline control logic.
 * 
 * Inputs:
 *   - clk: Clock signal for PC updates
 *   - reset: Asynchronous reset signal (active high)
 *   - next_pc[31:0]: Next PC value (from branch/jump logic or PC+4)
 * 
 * Output:
 *   - pc[31:0]: Current program counter value
 * 
 * Operation:
 *   - Reset: PC is set to 0x00000000 (start of instruction memory)
 *   - Normal: PC is updated to next_pc on each positive clock edge
 *   - Word-aligned: PC is always word-aligned (bits [1:0] forced to 00)
 * 
 * Note: In a pipelined implementation, this module may be controlled by
 * stall signals from the hazard detection unit to freeze the PC during
 * pipeline stalls.
 */

module PC (
   input logic clk,                 // Clock signal
   input logic reset,               // Asynchronous reset (active high)
   input logic [31:0] next_pc,      // Next PC value
   output logic [31:0] pc           // Current PC value
);
   
   // Synchronous PC update with asynchronous reset
   always_ff @(posedge clk or posedge reset) begin
      if (reset) begin
         // Reset PC to start of instruction memory
         pc <= 32'h00000000;
      end else begin
         // Update PC to next value, ensuring word alignment
         // Force bits [1:0] to 00 since instructions are 4-byte aligned
         pc <= {next_pc[31:2], 2'b00};
      end
   end
endmodule