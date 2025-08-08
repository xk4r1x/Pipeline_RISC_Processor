/*
 * Immediate Generator
 * 
 * This module extracts and sign-extends immediate values from RISC-V instructions.
 * Different instruction types have different immediate formats, so this module
 * decodes the ImmSrc control signal to select the appropriate immediate format.
 * 
 * Inputs:
 *   - instruction[31:0]: Complete 32-bit instruction
 *   - ImmSrc[2:0]: Immediate source type selector from Control Unit
 * 
 * Output:
 *   - immediate[31:0]: 32-bit sign-extended immediate value
 * 
 * RISC-V Immediate Formats:
 *   - I-type: 12-bit immediate in bits [31:20]
 *   - S-type: 12-bit immediate split across bits [31:25] and [11:7]  
 *   - B-type: 13-bit immediate for branches (with implicit bit 0 = 0)
 *   - U-type: 20-bit immediate in upper bits [31:12]
 *   - J-type: 21-bit immediate for jumps (with implicit bit 0 = 0)
 */

module Immediate_Generator (
   input logic [31:0] instruction,  // Complete instruction
   input logic [2:0] ImmSrc,        // Immediate type selector
   output logic [31:0] immediate    // Sign-extended immediate output
);
   always_comb begin
      case (ImmSrc)
         // I-type immediate: ADDI, ANDI, ORI, XORI, loads, JALR
         // Format: [31:20] = immediate, sign-extended to 32 bits
         3'b001: immediate = {{20{instruction[31]}}, instruction[31:20]};
         
         // S-type immediate: Store instructions (SW, SH, SB)
         // Format: [31:25] = imm[11:5], [11:7] = imm[4:0], sign-extended
         3'b010: immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
         
         // B-type immediate: Branch instructions (BEQ, BNE, BLT, BGE)
         // Format: [31] = imm[12], [7] = imm[11], [30:25] = imm[10:5], [11:8] = imm[4:1]
         // Note: imm[0] is always 0 (instructions are 2-byte aligned), so we append 1'b0
         3'b011: immediate = {{19{instruction[31]}}, instruction[31], instruction[7], 
                              instruction[30:25], instruction[11:8], 1'b0};
         
         // J-type immediate: JAL instruction
         // Format: [31] = imm[20], [19:12] = imm[19:12], [20] = imm[11], [30:21] = imm[10:1]
         // Note: imm[0] is always 0 (instructions are 2-byte aligned), so we append 1'b0
         3'b100: immediate = {{11{instruction[31]}}, instruction[31], instruction[19:12], 
                              instruction[20], instruction[30:21], 1'b0};
         
         // U-type immediate: LUI, AUIPC
         // Format: [31:12] = immediate, [11:0] = 0 (immediate goes in upper 20 bits)
         3'b101: immediate = {instruction[31:12], 12'b0};
         
         // Default case: No immediate (output zero)
         default: immediate = 32'b0;
      endcase
   end
endmodule