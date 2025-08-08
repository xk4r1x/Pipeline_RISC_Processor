# RISC-V Instructions Supported

This document lists all RISC-V RV32I instructions supported by the 5-stage pipelined CPU.

## R-Type Instructions (Register-Register)

| Instruction | Format | Description | Example |
|-------------|--------|-------------|---------|
| ADD | ADD rd, rs1, rs2 | rd = rs1 + rs2 | ADD x1, x2, x3 |
| SUB | SUB rd, rs1, rs2 | rd = rs1 - rs2 | SUB x1, x2, x3 |
| AND | AND rd, rs1, rs2 | rd = rs1 & rs2 | AND x1, x2, x3 |
| OR | OR rd, rs1, rs2 | rd = rs1 \| rs2 | OR x1, x2, x3 |
| XOR | XOR rd, rs1, rs2 | rd = rs1 ^ rs2 | XOR x1, x2, x3 |
| SLL | SLL rd, rs1, rs2 | rd = rs1 << rs2 | SLL x1, x2, x3 |
| SRL | SRL rd, rs1, rs2 | rd = rs1 >> rs2 (logical) | SRL x1, x2, x3 |
| SRA | SRA rd, rs1, rs2 | rd = rs1 >> rs2 (arithmetic) | SRA x1, x2, x3 |
| SLT | SLT rd, rs1, rs2 | rd = (rs1 < rs2) ? 1 : 0 (signed) | SLT x1, x2, x3 |
| SLTU | SLTU rd, rs1, rs2 | rd = (rs1 < rs2) ? 1 : 0 (unsigned) | SLTU x1, x2, x3 |

## I-Type Instructions (Immediate)

| Instruction | Format | Description | Example |
|-------------|--------|-------------|---------|
| ADDI | ADDI rd, rs1, imm | rd = rs1 + imm | ADDI x1, x2, 100 |
| ANDI | ANDI rd, rs1, imm | rd = rs1 & imm | ANDI x1, x2, 0xFF |
| ORI | ORI rd, rs1, imm | rd = rs1 \| imm | ORI x1, x2, 0x10 |
| XORI | XORI rd, rs1, imm | rd = rs1 ^ imm | XORI x1, x2, 0xF0 |
| SLLI | SLLI rd, rs1, imm | rd = rs1 << imm | SLLI x1, x2, 4 |
| SRLI | SRLI rd, rs1, imm | rd = rs1 >> imm (logical) | SRLI x1, x2, 2 |
| SRAI | SRAI rd, rs1, imm | rd = rs1 >> imm (arithmetic) | SRAI x1, x2, 3 |
| SLTI | SLTI rd, rs1, imm | rd = (rs1 < imm) ? 1 : 0 (signed) | SLTI x1, x2, 50 |
| SLTIU | SLTIU rd, rs1, imm | rd = (rs1 < imm) ? 1 : 0 (unsigned) | SLTIU x1, x2, 50 |

## Load Instructions

| Instruction | Format | Description | Example |
|-------------|--------|-------------|---------|
| LW | LW rd, offset(rs1) | rd = MEM[rs1 + offset] | LW x1, 4(x2) |

## Store Instructions

| Instruction | Format | Description | Example |
|-------------|--------|-------------|---------|
| SW | SW rs2, offset(rs1) | MEM[rs1 + offset] = rs2 | SW x1, 8(x2) |

## Branch Instructions

| Instruction | Format | Description | Example |
|-------------|--------|-------------|---------|
| BEQ | BEQ rs1, rs2, offset | if (rs1 == rs2) PC += offset | BEQ x1, x2, loop |
| BNE | BNE rs1, rs2, offset | if (rs1 != rs2) PC += offset | BNE x1, x2, end |
| BLT | BLT rs1, rs2, offset | if (rs1 < rs2) PC += offset (signed) | BLT x1, x2, less |
| BGE | BGE rs1, rs2, offset | if (rs1 >= rs2) PC += offset (signed) | BGE x1, x2, greater |
| BLTU | BLTU rs1, rs2, offset | if (rs1 < rs2) PC += offset (unsigned) | BLTU x1, x2, less |
| BGEU | BGEU rs1, rs2, offset | if (rs1 >= rs2) PC += offset (unsigned) | BGEU x1, x2, greater |

## Jump Instructions

| Instruction | Format | Description | Example |
|-------------|--------|-------------|---------|
| JAL | JAL rd, offset | rd = PC + 4; PC += offset | JAL x1, function |
| JALR | JALR rd, rs1, offset | rd = PC + 4; PC = rs1 + offset | JALR x1, x2, 0 |

## Upper Immediate Instructions

| Instruction | Format | Description | Example |
|-------------|--------|-------------|---------|
| LUI | LUI rd, imm | rd = imm << 12 | LUI x1, 0x12345 |
| AUIPC | AUIPC rd, imm | rd = PC + (imm << 12) | AUIPC x1, 0x1000 |

## Instruction Encoding

All instructions follow standard RISC-V 32-bit encoding:

### R-Type Format
```
31    25 24  20 19  15 14  12 11   7 6     0
funct7   rs2   rs1   funct3  rd    opcode
```

### I-Type Format
```
31          20 19  15 14  12 11   7 6     0
imm[11:0]     rs1   funct3  rd    opcode
```

### S-Type Format
```
31    25 24  20 19  15 14  12 11   7 6     0
imm[11:5] rs2   rs1   funct3 imm[4:0] opcode
```

### B-Type Format
```
31 30    25 24  20 19  15 14  12 11    8 7 6     0
imm[12] imm[10:5] rs2   rs1   funct3 imm[4:1] imm[11] opcode
```

### U-Type Format
```
31          12 11   7 6     0
imm[31:12]     rd    opcode
```

### J-Type Format
```
31 30      21 20 19    12 11   7 6     0
imm[20] imm[10:1] imm[11] imm[19:12] rd    opcode
```

## Notes

- All immediate values are sign-extended to 32 bits
- Register x0 is hardwired to zero and cannot be written
- All instructions are 32 bits (4 bytes) and must be word-aligned
- PC is incremented by 4 for sequential instructions
- Branch and jump offsets are relative to the current PC