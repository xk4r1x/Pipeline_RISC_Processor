`timescale 1ns / 1ps

module CPU_tb;
    logic clk;
    logic reset;

    // Instantiate the CPU
    CPU uut (
        .clk(clk),
        .reset(reset)
    );

    // Clock generation: 10 ns period (100 MHz)
    always #5 clk = ~clk;

    // Waveform dumping - ADD THIS!
    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, CPU_tb);
    end

    // Simulation procedure
    initial begin
        clk = 0;
        reset = 1;
        
        $display("=== RISC-V Pipelined CPU Simulation Start ===");
        
        // Apply reset
        #10;
        reset = 0;
        
        // Run for 30 cycles
        repeat (30) begin
            @(posedge clk);
            #1; // Allow signals to settle
            
            $display("Time=%0t | PC=%0d | Instr=%h | x1=%0d | x2=%0d | x3=%0d", 
                $time, uut.PC, uut.IF_instruction, 
                uut.register_file.registers[1], 
                uut.register_file.registers[2], 
                uut.register_file.registers[3]);
        end
        
        $display("=== Final Register Values ===");
        for (int i = 1; i < 8; i++) begin
            $display("x%0d = %0d", i, uut.register_file.registers[i]);
        end
        
        $finish;
    end
endmodule