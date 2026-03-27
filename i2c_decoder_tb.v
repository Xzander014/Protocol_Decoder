// =============================================================================
// Testbench : i2c_decoder_tb
// Purpose   : Verifies the i2c_decoder module by driving real I2C sequences:
//               Test 1 - WRITE transaction (addr=0x4A, data=0xA5, 0x3C)
//               Test 2 - READ  transaction (addr=0x4A)
//               Test 3 - Repeated START
//               Test 4 - NACK from slave (error flag expected)
// =============================================================================

`timescale 1ns/1ps

module i2c_decoder_tb;

    // -----------------------------------------------------------------------
    // Parameters
    // -----------------------------------------------------------------------
    // System clock period (100 MHz → 10 ns)
    localparam CLK_PERIOD  = 10;

    // I2C timing in system-clock cycles
    // Standard-mode I2C = 100 kHz → period = 10 µs = 1000 sys clocks
    // Using a much shorter cycle here to keep simulation quick.
    localparam I2C_HALF    = 50;   // Half I2C clock period (cycles)
    localparam I2C_SETUP   = 5;    // SDA setup time before SCL edge (cycles)

    // -----------------------------------------------------------------------
    // DUT signals
    // -----------------------------------------------------------------------
    reg        clk;
    reg        rst_n;
    reg        scl;
    reg        sda;

    wire [6:0] addr;
    wire       rw;
    wire [7:0] data_byte;
    wire       addr_valid;
    wire       data_valid;
    wire       ack;
    wire       busy;
    wire       error;

    // -----------------------------------------------------------------------
    // DUT instantiation
    // -----------------------------------------------------------------------
    i2c_decoder uut (
        .clk        (clk),
        .rst_n      (rst_n),
        .scl        (scl),
        .sda        (sda),
        .addr       (addr),
        .rw         (rw),
        .data_byte  (data_byte),
        .addr_valid (addr_valid),
        .data_valid (data_valid),
        .ack        (ack),
        .busy       (busy),
        .error      (error)
    );

    // -----------------------------------------------------------------------
    // Clock generation
    // -----------------------------------------------------------------------
    initial clk = 1'b0;
    always  #(CLK_PERIOD/2) clk = ~clk;

    // -----------------------------------------------------------------------
    // Utility tasks
    // -----------------------------------------------------------------------

    // Wait N system clocks
    task wait_clocks;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1)
                @(posedge clk);
        end
    endtask

    // I2C START condition: SDA falls while SCL is HIGH
    task i2c_start;
        begin
            sda = 1'b1;
            scl = 1'b1;
            wait_clocks(I2C_HALF);
            sda = 1'b0;           // SDA falls
            wait_clocks(I2C_HALF);
            scl = 1'b0;           // SCL falls → prepare for first bit
            wait_clocks(I2C_SETUP);
        end
    endtask

    // I2C STOP condition: SDA rises while SCL is HIGH
    task i2c_stop;
        begin
            sda = 1'b0;
            wait_clocks(I2C_SETUP);
            scl = 1'b1;
            wait_clocks(I2C_HALF);
            sda = 1'b1;           // SDA rises
            wait_clocks(I2C_HALF);
        end
    endtask

    // Send one bit (MSB or LSB, caller's choice)
    task i2c_send_bit;
        input bit_val;
        begin
            sda = bit_val;
            wait_clocks(I2C_SETUP);
            scl = 1'b1;                    // SCL rises → DUT samples SDA
            wait_clocks(I2C_HALF);
            scl = 1'b0;                    // SCL falls
            wait_clocks(I2C_SETUP);
        end
    endtask

    // Send one full byte, MSB first; then clock one ACK/NACK cycle.
    // `slave_ack` = 1 → slave sends ACK (SDA=0)
    //             = 0 → slave sends NACK (SDA=1)
    task i2c_send_byte;
        input [7:0] data;
        input       slave_ack;
        integer     i;
        begin
            for (i = 7; i >= 0; i = i - 1)
                i2c_send_bit(data[i]);

            // ACK/NACK phase: SDA driven by slave (modelled here)
            sda = ~slave_ack;             // ACK → SDA=0, NACK → SDA=1
            wait_clocks(I2C_SETUP);
            scl = 1'b1;
            wait_clocks(I2C_HALF);
            scl = 1'b0;
            wait_clocks(I2C_SETUP);
            sda = 1'bz;                   // Release bus
        end
    endtask

    // -----------------------------------------------------------------------
    // Result checking helpers
    // -----------------------------------------------------------------------
    integer pass_cnt = 0;
    integer fail_cnt = 0;

    task check;
        input [63:0] got;
        input [63:0] expected;
        input [127:0] label;
        begin
            if (got === expected) begin
                $display("  [PASS] %s : 0x%0h", label, got);
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  [FAIL] %s : got=0x%0h  expected=0x%0h", label, got, expected);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    // -----------------------------------------------------------------------
    // Latch for single-cycle error pulse
    // -----------------------------------------------------------------------
    reg error_latch;

    task clear_error_latch;
        begin @(posedge clk); error_latch = 1'b0; end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            error_latch <= 1'b0;
        else if (error)
            error_latch <= 1'b1;
    end

    // -----------------------------------------------------------------------
    // Monitor: print every time addr_valid or data_valid pulse
    // -----------------------------------------------------------------------
    always @(posedge clk) begin
        if (addr_valid)
            $display("  [MON] addr_valid: addr=0x%0h  rw=%0b  (@%0t ns)", addr, rw, $time);
        if (data_valid)
            $display("  [MON] data_valid: data=0x%0h  ack=%0b  (@%0t ns)", data_byte, ack, $time);
        if (error)
            $display("  [MON] ERROR detected                           (@%0t ns)", $time);
    end

    // -----------------------------------------------------------------------
    // Stimulus
    // -----------------------------------------------------------------------
    initial begin
        $dumpfile("i2c_decoder_tb.vcd");
        $dumpvars(0, i2c_decoder_tb);

        // Default idle state
        scl   = 1'b1;
        sda   = 1'b1;
        rst_n = 1'b0;
        wait_clocks(4);
        rst_n = 1'b1;
        wait_clocks(4);

        // ==================================================================
        // Test 1: WRITE transaction
        //   START | addr=0x4A | W=0 | ACK | data=0xA5 | ACK | data=0x3C | ACK | STOP
        // ==================================================================
        $display("\n=== Test 1: WRITE  addr=0x4A  data=[0xA5, 0x3C] ===");
        i2c_start;
        i2c_send_byte({7'h4A, 1'b0}, 1'b1);   // address + WRITE + slave ACK
        wait_clocks(2);
        check(addr, 7'h4A, "T1 addr");
        check(rw,   1'b0,  "T1 rw  ");

        i2c_send_byte(8'hA5, 1'b1);            // data byte 1 + slave ACK
        wait_clocks(2);
        check(data_byte, 8'hA5, "T1 data[0]");

        i2c_send_byte(8'h3C, 1'b1);            // data byte 2 + slave ACK
        wait_clocks(2);
        check(data_byte, 8'h3C, "T1 data[1]");

        i2c_stop;
        wait_clocks(10);

        // ==================================================================
        // Test 2: READ transaction
        //   START | addr=0x4A | R=1 | ACK | STOP
        // ==================================================================
        $display("\n=== Test 2: READ   addr=0x4A ===");
        i2c_start;
        i2c_send_byte({7'h4A, 1'b1}, 1'b1);   // address + READ + slave ACK
        wait_clocks(2);
        check(addr, 7'h4A, "T2 addr");
        check(rw,   1'b1,  "T2 rw  ");
        i2c_stop;
        wait_clocks(10);

        // ==================================================================
        // Test 3: Repeated START (no STOP between two sub-transactions)
        //   START | addr=0x12 | W | ACK | RS | addr=0x55 | W | ACK | STOP
        // ==================================================================
        $display("\n=== Test 3: Repeated START ===");
        i2c_start;
        i2c_send_byte({7'h12, 1'b0}, 1'b1);
        wait_clocks(2);
        check(addr, 7'h12, "T3 addr1");

        // Repeated START: bring SDA/SCL high, then do START again
        sda = 1'b1;
        wait_clocks(I2C_HALF);
        scl = 1'b1;
        wait_clocks(I2C_HALF);
        i2c_start;                              // Repeated START

        i2c_send_byte({7'h55, 1'b0}, 1'b1);
        wait_clocks(2);
        check(addr, 7'h55, "T3 addr2 (after repeated START)");
        i2c_stop;
        wait_clocks(10);

        // ==================================================================
        // Test 4: NACK from slave after address → error flag expected
        //   START | addr=0x7F | W=0 | NACK | (idle)
        // ==================================================================
        $display("\n=== Test 4: NACK from slave (error expected) ===");
        clear_error_latch;                      // Reset latch before this test
        i2c_start;
        i2c_send_byte({7'h7F, 1'b0}, 1'b0);   // slave_ack=0 → NACK
        wait_clocks(10);
        check(error_latch, 1'b1, "T4 error flag");
        i2c_stop;
        wait_clocks(10);

        // ==================================================================
        // Summary
        // ==================================================================
        $display("\n======================================");
        $display(" Results: %0d PASSED, %0d FAILED", pass_cnt, fail_cnt);
        $display("======================================\n");

        if (fail_cnt == 0)
            $display("ALL TESTS PASSED");
        else
            $display("SOME TESTS FAILED - check log above");

        $finish;
    end

endmodule
