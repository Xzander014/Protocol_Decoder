// =============================================================================
// Module  : i2c_decoder
// Purpose : Decodes I2C bus (SCL + SDA) and extracts:
//             - 7-bit slave address
//             - R/W bit
//             - 8-bit data byte(s)
//             - ACK/NACK from slave
//             - Valid pulse when a complete address/data byte is captured
//
// I2C Protocol Overview:
//   START  : SDA falls while SCL is HIGH
//   STOP   : SDA rises  while SCL is HIGH
//   DATA   : Sampled on rising edge of SCL (MSB first)
//   ACK    : Pulled LOW by the receiver on the 9th clock after 8 data bits
// =============================================================================

module i2c_decoder (
    input  wire       clk,          // System clock (used for edge detection)
    input  wire       rst_n,        // Active-low synchronous reset
    input  wire       scl,          // I2C clock line
    input  wire       sda,          // I2C data  line

    // ---- Decoded outputs ----
    output reg  [6:0] addr,         // 7-bit slave address
    output reg        rw,           // 1 = Read, 0 = Write
    output reg  [7:0] data_byte,    // Latest captured data byte
    output reg        addr_valid,   // Pulsed 1 cycle when address + R/W ready
    output reg        data_valid,   // Pulsed 1 cycle when data byte ready
    output reg        ack,          // ACK bit captured after each byte
    output reg        busy,         // HIGH while a transaction is in progress
    output reg        error         // HIGH if unexpected condition detected
);

    // -----------------------------------------------------------------------
    // Synchronise SCL/SDA into the system clock domain (2-FF synchroniser)
    // -----------------------------------------------------------------------
    reg scl_r1, scl_r2, scl_r3;
    reg sda_r1, sda_r2, sda_r3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_r1 <= 1'b1; scl_r2 <= 1'b1; scl_r3 <= 1'b1;
            sda_r1 <= 1'b1; sda_r2 <= 1'b1; sda_r3 <= 1'b1;
        end else begin
            scl_r1 <= scl;  scl_r2 <= scl_r1;  scl_r3 <= scl_r2;
            sda_r1 <= sda;  sda_r2 <= sda_r1;  sda_r3 <= sda_r2;
        end
    end

    // Stable (debounced) versions
    wire scl_stable = scl_r2;
    wire sda_stable = sda_r2;

    // Edge detection on stable signals
    wire scl_rising  = ( scl_r2 & ~scl_r3);
    wire scl_falling = (~scl_r2 &  scl_r3);
    wire sda_rising  = ( sda_r2 & ~sda_r3);
    wire sda_falling = (~sda_r2 &  sda_r3);

    // -----------------------------------------------------------------------
    // START/STOP detection
    //   START : SDA falls while SCL is HIGH
    //   STOP  : SDA rises  while SCL is HIGH
    // -----------------------------------------------------------------------
    wire start_det = sda_falling & scl_stable;
    wire stop_det  = sda_rising  & scl_stable;

    // -----------------------------------------------------------------------
    // FSM state encoding
    // -----------------------------------------------------------------------
    localparam [2:0]
        S_IDLE      = 3'd0,   // Waiting for START
        S_ADDR      = 3'd1,   // Receiving 7-bit address
        S_RW        = 3'd2,   // Receiving R/W bit
        S_ACK_ADDR  = 3'd3,   // ACK after address byte
        S_DATA      = 3'd4,   // Receiving 8-bit data
        S_ACK_DATA  = 3'd5;   // ACK after data byte

    reg [2:0] state;
    reg [3:0] bit_cnt;        // Counts bits within a phase (0-7)
    reg [7:0] shift_reg;      // Shift register for incoming bits

    // -----------------------------------------------------------------------
    // FSM
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            bit_cnt    <= 4'd0;
            shift_reg  <= 8'd0;
            addr       <= 7'd0;
            rw         <= 1'b0;
            data_byte  <= 8'd0;
            addr_valid <= 1'b0;
            data_valid <= 1'b0;
            ack        <= 1'b0;
            busy       <= 1'b0;
            error      <= 1'b0;
        end else begin
            // Default pulse clears
            addr_valid <= 1'b0;
            data_valid <= 1'b0;
            error      <= 1'b0;

            // STOP always returns to IDLE regardless of current state
            if (stop_det) begin
                state <= S_IDLE;
                busy  <= 1'b0;
            end
            // Repeated START: restart from address phase
            else if (start_det) begin
                state     <= S_ADDR;
                bit_cnt   <= 4'd0;
                shift_reg <= 8'd0;
                busy      <= 1'b1;
            end
            else begin
                case (state)
                    // ------------------------------------------
                    S_IDLE: begin
                        busy <= 1'b0;
                        // Stays here until start_det handled above
                    end

                    // ------------------------------------------
                    // Sample bits on SCL rising edge
                    // ------------------------------------------
                    S_ADDR: begin
                        if (scl_rising) begin
                            shift_reg <= {shift_reg[6:0], sda_stable};
                            if (bit_cnt == 4'd6) begin
                                addr    <= {shift_reg[5:0], sda_stable};
                                bit_cnt <= 4'd0;
                                state   <= S_RW;
                            end else begin
                                bit_cnt <= bit_cnt + 1'b1;
                            end
                        end
                    end

                    // ------------------------------------------
                    S_RW: begin
                        if (scl_rising) begin
                            rw         <= sda_stable;
                            addr_valid <= 1'b1;       // Address + R/W captured
                            bit_cnt    <= 4'd0;
                            state      <= S_ACK_ADDR;
                        end
                    end

                    // ------------------------------------------
                    S_ACK_ADDR: begin
                        if (scl_rising) begin
                            ack   <= ~sda_stable;     // ACK = SDA LOW
                            if (sda_stable == 1'b0) begin
                                // ACK received → move to data phase
                                shift_reg <= 8'd0;
                                bit_cnt   <= 4'd0;
                                state     <= S_DATA;
                            end else begin
                                // NACK → flag error, wait for STOP/START
                                error <= 1'b1;
                                state <= S_IDLE;
                            end
                        end
                    end

                    // ------------------------------------------
                    S_DATA: begin
                        if (scl_rising) begin
                            shift_reg <= {shift_reg[6:0], sda_stable};
                            if (bit_cnt == 4'd7) begin
                                data_byte  <= {shift_reg[6:0], sda_stable};
                                data_valid <= 1'b1;
                                bit_cnt    <= 4'd0;
                                state      <= S_ACK_DATA;
                            end else begin
                                bit_cnt <= bit_cnt + 1'b1;
                            end
                        end
                    end

                    // ------------------------------------------
                    S_ACK_DATA: begin
                        if (scl_rising) begin
                            ack <= ~sda_stable;
                            // Regardless of ACK/NACK continue to next byte
                            // (Master may NACK the last byte before STOP)
                            shift_reg <= 8'd0;
                            bit_cnt   <= 4'd0;
                            state     <= S_DATA;
                        end
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule
