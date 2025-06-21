//==============================================================================
// I2C SLAVE MODULE
// Author:Praveen Saravanan
//==============================================================================
module i2c_slave(
    input  logic        clk,
    input  logic        rst,
    input  logic        scl,                // SCL from master
    inout  wire         sda,                // Bidirectional SDA
    input  logic [6:0]  my_address,         // This slave's 7-bit address
    input  logic [7:0]  tx_data,            // Data to send to master
    output logic [7:0]  rx_data,            // Data received from master
    output logic        data_valid,         // New data received flag
    output logic        read_request        // Master wants to read from slave
);

    // SDA control
    logic sda_oe;
    logic sda_in;
    logic sda_out;
    
    // SCL edge detection (synchronize to internal clock)
    logic [2:0] scl_sync;
    logic scl_rising, scl_falling;
    
    // SDA edge detection for START/STOP
    logic [2:0] sda_sync;
    logic sda_rising, sda_falling;
    
    // FSM states
    typedef enum logic[2:0] {
        IDLE         = 3'b000,
        ADDR_RX      = 3'b001,
        ACK_ADDR     = 3'b010,
        DATA_RX      = 3'b011,
        DATA_TX      = 3'b100,
        ACK_TX       = 3'b101,
        ACK_RX       = 3'b110
    } slave_state_t;
    
    slave_state_t state, next_state;
    
    // Registers
    logic [7:0] addr_shift_reg;
    logic [7:0] data_shift_reg;
    logic [2:0] bit_counter;
    logic address_match;
    logic rw_bit_received;
    logic start_detected, stop_detected;
    
    //==========================================================================
    // SCL AND SDA SYNCHRONIZATION & EDGE DETECTION
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            scl_sync <= 3'b111;
            sda_sync <= 3'b111;
        end else begin
            scl_sync <= {scl_sync[1:0], scl};
            sda_sync <= {sda_sync[1:0], sda_in};
        end
    end
    
    assign scl_rising  = (scl_sync[2:1] == 2'b01);
    assign scl_falling = (scl_sync[2:1] == 2'b10);
    assign sda_rising  = (sda_sync[2:1] == 2'b01);
    assign sda_falling = (sda_sync[2:1] == 2'b10);
    
    //==========================================================================
    // START/STOP CONDITION DETECTION
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            start_detected <= 0;
            stop_detected <= 0;
        end else begin
            // START: SDA falls while SCL is high
            start_detected <= sda_falling & scl_sync[1];
            
            // STOP: SDA rises while SCL is high
            stop_detected <= sda_rising & scl_sync[1];
        end
    end
    
    //==========================================================================
    // STATE MACHINE
    //==========================================================================
    always @(posedge clk) begin
        if (rst || stop_detected) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end
    
    always_comb begin
        next_state = state;
        
        case (state)
            IDLE: begin
                if (start_detected) begin
                    next_state = ADDR_RX;
                end
            end
            
            ADDR_RX: begin
                if (bit_counter == 7 && scl_falling) begin
                    next_state = ACK_ADDR;
                end
            end
            
            ACK_ADDR: begin
                if (scl_falling) begin
                    if (address_match) begin
                        if (rw_bit_received) begin
                            next_state = DATA_TX;  // READ
                        end else begin
                            next_state = DATA_RX;  // WRITE
                        end
                    end else begin
                        next_state = IDLE;  // Address mismatch
                    end
                end
            end
            
            DATA_RX: begin
                if (bit_counter == 7 && scl_falling) begin
                    next_state = ACK_TX;
                end
            end
            
            DATA_TX: begin
                if (bit_counter == 7 && scl_falling) begin
                    next_state = ACK_RX;
                end
            end
            
            ACK_TX: begin
                if (scl_falling) begin
                    next_state = IDLE;  // Single byte transaction
                end
            end
            
            ACK_RX: begin
                if (scl_falling) begin
                    next_state = IDLE;  // Transaction complete
                end
            end
        endcase
    end
    
    //==========================================================================
    // BIT COUNTER AND SHIFT REGISTERS
    //==========================================================================
    always @(posedge clk) begin
        if (rst || start_detected) begin
            bit_counter <= 0;
            addr_shift_reg <= 0;
            data_shift_reg <= 0;
            address_match <= 0;
            rw_bit_received <= 0;
            data_valid <= 0;
            read_request <= 0;
        end else begin
            case (state)
                ADDR_RX: begin
                    if (scl_rising) begin
                        addr_shift_reg <= {addr_shift_reg[6:0], sda_in};
                        bit_counter <= bit_counter + 1;
                        
                        if (bit_counter == 7) begin
                            // Check address match
                            address_match <= (addr_shift_reg[7:1] == my_address);
                            rw_bit_received <= addr_shift_reg[0];
                            if (addr_shift_reg[0]) read_request <= 1;
                        end
                    end
                end
                
                DATA_RX: begin
                    if (scl_rising) begin
                        data_shift_reg <= {data_shift_reg[6:0], sda_in};
                        bit_counter <= bit_counter + 1;
                        
                        if (bit_counter == 7) begin
                            rx_data <= {data_shift_reg[6:0], sda_in};
                            data_valid <= 1;
                        end
                    end
                end
                
                DATA_TX: begin
                    if (bit_counter == 0 && state != ACK_ADDR) begin
                        data_shift_reg <= tx_data;  // Load data to transmit
                    end
                    if (scl_falling) begin
                        data_shift_reg <= {data_shift_reg[6:0], 1'b0};
                        bit_counter <= bit_counter + 1;
                    end
                end
                
                default: begin
                    bit_counter <= 0;
                    data_valid <= 0;
                end
            endcase
        end
    end
    
    //==========================================================================
    // SDA OUTPUT CONTROL
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            sda_oe <= 0;
        end else begin
            case (state)
                ACK_ADDR: begin
                    if (address_match) begin
                        sda_oe <= 1;  // Pull SDA low for ACK
                    end else begin
                        sda_oe <= 0;  // No ACK if address doesn't match
                    end
                end
                
                DATA_TX: begin
                    sda_oe <= ~data_shift_reg[7];  // Drive low for '0', tri-state for '1'
                end
                
                ACK_TX: begin
                    sda_oe <= 1;  // Pull SDA low for ACK
                end
                
                default: begin
                    sda_oe <= 0;  // Tri-state
                end
            endcase
        end
    end
    
    //==========================================================================
    // SDA BIDIRECTIONAL CONTROL
    //==========================================================================
    assign sda = sda_oe ? 1'b0 : 1'bz;
    assign sda_in = sda;

endmodule
