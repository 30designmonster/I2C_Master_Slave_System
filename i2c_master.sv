//==============================================================================
// I2C MASTER MODULE
// Author:Praveen Saravanan
//==============================================================================
module i2c_master(
    input  logic        clk,
    input  logic        rst,
    input  logic        start_transaction,  // Trigger to start I2C transaction
    input  logic [7:0]  addr_rw,           // 8-bit: [7:1]=slave_addr, [0]=R/W bit
    input  logic [7:0]  tx_data,            // Data to transmit
    output logic [7:0]  rx_data,            // Data received
    output logic        transaction_done,    // Transaction complete flag
    output logic        error,              // NACK or bus error
    inout  wire         sda,                // I2C data line (bidirectional)
    output logic        scl                 // I2C clock line
);

    // Internal signals
    logic sclk_internal;
    logic sda_oe;       // SDA output enable (0=tri-state, 1=drive low)
    logic sda_in;
    
    // FSM states
    typedef enum logic[2:0] {
        IDLE     = 3'b000,
        START    = 3'b001,
        ADDR     = 3'b010,
        ACK_ADDR = 3'b011,
        DATA     = 3'b100,
        ACK_DATA = 3'b101,
        STOP     = 3'b110
    } state_t;
    
    state_t cur_state, next_state;
    
    // Counters and registers
    logic [2:0] bit_counter;        // Count bits within a byte (0-7)
    logic [7:0] shift_reg_tx;       // Transmit shift register
    logic [7:0] shift_reg_rx;       // Receive shift register
    logic [7:0] address_byte;       // 8-bit address+RW byte to transmit
    logic       rw_bit;             // Extracted R/W bit
    
    // SCL timing signals
    logic [7:0] clk_counter;
    logic [1:0] scl_quarter;        // 4 phases of SCL
    logic quarter_tick;
    parameter CLK_DIV = 100;        // Adjust for simulation speed
    
    // Control flags
    logic start_condition_done;
    logic stop_condition_done;
    logic ack_received;
    logic byte_complete;
    
    //==========================================================================
    // SCL GENERATOR WITH QUARTER TIMING
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            clk_counter <= 0;
            scl_quarter <= 0;
            quarter_tick <= 0;
        end else if (cur_state != IDLE) begin  // Only generate SCL during transaction
            if (clk_counter == (CLK_DIV/4)-1) begin
                clk_counter <= 0;
                scl_quarter <= scl_quarter + 1;
                quarter_tick <= 1;
            end else begin
                clk_counter <= clk_counter + 1;
                quarter_tick <= 0;
            end
        end else begin
            quarter_tick <= 0;
            scl_quarter <= 0;
            clk_counter <= 0;
        end
    end
    
    // Generate SCL based on quarter
    always @(posedge clk) begin
        if (rst) begin
            sclk_internal <= 1;  // SCL idles high
        end else begin
            case (scl_quarter)
                2'b00: sclk_internal <= 0;  // SCL LOW
                2'b01: sclk_internal <= 1;  // SCL LOW→HIGH
                2'b10: sclk_internal <= 1;  // SCL HIGH  
                2'b11: sclk_internal <= 0;  // SCL HIGH→LOW
            endcase
        end
    end
    
    assign scl = (cur_state == IDLE) ? 1'b1 : sclk_internal;
    
    //==========================================================================
    // FSM STATE REGISTER
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            cur_state <= IDLE;
        end else begin
            cur_state <= next_state;
        end
    end
    
    //==========================================================================
    // BIT COUNTER (inside DATA and ADDR stages)
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            bit_counter <= 0;
            byte_complete <= 0;
        end else begin
            case (cur_state)
                ADDR, DATA: begin
                    if (quarter_tick && scl_quarter == 2'b11) begin  // SCL falling edge
                        if (bit_counter == 7) begin
                            bit_counter <= 0;
                            byte_complete <= 1;
                        end else begin
                            bit_counter <= bit_counter + 1;
                            byte_complete <= 0;
                        end
                    end else begin
                        byte_complete <= 0;
                    end
                end
                default: begin
                    bit_counter <= 0;
                    byte_complete <= 0;
                end
            endcase
        end
    end
    
    //==========================================================================
    // SHIFT REGISTERS
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            shift_reg_tx <= 0;
            shift_reg_rx <= 0;
            address_byte <= 0;
        end else begin
            case (cur_state)
                IDLE: begin
                    if (start_transaction) begin
                        address_byte <= addr_rw;           // Load 8-bit address+RW directly
                        rw_bit <= addr_rw[0];              // Extract R/W bit
                        shift_reg_tx <= tx_data;           // Load TX data
                    end
                end
                
                ADDR: begin
                    if (quarter_tick && scl_quarter == 2'b11) begin  // SCL falling
                        address_byte <= {address_byte[6:0], 1'b0};    // Shift left
                    end
                end
                
                DATA: begin
                    if (quarter_tick && scl_quarter == 2'b11) begin  // SCL falling
                        if (rw_bit == 0) begin  // WRITE
                            shift_reg_tx <= {shift_reg_tx[6:0], 1'b0};   // Shift TX
                        end
                    end
                    if (quarter_tick && scl_quarter == 2'b01) begin  // SCL rising
                        if (rw_bit == 1) begin  // READ
                            shift_reg_rx <= {shift_reg_rx[6:0], sda_in}; // Sample RX
                        end
                    end
                end
            endcase
        end
    end
    
    //==========================================================================
    // FSM NEXT STATE LOGIC
    //==========================================================================
    always_comb begin
        next_state = cur_state;  // Default: stay in current state
        
        case (cur_state)
            IDLE: begin
                if (!rst && start_transaction) begin
                    next_state = START;
                end
            end
            
            START: begin
                if (start_condition_done) begin
                    next_state = ADDR;
                end
            end
            
            ADDR: begin
                if (byte_complete) begin
                    next_state = ACK_ADDR;
                end
            end
            
            ACK_ADDR: begin
                if (quarter_tick && scl_quarter == 2'b10) begin  // After SCL high
                    if (ack_received) begin
                        next_state = DATA;
                    end else begin
                        next_state = STOP;  // NACK - abort transaction
                    end
                end
            end
            
            DATA: begin
                if (byte_complete) begin
                    next_state = ACK_DATA;
                end
            end
            
            ACK_DATA: begin
                if (quarter_tick && scl_quarter == 2'b10) begin  // After SCL high
                    next_state = STOP;  // Single byte transaction
                end
            end
            
            STOP: begin
                if (stop_condition_done) begin
                    next_state = IDLE;
                end
            end
            
            default: next_state = IDLE;
        endcase
    end
    
    //==========================================================================
    // SDA CONTROL LOGIC
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            sda_oe <= 0;
            start_condition_done <= 0;
            stop_condition_done <= 0;
            ack_received <= 0;
            transaction_done <= 0;
            error <= 0;
        end else begin
            case (cur_state)
                IDLE: begin
                    sda_oe <= 0;  // Tri-state (SDA high via pull-up)
                    transaction_done <= 0;
                    start_condition_done <= 0;
                    stop_condition_done <= 0;
                    error <= 0;
                end
                
                START: begin
                    // START condition: SDA goes low while SCL is high
                    if (quarter_tick) begin
                        case (scl_quarter)
                            2'b00: sda_oe <= 0;  // SDA high
                            2'b01: sda_oe <= 0;  // SCL rising, SDA still high
                            2'b10: sda_oe <= 1;  // SCL high, SDA goes low (START!)
                            2'b11: begin
                                sda_oe <= 1;     // Keep SDA low
                                start_condition_done <= 1;
                            end
                        endcase
                    end
                end
                
                ADDR: begin
                    start_condition_done <= 0;
                    if (quarter_tick && scl_quarter == 2'b11) begin  // Setup on SCL falling
                        sda_oe <= ~address_byte[7];  // Drive low for '0', tri-state for '1'
                    end
                end
                
                ACK_ADDR: begin
                    sda_oe <= 0;  // Tri-state to let slave drive SDA
                    if (quarter_tick && scl_quarter == 2'b01) begin  // Sample on SCL rising
                        ack_received <= ~sda_in;  // ACK is SDA low
                        if (sda_in) error <= 1;   // NACK detected
                    end
                end
                
                DATA: begin
                    if (rw_bit == 0) begin  // WRITE
                        if (quarter_tick && scl_quarter == 2'b11) begin
                            sda_oe <= ~shift_reg_tx[7];  // Output data bit
                        end
                    end else begin  // READ
                        sda_oe <= 0;  // Tri-state, let slave drive
                    end
                end
                
                ACK_DATA: begin
                    if (rw_bit == 0) begin  // WRITE - wait for slave ACK
                        sda_oe <= 0;
                    end else begin  // READ - master sends NACK (end of read)
                        sda_oe <= 0;  // NACK = don't pull SDA low
                    end
                end
                
                STOP: begin
                    // STOP condition: SDA goes high while SCL is high
                    if (quarter_tick) begin
                        case (scl_quarter)
                            2'b00: sda_oe <= 1;  // SDA low
                            2'b01: sda_oe <= 1;  // SCL rising, SDA still low
                            2'b10: sda_oe <= 0;  // SCL high, SDA goes high (STOP!)
                            2'b11: begin
                                sda_oe <= 0;     // Keep SDA high
                                stop_condition_done <= 1;
                                transaction_done <= 1;
                            end
                        endcase
                    end
                end
            endcase
        end
    end
    
    //==========================================================================
    // SDA BIDIRECTIONAL CONTROL
    //==========================================================================
    assign sda = sda_oe ? 1'b0 : 1'bz;  // Open-drain: drive low or tri-state
    assign sda_in = sda;                 // Read SDA input
    
    //==========================================================================
    // OUTPUT ASSIGNMENTS
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            rx_data <= 0;
        end else if (cur_state == ACK_DATA && rw_bit == 1) begin
            rx_data <= shift_reg_rx;  // Capture received data
        end
    end

endmodule
