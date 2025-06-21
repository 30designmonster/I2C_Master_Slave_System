//==============================================================================
// I2C TESTBENCH
// Author:Praveen Saravanan
//==============================================================================
module i2c_testbench;

    // System signals
    logic clk;
    logic rst;
    
    // I2C bus
    wire sda;
    wire scl;
    
    // Pull-up resistors (essential for I2C)
    pullup(sda);  // Only SDA needs pull-up
    // SCL is driven by master, no pull-up needed
    
    // Master interface
    logic start_transaction;
    logic [7:0] addr_rw;            // 8-bit: [7:1]=address, [0]=R/W
    logic [7:0] master_tx_data;
    logic [7:0] master_rx_data;
    logic transaction_done;
    logic error;
    
    // Slave interface
    logic [6:0] slave_address = 7'h50;  // Slave address
    logic [7:0] slave_tx_data = 8'hAA;  // Data slave will send
    logic [7:0] slave_rx_data;
    logic data_valid;
    logic read_request;
    
    //==========================================================================
    // DUT INSTANTIATIONS
    //==========================================================================
    i2c_master master (
        .clk(clk),
        .rst(rst),
        .start_transaction(start_transaction),
        .addr_rw(addr_rw),
        .tx_data(master_tx_data),
        .rx_data(master_rx_data),
        .transaction_done(transaction_done),
        .error(error),
        .sda(sda),
        .scl(scl)
    );
    
    i2c_slave slave (
        .clk(clk),
        .rst(rst),
        .scl(scl),
        .sda(sda),
        .my_address(slave_address),
        .tx_data(slave_tx_data),
        .rx_data(slave_rx_data),
        .data_valid(data_valid),
        .read_request(read_request)
    );
    
    //==========================================================================
    // CLOCK GENERATION
    //==========================================================================
    parameter CLK_PERIOD = 10;  // 100MHz
    
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end
    
    //==========================================================================
    // TEST TASKS
    //==========================================================================
    
    // Task for I2C write transaction
    task i2c_write(input [6:0] slave_addr, input [7:0] data);
        begin
            $display("Time %0t: Starting I2C WRITE to address 0x%02X, data 0x%02X", 
                     $time, slave_addr, data);
            
            addr_rw = {slave_addr, 1'b0};  // 7-bit address + WRITE bit
            master_tx_data = data;
            start_transaction = 1;
            
            @(posedge clk);
            start_transaction = 0;
            
            // Wait for transaction to complete
            wait(transaction_done);
            @(posedge clk);
            
            if (error) begin
              $display("FAIL: WRITE FAILED: NACK received");
            end else begin
              $display("PASS: WRITE SUCCESS: Data 0x%02X written to address 0x%02X", 
                         data, slave_addr);
            end
            
            #(CLK_PERIOD * 50);  // Inter-transaction delay
        end
    endtask
    
    // Task for I2C read transaction
    task i2c_read(input [6:0] slave_addr);
        begin
            $display("Time %0t: Starting I2C READ from address 0x%02X", $time, slave_addr);
            
            addr_rw = {slave_addr, 1'b1};  // 7-bit address + READ bit
            start_transaction = 1;
            
            @(posedge clk);
            start_transaction = 0;
            
            // Wait for transaction to complete
            wait(transaction_done);
            @(posedge clk);
            
            if (error) begin
              $display("FAIL: READ FAILED: NACK received");
            end else begin
              $display("PASS: READ SUCCESS: Data 0x%02X read from address 0x%02X", 
                         master_rx_data, slave_addr);
            end
            
            #(CLK_PERIOD * 50);  // Inter-transaction delay
        end
    endtask
    
    //==========================================================================
    // MAIN TEST SEQUENCE
    //==========================================================================
    initial begin
        // Initialize signals
        rst = 1;
        start_transaction = 0;
        addr_rw = 0;
        master_tx_data = 0;
        
        // Generate VCD dump
        $dumpfile("i2c_test.vcd");
        $dumpvars(0, i2c_testbench);
        
        $display("=== I2C Master-Slave Test Starting ===");
        
        // Reset sequence
        #(CLK_PERIOD * 10);
        rst = 0;
        #(CLK_PERIOD * 10);
        
        // Test 1: Write to correct slave address
        $display("\n--- Test 1: Write to correct address ---");
        i2c_write(slave_address, 8'h55);
        
        // Verify slave received data
        if (slave_rx_data == 8'h55 && data_valid) begin
          $display("PASS: Slave correctly received: 0x%02X", slave_rx_data);
        end else begin
          $display("FAIL: Slave data mismatch. Expected: 0x55, Got: 0x%02X", slave_rx_data);
        end
        
        // Test 2: Write to wrong slave address  
        $display("\n--- Test 2: Write to wrong address ---");
        i2c_write(7'h77, 8'h33);  // Wrong address
        
        // Test 3: Read from slave
        $display("\n--- Test 3: Read from slave ---");
        slave_tx_data = 8'hCC;  // Change slave's response data
        i2c_read(slave_address);
        
        // Verify master received correct data
        if (master_rx_data == 8'hCC) begin
          $display("PASS: Master correctly received: 0x%02X", master_rx_data);
        end else begin
          $display("FAIL: Master data mismatch. Expected: 0xCC, Got: 0x%02X", master_rx_data);
        end
        
        // Test 4: Multiple transactions
        $display("\n--- Test 4: Multiple transactions ---");
        i2c_write(slave_address, 8'h11);
        i2c_write(slave_address, 8'h22);
        slave_tx_data = 8'h99;
        i2c_read(slave_address);
        
        #(CLK_PERIOD * 100);
        
        $display("\n=== All Tests Complete ===");
        $display("Check waveforms in i2c_test.vcd");
        $finish;
    end
    
    //==========================================================================
    // MONITORING
    //==========================================================================
    
    // Monitor bus activity
    always @(negedge sda) begin
        if (scl) $display("Time %0t: START condition detected", $time);
    end
    
    always @(posedge sda) begin
        if (scl) $display("Time %0t: STOP condition detected", $time);
    end
    
    // Timeout watchdog
    initial begin
        #(CLK_PERIOD * 100000);  // Timeout after many cycles
        $display("ERROR: Test timeout!");
        $finish;
    end

endmodule
