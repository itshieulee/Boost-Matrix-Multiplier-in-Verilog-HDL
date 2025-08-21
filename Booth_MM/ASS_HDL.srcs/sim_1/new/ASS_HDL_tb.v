`timescale 1ns / 1ps

module tb_Matrix_multi_scaled_time;

    // Inputs
    reg clk, rst, wrt_en;
    reg signed [7:0] din;

    // Outputs
    wire signed [16:0] dout;

    // Internal signals for monitoring
    wire [2:0] state_monitor;
    wire [3:0] output_count_monitor;

    // Instantiate the Unit Under Test (UUT)
    Matrix_multi uut (
        .clk(clk),
        .rst(rst),
        .wrt_en(wrt_en),
        .din(din),
        .dout(dout)
    );

    // Expose internal signals for monitoring
    assign state_monitor = uut.state;
    assign output_count_monitor = uut.output_count;

    // Clock generation: 2ns period (for faster simulation)
    always #1 clk = ~clk;

    // Matrix and result storage
    reg signed [7:0] matrix_A [0:15];
    reg signed [7:0] matrix_B [0:15];
    integer i;

    initial begin
        // Initialize signals
        clk = 0;
        rst = 1;
        wrt_en = 0;
        din = 0;

        // Brief reset (2ns)
        #2 rst = 0;

        // Initialize matrix_A with 1- and 2-digit decimal numbers
        matrix_A[0]  = 8'd3;   matrix_A[1]  = 8'd7;   matrix_A[2]  = 8'd12;  matrix_A[3]  = 8'd2;
        matrix_A[4]  = 8'd1;   matrix_A[5]  = 8'd4;   matrix_A[6]  = 8'd6;   matrix_A[7]  = 8'd8;
        matrix_A[8]  = 8'd9;   matrix_A[9]  = 8'd5;   matrix_A[10] = 8'd10;  matrix_A[11] = 8'd11;
        matrix_A[12] = 8'd7;   matrix_A[13] = 8'd2;   matrix_A[14] = 8'd4;   matrix_A[15] = 8'd8;

        // Initialize matrix_B with 1- and 2-digit decimal numbers
        matrix_B[0]  = 8'd5;   matrix_B[1]  = 8'd1;   matrix_B[2]  = 8'd3;   matrix_B[3]  = 8'd8;
        matrix_B[4]  = 8'd2;   matrix_B[5]  = 8'd7;   matrix_B[6]  = 8'd4;   matrix_B[7]  = 8'd6;
        matrix_B[8]  = 8'd9;   matrix_B[9]  = 8'd5;   matrix_B[10] = 8'd2;   matrix_B[11] = 8'd1;
        matrix_B[12] = 8'd3;   matrix_B[13] = 8'd6;   matrix_B[14] = 8'd1;   matrix_B[15] = 8'd7;

        // --- Write matrix A ---
        #2 wrt_en = 1;
        for (i = 0; i < 16; i = i + 1) begin
            din = matrix_A[i];
            #2; // 2ns per element (matches clock)
        end
        wrt_en = 0;

        // --- Write matrix B ---
        wrt_en = 1;
        for (i = 0; i < 16; i = i + 1) begin
            din = matrix_B[i];
            #2; // 2ns per element (matches clock)
        end
        wrt_en = 0;

        // Wait for OUTPUT state
        wait(state_monitor == 3);

        // Print each output as it appears, no array used
        for (i = 0; i < 16; i = i + 1) begin
            @(posedge clk);
            $display("Output %0d: %0d", i, dout);
        end

        #200 $finish;
    end

endmodule
