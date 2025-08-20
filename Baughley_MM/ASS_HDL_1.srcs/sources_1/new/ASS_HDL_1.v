`timescale 1ns / 1ps

module Matrix_multi(
    input  wire        clk, rst,wrt_en,
    input  wire signed [7:0] din,
    output reg  signed [16:0] dout
);
    reg signed [7:0] A [0:15];
    reg signed [7:0] B [0:15];
    
    reg [2:0] state;
    reg [5:0] load_count;
    reg [3:0] output_count;
    
    localparam IDLE   = 3'd0, LOAD_A = 3'd1, LOAD_B = 3'd2, OUTPUT = 3'd3;
    
    wire [1:0] row = output_count[3:2];
    wire [1:0] col = output_count[1:0];
    
    wire signed [15:0] p0, p1, p2, p3;
    wire signed [16:0] sum;
    
    baugh_wooley_multiplier bw0(.a(A[{row,2'b00}]), .b(B[{2'b00,col}]), .p(p0));
    baugh_wooley_multiplier bw1(.a(A[{row,2'b01}]), .b(B[{2'b01,col}]), .p(p1));
    baugh_wooley_multiplier bw2(.a(A[{row,2'b10}]), .b(B[{2'b10,col}]), .p(p2));
    baugh_wooley_multiplier bw3(.a(A[{row,2'b11}]), .b(B[{2'b11,col}]), .p(p3));
    
    assign sum = p0 + p1 + p2 + p3;

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE; 
            load_count <= 0; 
            output_count <= 0; 
            dout <= 0;
        end else begin
            case (state)
                IDLE: 
                    if (wrt_en) begin 
                        A[0] <= din; 
                        load_count <= 1; 
                        state <= LOAD_A; 
                    end
                LOAD_A: 
                    if (wrt_en) begin 
                        A[load_count] <= din; 
                        load_count <= load_count + 1;
                        if (load_count==15) state<=LOAD_B;
                    end
                LOAD_B: 
                    if (wrt_en) begin 
                        B[load_count-16] <= din; 
                        load_count <= load_count + 1;
                        if (load_count==31) begin
                            state<= OUTPUT; output_count<=0; 
                        end 
                    end
                OUTPUT: 
                    begin 
                        dout<=sum; 
                        output_count <= output_count + 1;
                        if (output_count==15) state<=IDLE; 
                    end
            endcase
        end
    end
endmodule
module baugh_wooley_multiplier(
    input  wire signed [7:0] a,  
    input  wire signed [7:0] b,  
    output reg  signed [15:0] p 
);
    integer i, j;
    reg signed [15:0] acc;  
    reg bit;                
    always @* begin
        acc = 16'sd0; 
        
        for (i = 0; i <= 6; i=i + 1) begin
            for (j = 0; j <= 6; j=j + 1) begin
                bit = a[i] & b[j];
                acc = acc + (bit << (i + j));
            end
        end
        
        for (j = 0; j <= 6; j=j + 1) begin
            bit = a[7] & b[j];
            acc = acc - (bit << (7 + j));
        end
        
        for (i = 0; i <= 6; i=i + 1) begin
            bit = a[i] & b[7];
            acc = acc - (bit << (i + 7));
        end
        
        bit = a[7] & b[7];
        acc = acc + (bit << 14);
        
        p = acc;
    end
endmodule