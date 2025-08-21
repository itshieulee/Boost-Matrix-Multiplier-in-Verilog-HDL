module Matrix_multi(
    input  wire        clk, rst, wrt_en,
    input  wire signed [7:0] din,
    output reg  signed [16:0] dout
);
    reg signed [7:0] A [0:15];
    reg signed [7:0] B [0:15];
    
    reg [2:0] state;
    reg [5:0] load_count;
    reg [3:0] output_count;
    reg [1:0] compute_count; // Counter for 3-cycle multiplication latency
    
    localparam IDLE    = 3'd0, 
               LOAD_A  = 3'd1, 
               LOAD_B  = 3'd2, 
               COMPUTE = 3'd3, 
               OUTPUT  = 3'd4;
    
    wire [1:0] row = output_count[3:2];
    wire [1:0] col = output_count[1:0];
    
    wire signed [15:0] p0, p1, p2, p3;
    wire valid0, valid1, valid2, valid3;
    reg  ld; // Load signal for Booth multipliers
    wire signed [16:0] sum;
    
    // Instantiate four Booth multipliers
    Booth_Multiplier_4xA #(
        .N(8)
    ) bw0 (
        .Rst(rst), 
        .Clk(clk), 
        .Ld(ld), 
        .M(A[{row,2'b00}]), 
        .R(B[{2'b00,col}]), 
        .Valid(valid0), 
        .P(p0)
    );
    Booth_Multiplier_4xA #(
        .N(8)
    ) bw1 (
        .Rst(rst), 
        .Clk(clk), 
        .Ld(ld), 
        .M(A[{row,2'b01}]), 
        .R(B[{2'b01,col}]), 
        .Valid(valid1), 
        .P(p1)
    );
    Booth_Multiplier_4xA #(
        .N(8)
    ) bw2 (
        .Rst(rst), 
        .Clk(clk), 
        .Ld(ld), 
        .M(A[{row,2'b10}]), 
        .R(B[{2'b10,col}]), 
        .Valid(valid2), 
        .P(p2)
    );
    Booth_Multiplier_4xA #(
        .N(8)
    ) bw3 (
        .Rst(rst), 
        .Clk(clk), 
        .Ld(ld), 
        .M(A[{row,2'b11}]), 
        .R(B[{2'b11,col}]), 
        .Valid(valid3), 
        .P(p3)
    );
    
    assign sum = p0 + p1 + p2 + p3;

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            load_count <= 0;
            output_count <= 0;
            compute_count <= 0;
            ld <= 0;
            dout <= 0;
        end else begin
            case (state)
                IDLE: begin
                    if (wrt_en) begin 
                        A[0] <= din; 
                        load_count <= 1; 
                        state <= LOAD_A; 
                    end
                end
                LOAD_A: begin
                    if (wrt_en) begin 
                        A[load_count] <= din; 
                        load_count <= load_count + 1;
                        if (load_count == 15) state <= LOAD_B;
                    end
                end
                LOAD_B: begin
                    if (wrt_en) begin 
                        B[load_count-16] <= din; 
                        load_count <= load_count + 1;
                        if (load_count == 31) begin
                            state <= COMPUTE;
                            output_count <= 0;
                            compute_count <= 0;
                            ld <= 1; // Start first multiplication
                        end 
                    end
                end
                COMPUTE: begin
                    ld <= 0; // Pulse ld for one cycle
                    compute_count <= compute_count + 1;
                    if (compute_count == 2) begin // 3 cycles total (0,1,2)
                        state <= OUTPUT;
                        compute_count <= 0;
                    end
                end
                OUTPUT: begin
                    dout <= sum;
                    output_count <= output_count + 1;
                    ld <= 1; // Start next multiplication
                    if (output_count == 15) begin
                        state <= IDLE;
                        ld <= 0;
                    end else begin
                        state <= COMPUTE; // Return to COMPUTE for next element
                    end
                end
            endcase
        end
    end
endmodule

module Booth_Multiplier_4xA #(
    parameter N = 8                // Width = N: multiplicand & multiplier
)(
    input   Rst,                    // Reset
    input   Clk,                    // Clock
    input   Ld,                     // Load Registers and Start Multiplier
    input   [(N - 1):0] M,          // Multiplicand
    input   [(N - 1):0] R,          // Multiplier
    output  reg Valid,              // Product Valid
    output  reg [((2*N) - 1):0] P   // Product <= M * R
);

    localparam pNumCycles = ((N + 1)/4);    // No. of cycles required for product

    reg     [4:0] Cntr;                     // Operation Counter
    reg     [4:0] Booth;                    // Booth Recoding Field
    reg     Guard;                          // Shift Bit for Booth Recoding
    reg     [(N + 3):0] A;                  // Multiplicand w/ guards
    wire    [(N + 3):0] Mx8, Mx4, Mx2, Mx1; // Multiplicand products w/ guards
    reg     PnM_B, M_Sel_B, En_B;           // Operand B Control Triple
    reg     PnM_C, M_Sel_C, En_C;           // Operand C Control Triple
    wire    [(N + 3):0] Hi;                 // Upper Half of Product w/ guards
    reg     [(N + 3):0] B, C;               // Adder tree Operand Inputs
    reg     Ci_B, Ci_C;                     // Adder tree Carry Inputs
    wire    [(N + 3):0] T, S;               // Adder Tree Outputs w/ guards
    reg     [((2*N) + 3):0] Prod;           // Double Length Product w/ guards

    always @(posedge Clk)
    begin
        if(Rst)
            Cntr <= #1 0;
        else if(Ld)
            Cntr <= #1 pNumCycles;
        else if(|Cntr)
            Cntr <= #1 (Cntr - 1);
    end

    always @(posedge Clk)
    begin
        if(Rst)
            A <= #1 0;
        else if(Ld)
            A <= #1 {{4{M[(N - 1)]}}, M};
    end

    assign Mx8 = {A, 3'b0};
    assign Mx4 = {A, 2'b0};
    assign Mx2 = {A, 1'b0};
    assign Mx1 = A;

    always @(*) Booth <= {Prod[3:0], Guard};

    assign Hi = Prod[((2*N) + 3):N];

    always @(*)
    begin
        case(Booth)
            5'b00000 : {PnM_B, M_Sel_B, En_B} <= 3'b000;
            5'b00001 : {PnM_B, M_Sel_B, En_B} <= 3'b000;
            5'b00010 : {PnM_B, M_Sel_B, En_B} <= 3'b000;
            5'b00011 : {PnM_B, M_Sel_B, En_B} <= 3'b000;
            5'b00100 : {PnM_B, M_Sel_B, En_B} <= 3'b000;
            5'b00101 : {PnM_B, M_Sel_B, En_B} <= 3'b001;
            5'b00110 : {PnM_B, M_Sel_B, En_B} <= 3'b001;
            5'b00111 : {PnM_B, M_Sel_B, En_B} <= 3'b001;
            5'b01000 : {PnM_B, M_Sel_B, En_B} <= 3'b001;
            5'b01001 : {PnM_B, M_Sel_B, En_B} <= 3'b001;
            5'b01010 : {PnM_B, M_Sel_B, En_B} <= 3'b001;
            5'b01101 : {PnM_B, M_Sel_B, En_B} <= 3'b011;
            5'b01110 : {PnM_B, M_Sel_B, En_B} <= 3'b011;
            5'b01111 : {PnM_B, M_Sel_B, En_B} <= 3'b011;
            5'b10000 : {PnM_B, M_Sel_B, En_B} <= 3'b111;
            5'b10001 : {PnM_B, M_Sel_B, En_B} <= 3'b111;
            5'b10010 : {PnM_B, M_Sel_B, En_B} <= 3'b111;
            5'b10011 : {PnM_B, M_Sel_B, En_B} <= 3'b101;
            5'b10100 : {PnM_B, M_Sel_B, En_B} <= 3'b101;
            5'b10101 : {PnM_B, M_Sel_B, En_B} <= 3'b101;
            5'b10110 : {PnM_B, M_Sel_B, En_B} <= 3'b101;
            5'b10111 : {PnM_B, M_Sel_B, En_B} <= 3'b101;
            5'b11000 : {PnM_B, M_Sel_B, En_B} <= 3'b101;
            5'b11001 : {PnM_B, M_Sel_B, En_B} <= 3'b101;
            5'b11010 : {PnM_B, M_Sel_B, En_B} <= 3'b101;
            5'b11011 : {PnM_B, M_Sel_B, En_B} <= 3'b000;
            5'b11100 : {PnM_B, M_Sel_B, En_B} <= 3'b000;
            5'b11101 : {PnM_B, M_Sel_B, En_B} <= 3'b000;
            5'b11110 : {PnM_B, M_Sel_B, En_B} <= 3'b000;
            5'b11111 : {PnM_B, M_Sel_B, En_B} <= 3'b000;
            default  : {PnM_B, M_Sel_B, En_B} <= 3'b000;
        endcase
    end

    always @(*)
    begin
        case(Booth)
            5'b00000 : {PnM_C, M_Sel_C, En_C} <= 3'b000;
            5'b00001 : {PnM_C, M_Sel_C, En_C} <= 3'b001;
            5'b00010 : {PnM_C, M_Sel_C, En_C} <= 3'b001;
            5'b00011 : {PnM_C, M_Sel_C, En_C} <= 3'b011;
            5'b00100 : {PnM_C, M_Sel_C, En_C} <= 3'b011;
            5'b00101 : {PnM_C, M_Sel_C, En_C} <= 3'b101;
            5'b00110 : {PnM_C, M_Sel_C, En_C} <= 3'b101;
            5'b00111 : {PnM_C, M_Sel_C, En_C} <= 3'b000;
            5'b01000 : {PnM_C, M_Sel_C, En_C} <= 3'b000;
            5'b01001 : {PnM_C, M_Sel_C, En_C} <= 3'b001;
            5'b01010 : {PnM_C, M_Sel_C, En_C} <= 3'b001;
            5'b01011 : {PnM_C, M_Sel_C, En_C} <= 3'b011;
            5'b01100 : {PnM_C, M_Sel_C, En_C} <= 3'b011;
            5'b01101 : {PnM_C, M_Sel_C, En_C} <= 3'b101;
            5'b01110 : {PnM_C, M_Sel_C, En_C} <= 3'b101;
            5'b01111 : {PnM_C, M_Sel_C, En_C} <= 3'b000;
            5'b10000 : {PnM_C, M_Sel_C, En_C} <= 3'b000;
            5'b10001 : {PnM_C, M_Sel_C, En_C} <= 3'b001;
            5'b10010 : {PnM_C, M_Sel_C, En_C} <= 3'b001;
            5'b10011 : {PnM_C, M_Sel_C, En_C} <= 3'b111;
            5'b10100 : {PnM_C, M_Sel_C, En_C} <= 3'b111;
            5'b10101 : {PnM_C, M_Sel_C, En_C} <= 3'b101;
            5'b10110 : {PnM_C, M_Sel_C, En_C} <= 3'b101;
            5'b10111 : {PnM_C, M_Sel_C, En_C} <= 3'b000;
            5'b11000 : {PnM_C, M_Sel_C, En_C} <= 3'b000;
            5'b11001 : {PnM_C, M_Sel_C, En_C} <= 3'b001;
            5'b11010 : {PnM_C, M_Sel_C, En_C} <= 3'b001;
            5'b11011 : {PnM_C, M_Sel_C, En_C} <= 3'b111;
            5'b11100 : {PnM_C, M_Sel_C, En_C} <= 3'b111;
            5'b11101 : {PnM_C, M_Sel_C, En_C} <= 3'b101;
            5'b11110 : {PnM_C, M_Sel_C, En_C} <= 3'b101;
            5'b11111 : {PnM_C, M_Sel_C, En_C} <= 3'b000;
            default  : {PnM_C, M_Sel_C, En_C} <= 3'b000;
        endcase
    end

    always @(*)
    begin
        case({PnM_B, M_Sel_B, En_B})
            3'b001  : {Ci_B, B} <= {1'b0,  Mx4};
            3'b011  : {Ci_B, B} <= {1'b0,  Mx8};
            3'b101  : {Ci_B, B} <= {1'b1, ~Mx4};
            3'b111  : {Ci_B, B} <= {1'b1, ~Mx8};
            default : {Ci_B, B} <= 0;
        endcase
    end

    always @(*)
    begin
        case({PnM_C, M_Sel_C, En_C})
            3'b001  : {Ci_C, C} <= {1'b0,  Mx1};
            3'b011  : {Ci_C, C} <= {1'b0,  Mx2};
            3'b101  : {Ci_C, C} <= {1'b1, ~Mx1};
            3'b111  : {Ci_C, C} <= {1'b1, ~Mx2};
            default : {Ci_C, C} <= 0;
        endcase
    end

    assign T = Hi + B + Ci_B;
    assign S = T + C + Ci_C;

    always @(posedge Clk)
    begin
        if(Rst)
            Prod <= #1 0;
        else if(Ld)
            Prod <= #1 R;
        else if(|Cntr)
            Prod <= #1 {{4{S[(N + 3)]}}, S, Prod[(N - 1):4]};
    end

    always @(posedge Clk)
    begin
        if(Rst)
            Guard <= #1 0;
        else if(Ld)
            Guard <= #1 0;
        else if(|Cntr)
            Guard <= #1 Prod[3];
    end

    always @(posedge Clk)
    begin
        if(Rst)
            P <= #1 0;
        else if(Cntr == 1)
            P <= #1 {S, Prod[(N - 1):4]};
    end

    always @(posedge Clk)
    begin
        if(Rst)
            Valid <= #1 0;
        else
            Valid <= #1 (Cntr == 1);
    end
endmodule