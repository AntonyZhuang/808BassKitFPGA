// Part 2 skeleton

module fsmtest
	(
		CLOCK_50,						//	On Board 50 MHz
		// Your inputs and outputs here
		SW,
		KEY,							// On Board Keys
		LEDR,							// HEDR [4:0]
		HEX0,							// HEX display
		HEX1,
		HEX4,
		HEX5,
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,   						//	VGA Blue[9:0]
		AUD_ADCDAT,

		// Bidirectionals
		AUD_BCLK,
		AUD_ADCLRCK,
		AUD_DACLRCK,
		PS2_CLK,
		PS2_DAT,
		

		FPGA_I2C_SDAT,

		// Outputs
		AUD_XCK,
		AUD_DACDAT,

		FPGA_I2C_SCLK
	);

	input			CLOCK_50;				//	50 MHz
	input	[3:0]	KEY;					
	input [9:0] SW;
	output wire   [9:0] LEDR;						// LEDR [4:0]
	output wire   [6:0]HEX0;        			// HEX display
	output wire   [6:0]HEX1;
	output wire [6:0] HEX4;
	output wire [6:0] HEX5;

	// Declare your inputs and outputs here
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[7:0]	VGA_R;   				//	VGA Red[7:0] Changed from 10 to 8-bit DAC
	output	[7:0]	VGA_G;	 				//	VGA Green[7:0]
	output	[7:0]	VGA_B;   				//	VGA Blue[7:0]
	
	input                                AUD_ADCDAT;

	// Bidirectionals
	inout                                AUD_BCLK;    
	inout                                AUD_ADCLRCK;
	inout                                AUD_DACLRCK;
	inout PS2_CLK;
	inout PS2_DAT;
	


	inout                                FPGA_I2C_SDAT;

	// Outputs
	output                                AUD_XCK;
	output                                AUD_DACDAT;

	
	output                                FPGA_I2C_SCLK;

	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.

	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;
	wire r, p ,b ,w1;
	wire [2:0] c;
	wire [6:0] xy;
	wire [3:0] wHEX;
	wire [3:0] aluHEX;
	wire key1;
	wire [3:0] DataResult;
	wire keyboardInput;
	
	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(r),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "drumfinal.mif";
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn
	// for the VGA controller, in addition to any other functionality your design may require.
   ps2audio keyaud(.CLOCK(CLOCK_50), .PS2_CLK(PS2_CLK), .PS2_DAT(PS2_DAT), .keyInput(keyboardInput), .HEX4(HEX4), .HEX5(HEX5));

	Audio_Example d1(CLOCK_50, key1, keyboardInput, AUD_ADCDAT, AUD_BCLK, AUD_ADCLRCK, AUD_DACLRCK, FPGA_I2C_SDAT, AUD_XCK, AUD_DACDAT, FPGA_I2C_SCLK, DataResult);
	fill f1(.Clock(CLOCK_50), .Reset(~KEY[3]), .w(~KEY[1]), .z(LEDR[5]), .CurState(LEDR[4:0]), .iResetn(r), .iPlotBox(p), .iBlack(b), .iLoadX(w1), .iColour(c), .iXY_Coord(xy),.HEX(wHEX),.p(~KEY[2]),.m(key1),.yin(7'd16));
	hex_decoder q0(.c(wHEX[3:0]),.display(HEX0));
	project_vga p1(.iResetn(r),.iPlotBox(p),.iBlack(b),.iColour(c),.iLoadX(w1),.iXY_Coord(xy),.iClock(CLOCK_50),.oX(x),.oY(y),/*.oColour(colour),*/.oPlot(writeEn));

	ALU h1(CLOCK_50, ~KEY[3], KEY[0], SW[3:0], DataResult, LEDR[9],aluHEX[3:0]);
	hex_decoder q1(.c(aluHEX[3:0]),.display(HEX1));
assign colour = aluHEX[3:0];
	endmodule
	








module fill(Clock,Reset,w,z,CurState,iResetn,iPlotBox,iBlack,iLoadX,iColour,iXY_Coord,HEX,p,m,yin);
    input Clock;
    input Reset;
    input w;
	 input p;
	 input [6:0] yin;
    output z;
	 output m;
    output [4:0] CurState;
	 
   output reg iResetn, iPlotBox, iBlack, iLoadX;
   output reg [2:0] iColour;
   output reg [6:0] iXY_Coord;
	output reg [3:0] HEX;
   reg [3:0] y_Q, Y_D; // y_Q represents current state, Y_D represents next state
   //wire [2:0] counter5cycle;
    localparam A = 5'b00000, B = 5'b00001, C = 5'b00010, D = 5'b00011, E = 5'b00100, F = 5'b00101, G = 5'b00110, H = 5'b00111,
	 I = 5'b01000, J = 5'b01001, K = 5'b01010, L = 5'b01011, M = 5'b01100, N = 5'b01101,
	 O = 5'b01111, P = 5'b10000, Q = 5'b10001, R = 5'b10010, S = 5'b10011, T = 5'b10100,
	 U = 5'b10101, V = 5'b10110;


    //State table
    //The state table should only contain the logic for state transitions
    //Do not mix in any output logic. The output logic should be handled separately.
    //This will make it easier to read, modify and debug the code.
    always@(*)
    begin: state_table
        case (y_Q)
            A: begin
						 if(!w) Y_D = A;
                   else Y_D = B;
               end		//RESET
            B: begin
                   Y_D = C;
               end		//empty
            C: begin
                   Y_D = D; 
               end		//put x
            D: begin
                   Y_D = E;
               end		//iLOADX
            E: begin
                   Y_D = F;
               end		//put y
            F: begin
                   Y_D = G;
               end		//put color
            G: begin
                   Y_D = H;
               end		//iplotbox
            H: begin
                   if(!p) Y_D = H;
                   else Y_D = K;
               end		//empty
					
					

            K: begin
                   Y_D = L;
               end		//put y
            L: begin
                   Y_D = M;
               end		//put color
            M: begin
						 Y_D = A;
               end		//iplotbox

            default: Y_D = A;
        endcase
    end // state_table
	
	//SIGNALS IN EACH STATE
		always @(*)
		begin: enable_signals
			case (y_Q)
            A: begin
                  iResetn <= 0;
						iPlotBox <= 0;
						iBlack <= 0;
						iLoadX <= 0;
						iColour <= 3'd0;
						iXY_Coord <= 7'd0;
						HEX <= 3'b000;
               end		//RESET
            B: begin
                  iResetn <= 1;
						iPlotBox <= 0;
						iBlack <= 0;
						iLoadX <= 0;
						iColour <= 3'd0;
						iXY_Coord <= 7'd0;
               end		//empty
            C: begin
                  iResetn <= 1;
						iPlotBox <= 0;
						iBlack <= 0;
						iLoadX <= 0;
						iColour <= 3'd0;
						iXY_Coord <= 7'd86;
               end		//put x
            D: begin
                  iResetn <= 1;
						iPlotBox <= 0;
						iBlack <= 0;
						iLoadX <= 1;
						iColour <= 3'd0;
						iXY_Coord <= 7'd83;
               end		//iloadx
            E: begin
                  iResetn <= 1;
						iPlotBox <= 0;
						iBlack <= 0;
						iLoadX <= 0;
						iColour <= 3'd0;
						iXY_Coord <= 7'd60;
               end		//put y
            F: begin
                  iResetn <= 1;
						iPlotBox <= 0;
						iBlack <= 0;
						iLoadX <= 0;
						iColour <= 3'd4;
						iXY_Coord <= 7'd60;
               end		//put color
            G: begin
                  iResetn <= 1;
						iPlotBox <= 1;
						iBlack <= 0;
						iLoadX <= 0;
						iColour <= 3'd4;
						iXY_Coord <= 7'd60;
						HEX <= 3'b001;
               end		//iplotbox
				H: begin
                  iResetn <= 1;
						iPlotBox <= 0;
						iBlack <= 0;
						iLoadX <= 0;
						iColour <= 3'd0;
						iXY_Coord <= 7'd0;
               end		//HOLD
					

				K: begin
                  iResetn <= 1;
						iPlotBox <= 0;
						iBlack <= 0;
						iLoadX <= 0;
						iColour <= 3'd0;
						iXY_Coord <= 7'd40;
               end		//clear
            L: begin
                  iResetn <= 1;
						iPlotBox <= 0;
						iBlack <= 1;
						iLoadX <= 0;
						iColour <= 3'd0;
						iXY_Coord <= 7'd0;
               end		//iblack
            M: begin
                  iResetn <= 1;
						iPlotBox <= 0;
						iBlack <= 0;
						iLoadX <= 0;
						iColour <= 3'd0;
						iXY_Coord <= 7'd0;
               end		//hold

			endcase
		end
	
    // State Registers
    always @(posedge Clock)
    begin //state_FFs
        if(Reset == 1'b1)
		  begin
            y_Q <=  A; // Should set reset state to state A
				end
        else
            y_Q <= Y_D;
    end // state_FFS

    // Output logic
    // Set z to 1 to turn on LED when in relevant states
	 // intantiate project_vga to output a square when in relevant states
    assign z = (y_Q == S);
    assign CurState = y_Q;
	 assign m = w;
endmodule


module project_vga(iResetn,iPlotBox,iBlack,iColour,iLoadX,iXY_Coord,iClock,oX,oY,oColour,oPlot,oDone);
   parameter X_SCREEN_PIXELS = 8'd160;
   parameter Y_SCREEN_PIXELS = 7'd120;

   input wire iResetn, iPlotBox, iBlack, iLoadX;
   input wire [2:0] iColour;
   input wire [6:0] iXY_Coord;
   input wire 	    iClock;
   output wire [7:0] oX;         // VGA pixel coordinates
   output wire [6:0] oY;

   output wire [2:0] oColour;     // VGA pixel colour (0-7)
   output wire 	     oPlot;       // Pixel draw enable
   output wire       oDone;       // goes high when finished drawing frame

	//MYCODE
  	wire ldX, ldY, ldC, clear, count; 
	wire [6:0] counter;
	wire [7:0] countX;
	wire [6:0] countY;
   //MYCODE
	
	control c0(iClock, iResetn, iPlotBox, iLoadX, iBlack, counter, countX, countY, ldX, ldY, clear, count, oPlot);
	datapath d0(iClock, iResetn, clear, oPlot, ldX, ldY, iXY_Coord, iColour, counter, countX, countY, oX, oY, oColour, oDone);
	
	
endmodule 

module datapath
#(parameter X_SCREEN_PIXELS = 8'd160,
parameter Y_SCREEN_PIXELS = 7'd120) (
	input iClock, iResetn, clearEnable, oPlot, 
	input loadX, loadY,
	input [6:0] iXY_Coord,
	input [2:0] iColor,
	
	output reg [6:0] counter,
	output reg [7:0] countX, 
	output reg [6:0] countY,
	output reg [7:0] outX,
	output reg [6:0] outY,
	output reg [2:0] outColor,
	output reg oDone
	);
	
	reg [7:0] X;
	reg [6:0] Y;
//ADD MORE DECLARATIONS IF NEEDED WHEN NEEDED START CODE NOW
	
	always @ (posedge iClock)
	begin
		if (~iResetn)
			begin
				outX = 8'b0;
				outY = 7'b0;
				X<=8'b0;
				Y<=7'b0;
				outColor <= 3'b0;
				counter = 7'b0;
				countX <= 8'b0;
				countY <= 7'b0;
				oDone = 0;
			end
		else
			begin
				if (loadX)
					begin
					X <= {iXY_Coord};
					outX = X;
					end
				
			if (loadY)
					begin
					Y = {iXY_Coord};
					outY = iXY_Coord;
					outColor = iColor;
					end
					
					
			if (oPlot)
					
						begin
							begin
						if (counter == 7'b1111111)
							begin
							counter = 0;
							outX = X + 2'b11;
							outY = Y + 2'b11;
							oDone = 1;
							end
						else if (counter != 7'b1111111)
							begin
							counter = counter + 1;
							oDone = 0;
							outX = X + counter [1:0];
							outY = Y + counter[6:2];
							end
						end
					end
			
			if (clearEnable) //do not else if this
					begin
						if (countX != 8'd160)
							begin
								countX <= countX + 1;
							end
						else if (countY != 7'd120 && countX == 8'd160)
							begin
								countX <= 8'b0;
								countY <= countY + 1;
							end
						else if (countY == 7'd120 && countX == 8'd160)
							begin
								countX <= 8'd0;
								countY <= 7'd0;
							end
					outX = countX;
					outY = countY;
					outColor <= iColor;
					
				end					
					
			end
		end

endmodule 

module control(
	input iClock, iResetn, iPlotBox, iLoadX, iBlack, 
	input [6:0] counter,
	input [7:0] countX,
	input [6:0] countY,
	
	output reg ldX, ldY, clear, count, oPlot //, oDone
	);
	
	parameter X_SCREEN_PIXELS = 8'd160;
   	parameter Y_SCREEN_PIXELS = 7'd120;
	
	reg [3:0] current_state, next_state;
	
	localparam S_LOAD_X      = 5'd0,
				  S_LOAD_X_WAIT = 5'd1,
				  S_LOAD_Y      = 5'd2,
				  S_LOAD_Y_WAIT = 5'd3,
				  S_DRAW        = 5'd4,
				  S_BLACK       = 5'd5;
				  //S_DONE        = 5'd6;		  
				  
	always@(*)
		begin: state_table
			case (current_state)
				S_LOAD_X: next_state = iLoadX ? S_LOAD_X_WAIT : S_LOAD_X;
				S_LOAD_X_WAIT: next_state = iLoadX ? S_LOAD_X_WAIT : S_LOAD_Y;
				S_LOAD_Y: next_state = iPlotBox ? S_LOAD_Y_WAIT : S_LOAD_Y;
				S_LOAD_Y_WAIT: next_state = iPlotBox ? S_LOAD_Y_WAIT : S_DRAW;
				S_DRAW: next_state = (counter == 128) ? S_LOAD_X : S_DRAW ;
				S_BLACK: next_state = (countX == 8'd159 & countY == 7'd119) ? S_LOAD_X : S_BLACK;
				//S_DONE: next_state = S_LOAD_X;
			endcase
		end
				  
	
	always @(*)
		begin: enable_signals
			
			ldX = 0;
			ldY = 0;
			clear = 0;
			count = 0;
			oPlot = 0;
			//oDone = 0;
			
			case (current_state)
				S_LOAD_X:
					begin
					ldX = 1;
					end
				S_LOAD_Y:
					begin
					ldY = 1;
					end
				S_DRAW:
					begin
					oPlot = 1;
					count = 1;
					end
				S_BLACK:
					begin
					oPlot = 1;
					clear = 1;
					end
				/*S_DONE:
					//oDone = 1;*/
			endcase
		end
		

	always@(posedge iClock)
		begin: state_FFs
			if (!iResetn)
				current_state <= S_LOAD_X;
			else if (iBlack)
				current_state <= S_BLACK;
				
			else 
				current_state <= next_state;
		end


endmodule 







module hex_decoder(c, display);
    input [3:0] c;
    output [6:0] display;

    assign display[0] = ~c[3] & ~c[2] & ~c[1] & c[0] |
		~c[3] & c[2] & ~c[1] & ~c[0] |
		c[3] & c[2] & ~c[1] & c[0] |
		c[3] & ~c[2] & c[1] & c[0];

    assign display[1] = c[3] & c[2] & ~c[0] |
		c[3] & c[1] & c[0] |
		c[2] & c[1] & ~c[0] |
		~c[3] & c[2] & ~c[1] & c[0];

    assign display[2] = c[3] & c[2] & ~c[0] |
		c[3] & c[2] & c[1] |
		~c[3] & ~c[2] & c[1] & ~c[0];

    assign display[3] = c[2] & c[1] & c[0] |
		~c[2] & ~c[1] & c[0] |
		~c[3] & c[2] & ~c[1] & ~c[0] |
		c[3] & ~c[2] & c[1] & ~c[0];

    assign display[4] = ~c[3] & c[0] |
		~c[3] & c[2] & ~c[1] |
		~c[2] & ~c[1] & c[0];

    assign display[5] = ~c[3] & ~c[2] & c[0] |
		~c[3] & ~c[2] & c[1] |
		~c[3] & c[1] & c[0] |
		c[3] & c[2] & ~c[1] & c[0];

    assign display[6] = ~c[3] & ~c[2] & ~c[1] |
		~c[3] & c[2] & c[1] & c[0] |
		c[3] & c[2] & ~c[1] & ~c[0];

endmodule

//Sw[7:0] data_in

//KEY[0] synchronous reset when pressed
//KEY[1] go signal

//LEDR displays result
//HEX0 & HEX1 also displays result

module ALU(Clock, Reset, Go, DataIn, DataResult, ResultValid,HEX1);
    input Clock;
    input Reset;
    input Go;
    input [3:0] DataIn;
    output [3:0] DataResult;
    output ResultValid;
	 output [3:0] HEX1;
    // lots of wires to connect our datapath and control
    wire ld_a, ld_b, ld_c, ld_x, ld_r;
    wire ld_alu_out;
    wire [1:0]  alu_select_a, alu_select_b;
    wire alu_op;
    controll C0(
	 
        .clk(Clock),
        .Reset(Reset),

        .go(Go),

        .ld_alu_out(ld_alu_out),
        .ld_x(ld_x),
        .ld_a(ld_a),
        .ld_b(ld_b),
        .ld_c(ld_c),
        .ld_r(ld_r),

        .alu_select_a(alu_select_a),
        .alu_select_b(alu_select_b),
        .alu_op(alu_op),
        .result_valid(ResultValid)
    );

    datapathh D0(
        .clk(Clock),
        .Reset(Reset),

        .ld_alu_out(ld_alu_out),
        .ld_x(ld_x),
        .ld_a(ld_a),
        .ld_b(ld_b),
        .ld_c(ld_c),
        .ld_r(ld_r),

        .alu_select_a(alu_select_a),
        .alu_select_b(alu_select_b),
        .alu_op(alu_op),

        .data_in(DataIn),
        .data_result(DataResult)
    );
	 assign HEX1 = DataResult;

 endmodule


module controll(
    input clk,
    input Reset,
    input go,

    output reg  ld_a, ld_b, ld_c, ld_x, ld_r,
    output reg  ld_alu_out,
    output reg [1:0]  alu_select_a, alu_select_b,
    output reg alu_op,
    output reg result_valid = 0
    );

    reg [14:0] current_state, next_state;

    localparam  S_LOAD_A        = 14'd0,
                S_LOAD_A_WAIT   = 14'd1,
                S_LOAD_B        = 14'd2,
                S_LOAD_B_WAIT   = 14'd3,
                S_LOAD_C        = 14'd4,
                S_LOAD_C_WAIT   = 14'd5,
                S_LOAD_X        = 14'd6,
                S_LOAD_X_WAIT   = 14'd7,
                S_CYCLE_0       = 14'd8,
                S_CYCLE_1       = 14'd9,
                S_CYCLE_2       = 14'd10,
					 S_CYCLE_3       = 14'd11,
					 S_CYCLE_4       = 14'd12,
					 S_CYCLE_5       = 14'd13;

    // Next state logic aka our state table
    always@(*)
    begin: state_table
            case (current_state)
                S_LOAD_A: next_state = go ? S_LOAD_A_WAIT : S_LOAD_A; // Loop in current state until value is input
                S_LOAD_A_WAIT: next_state = go ? S_LOAD_A_WAIT : S_LOAD_B; // Loop in current state until go signal goes low
                S_LOAD_B: next_state = go ? S_LOAD_B_WAIT : S_LOAD_B; // Loop in current state until value is input
                S_LOAD_B_WAIT: next_state = go ? S_LOAD_B_WAIT : S_LOAD_C; // Loop in current state until go signal goes low
                S_LOAD_C: next_state = go ? S_LOAD_C_WAIT : S_LOAD_C; // Loop in current state until value is input
                S_LOAD_C_WAIT: next_state = go ? S_LOAD_C_WAIT : S_LOAD_X; // Loop in current state until go signal goes low
                S_LOAD_X: next_state = go ? S_LOAD_X_WAIT : S_LOAD_X; // Loop in current state until value is input
                S_LOAD_X_WAIT: next_state = go ? S_LOAD_X_WAIT : S_CYCLE_0; // Loop in current state until go signal goes low
      S_CYCLE_0: next_state = S_CYCLE_2;
		S_CYCLE_2: next_state = S_CYCLE_3;
		S_CYCLE_3: next_state = S_CYCLE_4;
		S_CYCLE_4: next_state = S_CYCLE_5;
		S_CYCLE_5: next_state = S_CYCLE_1;
      S_CYCLE_1: next_state =  go ? S_CYCLE_1 : S_LOAD_A; // we will be done our two operations, start over after
            default:     next_state = S_LOAD_A;
        endcase
    end // state_table


    // Output logic aka all of our datapath control signals
    always @(*)
    begin: enable_signals
        // By default make all our signals 0
        ld_alu_out = 1'b0;
        ld_a = 1'b0;
        ld_b = 1'b0;
        ld_c = 1'b0;
        ld_x = 1'b0;
        ld_r = 1'b0;
        alu_select_a = 2'b0;
        alu_select_b = 2'b0;
        alu_op       = 1'b0;

        case (current_state)
            S_LOAD_A: begin
                ld_a = 1'b1;
                end
            S_LOAD_B: begin
                ld_b = 1'b1;
                end
            S_LOAD_C: begin
                ld_c = 1'b1;
                end
            S_LOAD_X: begin
                ld_x = 1'b1;
                end
            S_CYCLE_0: begin // Do A <- A * x
                ld_alu_out = 1'b1; ld_a = 1'b1; // store result back into A
                alu_select_a = 2'b00; // Select register A
                alu_select_b = 2'b11; // Also select register x
                alu_op = 1'b1; // Do multiply operation
            end
	    S_CYCLE_2: begin // Do A <- (A * x) * A
                ld_alu_out = 1'b1; ld_a = 1'b1; // store result back into A
                alu_select_a = 2'b00; // Select register A
                alu_select_b = 2'b00; // Also select register A
                alu_op = 1'b1; // Do multiply operation
            end
	    S_CYCLE_3: begin // Do B <- B * C
                ld_alu_out = 1'b1; ld_b = 1'b1; // store result back into B
                alu_select_a = 2'b01; // Select register B
                alu_select_b = 2'b10; // Also select register C
                alu_op = 1'b1; // Do multiply operation
            end
	    S_CYCLE_4: begin // Do A <- (A * x * A) + (B * C)
                ld_alu_out = 1'b1; ld_a = 1'b1; // store result back into A
                alu_select_a = 2'b00; // Select register A
                alu_select_b = 2'b01; // Also select register B
                alu_op = 1'b0; // Do Add operation
            end
            S_CYCLE_5: begin
                ld_r = 1'b1; // store result in result register
                alu_select_a = 2'b00; // Select register A
                alu_select_b = 2'b10; // Select register C
                alu_op = 1'b0; // Do Add operation
            end
				S_CYCLE_1: begin
					 result_valid = 1;
               end

        // default:    // don't need default since we already made sure all of our outputs were assigned a value at the start of the always block
        endcase
    end // enable_signals

    // current_state registers
    always@(posedge clk)
    begin: state_FFs
        if(Reset)
            current_state <= S_LOAD_A;
        else
            current_state <= next_state;
    end // state_FFS
endmodule

module datapathh(
    input clk,
    input Reset,
    input [3:0] data_in,
    input ld_alu_out,
    input ld_x, ld_a, ld_b, ld_c,
    input ld_r,
    input alu_op,
    input [1:0] alu_select_a, alu_select_b,
    output reg [3:0] data_result
    );

    // input registers
    reg [3:0] a, b, c, x;

    // output of the alu
    reg [3:0] alu_out;
    // alu input muxes
    reg [3:0] alu_a, alu_b;

    // Registers a, b, c, x with respective input logic
    always@(posedge clk) begin
        if(Reset) begin
            a <= 4'b0;
            b <= 4'b0;
            c <= 4'b0;
            x <= 4'b0;
        end
        else begin
            if(ld_a)
                a <= ld_alu_out ? alu_out : data_in; // load alu_out if load_alu_out signal is high, otherwise load from data_in
            if(ld_b)
                b <= ld_alu_out ? alu_out : data_in; // load alu_out if load_alu_out signal is high, otherwise load from data_in
            if(ld_x)
                x <= data_in;

            if(ld_c)
                c <= data_in;
        end
    end

    // Output result register
    always@(posedge clk) begin
        if(Reset) begin
            data_result <= 4'b0;
        end
        else
            if(ld_r)
                data_result <= alu_out;
    end

    // The ALU input multiplexers
    always @(*)
    begin
        case (alu_select_a)
            2'd0:
                alu_a = a;
            2'd1:
                alu_a = b;
            2'd2:
                alu_a = c;
            2'd3:
                alu_a = x;
            default: alu_a = 4'b0;
        endcase

        case (alu_select_b)
            2'd0:
                alu_b = a;
            2'd1:
                alu_b = b;
            2'd2:
                alu_b = c;
            2'd3:
                alu_b = x;
            default: alu_b = 4'b0;
        endcase
    end

    // The ALU
    always @(*)
    begin : ALU
        // alu
        case (alu_op)
            0: begin
                   alu_out = alu_a + alu_b; //performs addition
               end
            1: begin
                   alu_out = alu_a * alu_b; //performs multiplication
               end
            default: alu_out = 4'b0;
        endcase
    end

endmodule
