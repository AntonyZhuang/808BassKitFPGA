module ps2audio (
	// Inputs
	CLOCK,

	// Bidirectionals
	PS2_CLK,
	PS2_DAT,
	
	// Outputs
	keyInput,
	HEX4,
	HEX5,
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/


/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/

// Inputs
input				CLOCK;

// Bidirectionals
inout				PS2_CLK;
inout				PS2_DAT;

// Outputs
output keyInput;

output		[6:0]	HEX4;
output		[6:0]	HEX5;


/*****************************************************************************
 *                 Internal Wires and Registers Declarations                 *
 *****************************************************************************/

// Internal Wires
wire		[7:0]	ps2_key_data;
wire				ps2_key_pressed;

// Internal Registers
reg keyboardIn;


assign keyInput = keyboardIn;
// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential Logic                              *
 *****************************************************************************/



always @(posedge ps2_key_pressed)
	begin
		if ((ps2_key_data == 8'h3A) || (ps2_key_data == 8'hF0))
				keyboardIn = 0;
		else 
			keyboardIn = 1;
	end


/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

PS2_Controller PS2 (
	// Inputs
	.CLOCK_50				(CLOCK),
	.reset				(0), //MAKE 0 we used up all the keys!!!!!

	// Bidirectionals
	.PS2_CLK			(PS2_CLK),
 	.PS2_DAT			(PS2_DAT),

	// Outputs
	.received_data		(ps2_key_data),
	.received_data_en	(ps2_key_pressed)
);

Hexadecimal_To_Seven_Segment Segment0 (
	// Inputs
	.hex_number			(ps2_key_data[7:4]), //ps2_key_data --maybe use the internal register in initial demo?????

	// Bidirectional

	// Outputs
	.seven_seg_display	(HEX5)
);

Hexadecimal_To_Seven_Segment Segment1 (
	// Inputs
	.hex_number			(ps2_key_data[3:0]), //ps2_key_data --maybe use the internal register in initial demo?????

	// Bidirectional

	// Outputs
	.seven_seg_display	(HEX4)
);


endmodule
