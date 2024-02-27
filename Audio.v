module Audio_Example (
        // Inputs
        CLOCK_50,
        key1,
		  keybIn,

        AUD_ADCDAT, 
        // Bidirectionals
        AUD_BCLK,
        AUD_ADCLRCK,
        AUD_DACLRCK,

        FPGA_I2C_SDAT,

        // Outputs
        AUD_XCK,
        AUD_DACDAT,

        FPGA_I2C_SCLK,
		  SW
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/


/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input                                CLOCK_50;
input                key1;
input                [3:0]        SW;
input keybIn;
//input [3:0] DR;


input                                AUD_ADCDAT;

// Bidirectionals
inout                                AUD_BCLK;    
inout                                AUD_ADCLRCK;
inout                                AUD_DACLRCK;

inout                                FPGA_I2C_SDAT;


// Outputs
output                                AUD_XCK;
output                                AUD_DACDAT;

output                                FPGA_I2C_SCLK;

/*****************************************************************************
 *                 Internal Wires and Registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire                                audio_in_available;
wire                [31:0]        left_channel_audio_in;
wire                [31:0]        right_channel_audio_in;
wire                                read_audio_in;

wire                                audio_out_allowed;
wire                [31:0]        left_channel_audio_out;
wire                [31:0]        right_channel_audio_out;
wire                                write_audio_out;

//wire keyboardOutput;
wire [31:0] previewSound;
//wire keyboardInput;
wire press;
wire rst;

// Internal Registers

reg [18:0] delay_cnt;
wire [18:0] delay;

reg snd;

//ps2audio keyaud(.CLOCK(CLOCK_50), .PS2_CLK(PS2_CLK), .PS2_DAT(PS2_DAT), .keyInput(keyboardInput), .HEX4(HEX4), .HEX5(HEX5), .isPressed(press) ); 

////////////////////////////////////////////////////////////////////////////////

always @(posedge CLOCK_50)
        if(delay_cnt == delay) begin
                delay_cnt <= 0;
                snd <= !snd;
        end else delay_cnt <= delay_cnt + 1;

///////////////////////////////////////////////////////////////////////////////
//                            Combinational Logic                            //
///////////////////////////////////////////////////////////////////////////////

assign delay = {SW[3:0], 16'h7fff};
//assign delay1 = {SW[9:6], 16'h7fff};
assign rst = key1 | keybIn;

wire [31:0] sound = (SW[3:0] == 0) ? 0 : snd ? 32'd10000000 : -32'd10000000;
//wire [31:0] sound1 = (SW[9:6] == 0) ? 0 : snd ? 32'd10000000 : -32'd10000000;
//wire [31:0] sound = (SW == 0) ? 0 : snd ? 32'd10000000 : -32'd10000000;
//wire [31:0] sound = (SW == 0) ? 0 : snd ? 32'd10000000 : -32'd10000000;
//wire [31:0] sound = (SW == 0) ? 0 : snd ? 32'd10000000 : -32'd10000000;

//assign keyboardOutput = (SW == 0) ? 0 : 1;

assign previewSound = (keybIn == 0) ? 0 :  sound; //integrate this with the ps2 controller 


assign read_audio_in                        = audio_in_available & audio_out_allowed;

assign left_channel_audio_out        = left_channel_audio_in+sound+previewSound;
assign right_channel_audio_out        = right_channel_audio_in+sound+previewSound;
assign write_audio_out                        = audio_in_available & audio_out_allowed;


/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/



Audio_Controller Audio_Controller (
        // Inputs
        .CLOCK_50                                                (CLOCK_50),
        .reset                                                (~rst),

        .clear_audio_in_memory                (),
        .read_audio_in                                (read_audio_in),
        
        .clear_audio_out_memory                (),
        .left_channel_audio_out                (left_channel_audio_out),
        .right_channel_audio_out        (right_channel_audio_out),
        .write_audio_out                        (write_audio_out),

        .AUD_ADCDAT                                        (AUD_ADCDAT),

        // Bidirectionals
        .AUD_BCLK                                        (AUD_BCLK),
        .AUD_ADCLRCK                                (AUD_ADCLRCK),
        .AUD_DACLRCK                                (AUD_DACLRCK),


        // Outputs
        .audio_in_available                        (audio_in_available),
        .left_channel_audio_in                (left_channel_audio_in),
        .right_channel_audio_in                (right_channel_audio_in),

        .audio_out_allowed                        (audio_out_allowed),

        .AUD_XCK                                        (AUD_XCK),
        .AUD_DACDAT                                        (AUD_DACDAT)

);

avconf #(.USE_MIC_INPUT(1)) avc (
        .FPGA_I2C_SCLK                                        (FPGA_I2C_SCLK),
        .FPGA_I2C_SDAT                                        (FPGA_I2C_SDAT),
        .CLOCK_50                                        (CLOCK_50),
        .reset                                                (~rst)
);

endmodule


