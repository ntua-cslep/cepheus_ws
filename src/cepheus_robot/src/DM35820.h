#include <dm7820_library.h>

/**
 * @brief
 *      Incremental encoder input modes
 */

	enum _dm7820_pwm_output_mode {
    //Single ended + positive output pin
		DM7820_PWM_OUTPUT_POSITIVE_OUT = 0,
    //Single ended - negative output pin
		DM7820_PWM_OUTPUT_NEGATIVE_OUT
    //differential both signal enabled
  	DM7820_PWM_OUTPUT_DIFFERENTIAL_OUT
	};
	typedef enum _dm7820_pwm_output_mode dm7820_pwm_output_mode;

class DM35820_card {
public:
	DM35820(uint32_t minor_number) : base_minor_number(minor_number) //initialize const 
	{
		dm7820_status = DM7820_General_Open_Board(base_minor_number, &board);
	  DM7820_Return_Status(dm7820_status, "Opening Device, base card");
	  for(int i=0; i<2;i++) {
	  	for(int j=0; j<2;j++) {
	  		encoder_value[i][j] = 0;
			  encoder_ovf[i][j] = 0;
	  	}
	  }
	}

	void initEncoder (dm7820_incenc_encoder      encoder, 
          					dm7820_incenc_channel_mode channel_mode, //		DM7820_INCENC_CHANNEL_INDEPENDENT, DM7820_INCENC_CHANNEL_JOINED
          					dm7820_incenc_input_mode   input_mode)   //   DM7820_INCENC_INPUT_SINGLE_ENDED, DM7820_INCENC_INPUT_DIFFERENTIAL
	{
		encoder_channel_mode[encoder] = channel_mode;

		dm7820_status = DM7820_IncEnc_Enable(board, encoder, 0x00); 
	  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

	 	if(channel_mode == DM7820_INCENC_CHANNEL_INDEPENDENT)
	 	{
		  if(input_mode == DM7820_INCENC_INPUT_DIFFERENTIAL)
		  {
			  //Set encoder port 0011 1111 0011 1111 bit to input which enables incremental encoder pins
			  dm7820_status = DM7820_StdIO_Set_IO_Mode(board, encoder, 0x3F3F, DM7820_STDIO_MODE_INPUT);
			  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");
			}
			else if(input_mode == DM7820_INCENC_INPUT_SINGLE_ENDED) 
			{
			  //Set encoder port 0001 0101 0001 0101 bit to input which enables incremental encoder pins
			  dm7820_status = DM7820_StdIO_Set_IO_Mode(board, encoder, 0x1515, DM7820_STDIO_MODE_INPUT);
			  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");
			}
		}
		else if(channel_mode == DM7820_INCENC_CHANNEL_JOINED)
	 	{
		  if(input_mode == DM7820_INCENC_INPUT_DIFFERENTIAL)
		  {
			  //Set encoder port 0000 0000 0011 1111 bit to input which enables incremental encoder pins
			  dm7820_status = DM7820_StdIO_Set_IO_Mode(board, encoder, 0x003F, DM7820_STDIO_MODE_INPUT);
			  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");
			}
			else if(input_mode == DM7820_INCENC_INPUT_SINGLE_ENDED) 
			{
			  //Set encoder port 0000 0000 0001 0101 bit to input which enables incremental encoder pins
			  dm7820_status = DM7820_StdIO_Set_IO_Mode(board, encoder, 0x0015, DM7820_STDIO_MODE_INPUT);
			  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");
			}
		}

	  //Set master clock to 25 MHz clock
	  //Incremental encoder initialization
	  dm7820_status = DM7820_IncEnc_Set_Master(board, encoder, DM7820_INCENC_MASTER_25_MHZ);
	  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Master()");

	  //Disable value register hold 0
	  dm7820_status = DM7820_IncEnc_Enable_Hold(board, encoder, 0x00);
	  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable_Hold()");

	  
	   //* Configure the incremental encoder as follows: 1) disable all up
	   //* transitions from modifying the counter, 2) set single-ended input mode,
	   //* 3) disable the input filter, 4) set channels A and B to operate
	   //* independently of each other, and 5) disable index input
	   
	  DM7820_INCENC_RESET_PHASE_FILTER(phase_filter_0);
	  dm7820_status = DM7820_IncEnc_Configure(board, encoder,
	            phase_filter_0,
	            input_mode,
	            0x00,
	            channel_mode,
	            0x00);
	  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Configure()");

	  //Set initial value for channel A counter
	  dm7820_status = DM7820_IncEnc_Set_Independent_Value(board,
	                  encoder,
	                  DM7820_INCENC_CHANNEL_A,
	                  encoder_init_value);
	  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Independent_Value_0_A");

		if(channel_mode == DM7820_INCENC_CHANNEL_INDEPENDENT)
		{
		  //Set initial value for channel B counter
		  dm7820_status = DM7820_IncEnc_Set_Independent_Value(board,
		                  encoder,
		                  DM7820_INCENC_CHANNEL_B,
		                  encoder_init_value);
		  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Independent_Value_0_B");
		}

	  //incremental encoder 0 enable again
	  dm7820_status = DM7820_IncEnc_Enable(board, encoder, 0xFF);
	  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

	  //Clear channel negative and positive rollover status flag without checking its state just reading it
	  dm7820_status = DM7820_IncEnc_Get_Status(board, encoder, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
	  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");
	  dm7820_status = DM7820_IncEnc_Get_Status(board, encoder, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
	  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");

	  dm7820_status = DM7820_IncEnc_Get_Status(board, encoder, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
	  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");
	  dm7820_status = DM7820_IncEnc_Get_Status(board, encoder, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
	  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");
	}

	int32_t readIndependentEncoder (dm7820_incenc_encoder      encoder, 
          										    dm7820_incenc_channel      channel)
	{
  	dm7820_status = DM7820_IncEnc_Get_Independent_Value(board, encoder, channel, 
  		&encoder_register_independent_buffer);
  	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");
	  //handle overflow
	  if(channel == DM7820_INCENC_CHANNEL_A)
	  {
		  dm7820_status = DM7820_IncEnc_Get_Status(board, encoder, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
		  DM7820_Return_Status(dm7820_status, "Read positive overflow channel A");
		  if (encoder_status) 
		    encoder_ovf[encoder][channel]++;

		  dm7820_status = DM7820_IncEnc_Get_Status(board, encoder, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
		  DM7820_Return_Status(dm7820_status, "Read negative overflow channel A");
		  if (encoder_status) 
		    encoder_ovf[encoder][channel]--;

		  dm7820_status = DM7820_IncEnc_Get_Status(board, encoder, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
		  DM7820_Return_Status(dm7820_status, "Positive overflow clear channel 0A");
		  if (encoder_status) 
		    error(EXIT_FAILURE, 0, "ERROR: Channel A positive overflow status not cleared");

		  dm7820_status = DM7820_IncEnc_Get_Status(board, encoder, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
		  DM7820_Return_Status(dm7820_status, "negative overflow clear channel 0A");
		  if (encoder_status)  
		    error(EXIT_FAILURE, 0, "ERROR: Channel A negative overflow status not cleared");
		}
		else 
		{
		  dm7820_status = DM7820_IncEnc_Get_Status(board, encoder, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
		  DM7820_Return_Status(dm7820_status, "Read positive overflow channel B");
		  if (encoder_status) 
		    encoder_ovf[encoder][channel]++;

		  dm7820_status = DM7820_IncEnc_Get_Status(board, encoder, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
		  DM7820_Return_Status(dm7820_status, "Read negative overflow channel B");
		  if (encoder_status) 
		    encoder_ovf[encoder][channel]--;

		  dm7820_status = DM7820_IncEnc_Get_Status(board, encoder, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
		  DM7820_Return_Status(dm7820_status, "Positive overflow clear channel B");
		  if (encoder_status) 
		    error(EXIT_FAILURE, 0, "ERROR: Channel A positive overflow status not cleared");

		  dm7820_status = DM7820_IncEnc_Get_Status(board, encoder, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
		  DM7820_Return_Status(dm7820_status, "negative overflow clear channel B");
		  if (encoder_status)  
		    error(EXIT_FAILURE, 0, "ERROR: Channel A negative overflow status not cleared");
		}

	  encoder_value[encoder][channel] = (int32_t)encoder_register_independent_buffer + 65535*encoder_ovf[encoder][channel];
	  return encoder_value[encoder][channel];
	}

	int32_t readJoinedEncoder (dm7820_incenc_encoder encoder)
	{
		dm7820_status = DM7820_IncEnc_Get_Joined_Value(board, encoder, 
  		&encoder_register_joined_buffer);
  	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Joined_Value()");

  	encoder_value[encoder][0] = (int32_t)encoder_register_independent_buffer;
  	return encoder_value[encoder][0];
	}

	void setIndependentEncoder(dm7820_incenc_encoder      encoder, 
          									 dm7820_incenc_channel      channel,
          									 uint16_t                   value)
	{
		dm7820_status = DM7820_IncEnc_Set_Independent_Value(board, encoder, channel, value);
  	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Independent_Value()");
  	encoder_ovf[encoder][channel] = 0;
  }

	void setJoinedEncoder(dm7820_incenc_encoder      encoder, 
    									  uint32_t                   value)
	{
		dm7820_status = DM7820_IncEnc_Set_Joined_Value(board, encoder, value);
  	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Joined_Value()");
  }

	void initPWM (dm7820_pwm_modulator            pwm_modulator,
								dm7820_pwm_period_master_clock  period_master_clock,
								dm7820_pwm_width_master_clock   period_width_clock,
								dm7820_pwm_output_mode          pwm_ch_0 = 0,
								dm7820_pwm_output_mode          pwm_ch_1 = 0,
								dm7820_pwm_output_mode          pwm_ch_2 = 0,
								dm7820_pwm_output_mode          pwm_ch_3 = 0)
	{
		uint16_t portb_pwm_enabled_pins = 0;
		
		//shift 2 bits for every channel
		uint16_t channel_bits = 0;
		channel_bits |= pwm_ch_0; 
		channel_bits |= pwm_ch_1<<2;
		channel_bits |= pwm_ch_2<<4;
		channel_bits |= pwm_ch_3<<6;

  	uint16_t portb_pwm_enabled_pins = channel_bits<<(pwm_modulator*8)

		//Disable pulse width modulator to put them into a known state.
		//pulse width modulator should be disabled before programming it
		dm7820_status = DM7820_PWM_Enable(board, pwm_modulator, 0x00); //setting x00 is disable
		DM7820_Return_Status(dm7820_status, "DM7820_PWM_Disable()");
		
		dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_2, portb_pwm_enabled_pins, DM7820_STDIO_MODE_PER_OUT);
		DM7820_Return_Status(dm7820_status, "DM7820_Set port2 (0,2,4,6) bits to peripheral output");

		dm7820_status = DM7820_StdIO_Set_Periph_Mode(board, DM7820_STDIO_PORT_2, portb_pwm_enabled_pins, DM7820_STDIO_PERIPH_PWM);
		DM7820_Return_Status(dm7820_status, "Set port2 (0,2,4,6) bits to PWM output");

		//Set period master clock to 25 MHz clock
		dm7820_status = DM7820_PWM_Set_Period_Master(board, pwm_modulator, period_master_clock);
		DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period_Master()");

		//Set width master clock to 25 MHz clock
		dm7820_status = DM7820_PWM_Set_Width_Master(board, pwm_modulator, period_width_clock);
		DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width_Master()");

		//zero out all pwms
		dm7820_status = DM7820_PWM_Set_Width(board, pwm_modulator, DM7820_PWM_OUTPUT_A, 0);
		DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
		dm7820_status = DM7820_PWM_Set_Width(board, pwm_modulator, DM7820_PWM_OUTPUT_B, 0);
		DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
		dm7820_status = DM7820_PWM_Set_Width(board, pwm_modulator, DM7820_PWM_OUTPUT_C, 0);
		DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
		dm7820_status = DM7820_PWM_Set_Width(board, pwm_modulator, DM7820_PWM_OUTPUT_D, 0);
		DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

		//enable PWM
		dm7820_status = DM7820_PWM_Enable(board, pwm_modulator, 0xFF);
		DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");
	}

	void pwmWidth(dm7820_pwm_modulator pwm_modulator,
										dm7820_pwm_output    pwm_channel,
										uint16_t             value)
	{
		pwm_width[pwm_modulator][pwm_channel] = value;
	}

	void pwmPeriod(dm7820_pwm_modulator pwm_modulator,
										dm7820_pwm_output    pwm_channel,
										uint16_t             value)
	{
		pwm_period[pwm_modulator][pwm_channel] = value;
	}

	void setPwmWidths(dm7820_pwm_modulator pwm_modulator)
	{
		dm7820_status = DM7820_PWM_Set_Width(board, pwm_modulator, 0, pwm_width[pwm_modulator][0]);
		DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
		dm7820_status = DM7820_PWM_Set_Width(board, pwm_modulator, 1, pwm_width[pwm_modulator][1]);
		DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
		dm7820_status = DM7820_PWM_Set_Width(board, pwm_modulator, 2, pwm_width[pwm_modulator][2]);
		DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
		dm7820_status = DM7820_PWM_Set_Width(board, pwm_modulator, 3, pwm_width[pwm_modulator][3]);
		DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
	}

	void setPwmPeriods(dm7820_pwm_modulator pwm_modulator)
	{
		dm7820_status = DM7820_PWM_Set_Period(board, pwm_modulator, 0, pwm_period[pwm_modulator][0]);
		DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period()");
		dm7820_status = DM7820_PWM_Set_Period(board, pwm_modulator, 1, pwm_period[pwm_modulator][1]);
		DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period()");
		dm7820_status = DM7820_PWM_Set_Period(board, pwm_modulator, 2, pwm_period[pwm_modulator][2]);
		DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period()");
		dm7820_status = DM7820_PWM_Set_Period(board, pwm_modulator, 3, pwm_period[pwm_modulator][3]);
		DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period()");
	}

private:
	const uint32_t base_minor_number;
	const DM7820_Board_Descriptor *board; //this the board descriptor
	dm7820_incenc_phase_filter phase_filter_0;
	dm7820_incenc_phase_filter phase_filter_1;
	uint8_t encoder_status;
	uint16_t pwm_width[2][4];
	uint16_t pwm_period[2][4];

	dm7820_incenc_channel_mode encoder_channel_mode[2];
	uint16_t encoder_register_independent_buffer;
	uint32_t encoder_register_joined_buffer;
  uint8_t encoder_status;
	int32_t encoder_value[2][2];
	int32_t encoder_ovf[2][2];
}
