module game
	(	
		// input from board
		KEY, SW, CLOCK_50,//	On Board 50 MHz
		// output
		LEDR, LEDG, HEX7, HEX6, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0,
		// The ports below are for the VGA output
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,					//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B   							//	VGA Blue[9:0]
	);
	
	output			VGA_CLK;   				//	VGA Clockcontrol
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;			//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
	

	input CLOCK_50;
	input [3:0]KEY;
	input [17:0] SW;
	
	output [17:0] LEDR;
	output [7:0] LEDG;
	output [6:0] HEX0;
	output [6:0] HEX1;
	output [6:0] HEX2;
	output [6:0] HEX3;
	output [6:0] HEX4;
	output [6:0] HEX5;
	output [6:0] HEX6;
	output [6:0] HEX7;
	
	// wires for main game
	wire input_signal, to_display, to_checker, to_get, waiting, loading, random_sequence, finish, determining, diff_level;
	wire [3:0] user_count;
	wire [23:0]random_number, rand;
	wire [7:0] scoring, highscore;
	
	// wires for vga display
	wire s_signal, b_signal, c_signal, load_b, load_c, draw, restart_signal, deter, starting;
	wire writeEn;

	// Create the colour, x, y and writeEn wires that are inputs to the controller.
	wire [2:0] colour;
	wire [2:0] c_out;
	wire [7:0] x;
	wire [6:0] y;
	
	vga_adapter VGA(
			.resetn(SW[17]),
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
		defparam VGA.BACKGROUND_IMAGE = "background.mif";
		
	// control for main game
	control control(.clk(CLOCK_50),
		.resetn(SW[17]),
		.go(SW[16]),
		.to_input(input_signal),
		.rand_seq(random_sequence),
		.display(to_display),
		.checker(to_checker),
		.get(to_get),
		.w(waiting),
		.endgame(finish),
		.determine(determining),
		.difficulty(diff_level),
		.restart(restart_signal),
		.det(deter),
		.start(starting));
	
	// gives the random sequence
	RandomSeq rng(.square_number(3'b101),
		      .clk(CLOCK_50),
		      .go(input_signal),
		      .sequence(random_number),
		      .load(loading));
		 
	// control for vga display	 
	vga_control (.done_b(b_signal),
				 .done_c(c_signal),
				 .plot_s(s_signal),
				 .resetn(SW[17]),
				 .clock(CLOCK_50),
				 .ld_b(load_b),
				 .ld_c(load_c), 
				 .writeEn(writeEn),
				 .drawEn(draw),
				 .reset_start(reset_s),
				 .go(starting),
				 .done_start(done_signal),
				 .det(deter),
				 .determine(determining));	
	
	// datapath for vga display
	vga_datapath(.ld_b(load_b),
				 .ld_c(load_c),
				 .drawEn(draw),
				 .colour(colour),
				 .resetn(SW[17]),
				 .clock(CLOCK_50),
				 .x_out(x),
				 .y_out(y),
				 .c(c_out),
				 .done_b(b_signal),
				 .done_c(c_signal),
				 .reset_start(reset_s),
				 .done_start(done_signal));

	// datapath for main game
	datapath(.clk(CLOCK_50),
		 .rand_seq(random_sequence),
		 .display(to_display),
		 .checker(to_checker),
		 .get(to_get),
		 .determine(determining),
		 .reset_n(SW[17]),
		 .load(loading),
		 .b1(SW[1]),
		 .b2(!KEY[3]),
		 .b3(!KEY[2]),
		 .b4(!KEY[1]),
		 .b5(!KEY[0]),
		 .w(waiting),
		 .endgame(finish),
		 .rand_num(random_number),
		 .to_input(input_signal),
		 .b1out(LEDR[4]),
		 .b2out(LEDR[3]),
		 .b3out(LEDR[2]),
		 .b4out(LEDR[1]),
		 .b5out(LEDR[0]),
		 .user_num(user_count),
		 .difficulty(diff_level),
		 .score(scoring),
		 .easy(SW[10]),
		 .med(SW[11]),
		 .hard(SW[12]),
		 .plot_s(s_signal),
		 .color(colour),
		 .restart(restart_signal),
		 .high_score(highscore),
		 .det(deter),
		 .random_sequence(rand));
		 
	// display for user sequence input count
	hex_display h3(.IN(user_count[2:0]), .OUT(HEX6[6:0]));
	hex_display h2(.IN(4'hC), .OUT(HEX7[6:0]));
	
	// display for score for current round
	hex_display h1(.IN(scoring[3:0]), .OUT(HEX4[6:0]));
	hex_display h6(.IN(scoring[7:4]), .OUT(HEX5[6:0]));
	
	// display for high score of game overall
	hex_display h10(.IN(4'hF), .OUT(HEX3[6:0]));
	hex_display h11(.IN(4'h1), .OUT(HEX2[6:0]));
	hex_display h7(.IN(highscore[3:0]), .OUT(HEX0[6:0]));
	hex_display h8(.IN(highscore[7:4]), .OUT(HEX1[6:0]));

	

endmodule


module vga_datapath(ld_b, ld_c, drawEn, colour, resetn, clock, x_out, y_out, c, done_b, done_c, reset_start, done_start);

	input ld_b, ld_c, drawEn, resetn, clock, reset_start;
	input [2:0] colour;
	output [7:0] x_out;
	output [6:0] y_out;
	output reg [2:0] c;
	output reg done_b, done_c, done_start; 

	reg [7:0] x;
	reg [6:0] y;
	reg [4:0] counter_x;
	reg [4:0] counter_y;
	reg [2:0] reset_counter;
	
	initial begin
		done_b <= 1'b0;
		done_c <= 1'b0;
		reset_counter <= 3'b0;
		done_start <= 1'b0;
		y <= 7'b0101010;
	end
	
	always @(*)
	begin
		if (reset_start)
				done_start <= 1'b1;
		// load the colour of the box that corresponds to the sequence given
		else if (ld_c)
		begin
		y <= 7'b0101010;
			case (colour)
				3'b001: begin
					c <= 3'b001;
					x <= 8'b00001100;
					end
				3'b010: begin
					c <= 3'b010;
					x <= 8'b00101010;
					end
				3'b011: begin
					c <= 3'b011; 
					x <= 8'b01001000; 
					end
				3'b100: begin 
					c <= 3'b100;
					x <= 8'b01100110;
					end
				3'b101: begin
					c <= 3'b101;
					x <= 8'b10000011;
					end
				default:
					c <= 3'b111;
			endcase
			done_c <= 1'b1;
			done_b <= 1'b0;
		end
		// load a black box where the previous coloured box was drawn
		else if (ld_b) begin
			c <= 3'b000;
			y <= 7'b0101010;
			done_b <= 1'b1;
			done_c <= 1'b0;
		// reset the colour when reset is pressed
		end else if (!resetn)
		begin
		c <= 3'b0;
		end
	end
	
	always @(posedge clock)
	begin
		if (!resetn)
		begin
			counter_x <= 5'b00000;
			counter_y <= 5'b00000;
		end
		// draw a 18 X 18 box with the given colour and x and y coordinates
		else if (drawEn)
		begin
			if (counter_x == 5'b10001 && counter_y == 5'b10010)
			begin
				counter_y <= 5'b00000;
				counter_x <= 5'b00000;
			end
			else if (counter_x == 5'b10001)
			begin
				counter_y <= counter_y + 1'b1;
				counter_x <= 5'b00000;
			end
			else
				counter_x <= counter_x + 1'b1;
		end
	end

	assign x_out = x + counter_x;
	assign y_out = y + counter_y;
endmodule

module vga_control(done_b, done_c, plot_s, resetn, clock, ld_b, ld_c, writeEn, drawEn, reset_start, go, done_start, det, determine);

	input plot_s, done_b, done_c, resetn, clock, done_c, go, done_start, det, determine;
	output reg ld_c, ld_b, writeEn, drawEn, reset_start;
	
	reg [2:0] curr_state, next_state;

	// state parameters for vga display
	localparam			START = 3'd0,
					WAIT = 3'd1,
					LOAD_C = 3'd2,
					DRAW = 3'd3,
					LOAD_B = 3'd4,
					DRAW_B = 3'd5,
					RESET_START = 3'd6;			

	// state table for vga display
	always @(*)
	begin: state_table
		case (curr_state)
			START: next_state = go ? START : RESET_START; // stay in the start state until game starts
			RESET_START: next_state = done_start ? WAIT : RESET_START; // reset start state values and move to the wait state
			WAIT: next_state = plot_s ? LOAD_C : WAIT; // wait until colour input is given, then move to the load_c state
			LOAD_C: next_state = done_c ? DRAW : LOAD_C; // load the colour given and go to draw state
			DRAW: next_state = plot_s ? DRAW : LOAD_B; // draw the square and move to the load_b state
			LOAD_B: next_state = done_b ? DRAW_B : LOAD_B; // load black colour and move to draw_b
			DRAW_B: next_state = plot_s ? LOAD_C : DRAW_B; // draw black box on previous colour box and move to wait state
		endcase
	end
	
	// output logic for vga display
	always @(*)
	begin: part2_signals
		ld_b = 1'b0;
		ld_c = 1'b0;
		writeEn = 1'b0;
		drawEn = 1'b0;
		reset_start = 1'b0;
		case (curr_state)
			RESET_START: begin
				reset_start = 1'b1;
			end
			LOAD_C: begin
				ld_c <= 1'b1;
				end
			LOAD_B: begin
				ld_b <= 1'b1;
				end
			DRAW_B: begin
				writeEn <= 1'b1;
				drawEn <= 1'b1;
				end
			DRAW: begin
				writeEn <= 1'b1;
				drawEn <= 1'b1;
				end
		endcase
	end
	
	always@(posedge clock)
	begin: state_FFs
        if(!resetn)
            curr_state <= START;
        else
            curr_state <= next_state;
    end
endmodule

module control(clk, resetn, go, to_input, rand_seq, display, checker, get, w, endgame, determine, difficulty, to_wait, restart, det, start);

    input clk;
    input resetn;
    input go;
    input to_input;
    input determine;
    input to_wait, restart;

    output reg rand_seq, display, checker, get, w, endgame, difficulty, det, start;

    reg [2:0] current_state, next_state;
	 
    // state parameters for main game
    localparam  START_STATE       = 5'd0,
		DIFFICULTY_MODE	  = 5'd1,
                RANDOM_SEQUENCE   = 5'd2,
                DISPLAY_STATE     = 5'd3,
                WAIT_INPUT   	  = 5'd4,
                GET_INPUT         = 5'd5,
                CHECK_INPUT       = 5'd6,
		DETERMINE	  = 5'd7,
                ENDGAME           = 5'd8;
    
    // state logic for main game
    always@(*)
    begin: state_table 
            case (current_state)
                START_STATE: next_state = go ? DIFFICULTY_MODE : START_STATE; // loop in start state until game starts
		DIFFICULTY_MODE: next_state = to_input ? DIFFICULTY_MODE : RANDOM_SEQUENCE; // keep track of difficulty chosen and move to random_sequence
                RANDOM_SEQUENCE: next_state = to_input ? DISPLAY_STATE : RANDOM_SEQUENCE; // generate random sequence and move on to display state
                DISPLAY_STATE: next_state = to_input ? DISPLAY_STATE : WAIT_INPUT; // display the random sequence generated
                WAIT_INPUT: next_state = to_input ? GET_INPUT : WAIT_INPUT; // wait until an input is given and go to get state
                GET_INPUT: next_state = to_input ? GET_INPUT : CHECK_INPUT; // get the input given and go to check state
                CHECK_INPUT: next_state = to_input ? DETERMINE : CHECK_INPUT; // check the input, and go to determine state
		DETERMINE: next_state = determine ? RANDOM_SEQUENCE : ENDGAME; // if input is correct do to random_sequence state, if its wrong, go to endgame state
                ENDGAME: next_state = restart ? ENDGAME : START_STATE ; // end the game and go to the start state

            default: next_state = ENDGAME;
        endcase
    end
   
   

    // Output logic for main game
    always @(*)
    begin: enable_signals
        rand_seq = 1'b0;
        display = 1'b0;
        get = 1'b0;
        checker = 1'b0;
	w = 1'b0;
	endgame = 1'b0;
	difficulty = 1'b0;
	det = 1'b0;
	start = 1'b0;
        
        case (current_state)
		START_STATE: begin
			start <= 1'b1;
		end
		DIFFICULTY_MODE: begin
			difficulty = 1'b1;
		end
		RANDOM_SEQUENCE: begin
			rand_seq <= 1'b1;
		end
		DISPLAY_STATE: begin
			display <= 1'b1;
		end
		WAIT_INPUT: begin
			w = 1'b1;
		end
		GET_INPUT: begin
			get = 1'b1;
		end
		CHECK_INPUT: begin
			checker = 1'b1;
		end
		DETERMINE: begin
			det <= 1'b1;
		end
		ENDGAME: begin
			endgame = 1'b1;
		end
        endcase
    end
   
    always@(posedge clk)
    begin: state_FFs
        if(!resetn)
            current_state <= START_STATE;
        else
            current_state <= next_state;
    end
endmodule


module datapath(clk, rand_seq, display, checker, get, determine, reset_n,
		load, b1, b2, b3, b4, b5, w, endgame, rand_num, to_input, 
		b1out, b2out, b3out, b4out, b5out, user_num, 
		difficulty, score, easy, med, hard, plot_s, color, restart, high_score, det, random_sequence);

	input clk, w;
	input reset_n, load, endgame, difficulty, det;
	input rand_seq, display, checker, get, b1, b2, b3, b4, b5;
	input easy, med, hard;
	input [23:0] rand_num;
	output reg to_input, determine;
	reg [4:0] in;
	output reg b1out, b2out, b3out, b4out, b5out;
	output reg [3:0] user_num;
	output reg [7:0] score, high_score;
	output reg [2:0] color;
	output reg plot_s, restart;
	
	output reg [23:0] random_sequence;
	reg [23:0] input_sequence;
	reg input_given, add_score, update_score;
	reg [23:0] seq_speed;
	reg [3:0] seq_length;
	reg [2:0] increase_speed;
	reg [27:0] endgame_delay;
	reg [3:0] ones, tens;
	
	initial begin
		random_sequence = 24'b0;
		input_sequence = 24'b0;
		to_input <=1'b1;
		plot_s <= 1'b0;
		high_score <= 6'b0;
		tens <= 4'b0000;
		ones <= 4'b0000;
	end

	reg [3:0] display_counter;
	reg [24:0] display_delay, between_display;
	initial display_counter = 4'b0000;
	initial display_delay = 28'b0000000000000000000000000000;
	initial between_display = 28'b00000000000000000000000000;

	// parameters for the three speeds
	localparam	EASY_SPEED = 28'b1011111010111100000111111111,
			MED_SPEED = 28'b0000101111101011110000011111,
			HARD_SPEED = 28'b0001011111010111100000111111;
			
			
	always @(posedge clk)
	begin
		if (!reset_n)
			begin
			tens <= 4'b0000;
			ones <= 4'b0000;
			to_input <=1'b1;
			b1out <= 1'b0;
			b2out <= 1'b0;
			b3out <= 1'b0;
			b4out <= 1'b0;
			b5out <= 1'b0;
			restart <= 1'b1;
			endgame_delay <= 28'b0;
			
		end
		// in the difficulty state, set the sequence length and sequence speed
		else if (difficulty)
		begin
			restart <= 1'b1;
			if (easy || med || hard)
			begin
				if (easy)
				begin
					seq_length <= 4'b0100;
					seq_speed <= EASY_SPEED;
				end
				else if (med)
				begin
					seq_length <= 4'b0110;
					seq_speed <= MED_SPEED;
				end
				else if (hard)
				begin
					seq_length <= 4'b1000;
					seq_speed <= HARD_SPEED;
				end
				to_input <= 1'b0;
			end
		end
		// load the random sequence from the random sequence generator to a reg
		else if (rand_seq)
		begin
			endgame_delay <= 28'b0;
			user_num <= 4'b0000;
			if (load)
			begin
				random_sequence <= rand_num;
				to_input <= 1'b1;
			end
		end
		else if (display)
		begin
			// move on the the next state if all the sequences are displayed
			if ((display_delay == 28'b0000000000000000000000000000) && (display_counter == seq_length))
				to_input <= 1'b0;
			// draw the box with the colour corresponding to the sequence given
			else if (display_delay == 28'b0000000000000000000000000000 && between_display == 28'b0000000000000000000000000000)
			begin
				if (random_sequence[display_counter * 3] == 1'b0 && random_sequence[display_counter * 3 + 1] == 1'b0 && random_sequence[display_counter * 3 + 2] == 1'b0)
				begin
					color <= 3'b001;
					plot_s <= 1'b1;
					b1out <= 1'b1;
					b2out <= 1'b0;
					b3out <= 1'b0;
					b4out <= 1'b0;
					b5out <= 1'b0;
				end
				else if (random_sequence[display_counter * 3] == 1'b1 && random_sequence[display_counter * 3 + 1] == 1'b0 && random_sequence[display_counter * 3 + 2] == 1'b0)
				begin
					color <= 3'b010;
					plot_s <= 1'b1;
					b2out <= 1'b0;
					b1out <= 1'b0;
					b3out <= 1'b0;
					b4out <= 1'b1;
					b5out <= 1'b0;
				end
				else if (random_sequence[display_counter * 3] == 1'b0 && random_sequence[display_counter * 3 + 1] == 1'b1 && random_sequence[display_counter * 3 + 2] == 1'b0)
				begin
					color <= 3'b011;
					plot_s <= 1'b1;
					b3out <= 1'b1;
					b1out <= 1'b0;
					b2out <= 1'b0;
					b4out <= 1'b0;
					b5out <= 1'b0;
				end
				else if (random_sequence[display_counter * 3] == 1'b1 && random_sequence[display_counter * 3 + 1] == 1'b1 && random_sequence[display_counter * 3 + 2] == 1'b0)
				begin
					color <= 3'b100;
					plot_s <= 1'b1;
					b4out <= 1'b0;
					b1out <= 1'b0;
					b2out <= 1'b0;
					b3out <= 1'b1;
					b5out <= 1'b0;
				end
				else if (random_sequence[display_counter * 3] == 1'b0 && random_sequence[display_counter * 3 + 1] == 1'b0 && random_sequence[display_counter * 3 + 2] == 1'b1)
				begin
					color <= 3'b101;
					plot_s <= 1'b1;
					b5out <= 1'b1;
					b1out <= 1'b0;
					b2out <= 1'b0;
					b3out <= 1'b0;
					b4out <= 1'b0;
				end
				else
				begin
					color <= 3'b100;
					plot_s <= 1'b1;
					b5out <= 1'b0;
					b1out <= 1'b0;
					b2out <= 1'b0;
					b3out <= 1'b0;
					b4out <= 1'b1;
				end
				display_counter <= display_counter + 1'b1;
				display_delay <= seq_speed;
				between_display <= seq_speed;
			end
			// draw a black box where the previous colour box was drawn
			else if (display_delay == 28'b0000000000000000000000000000)
			begin
				b1out <= 1'b0;
				b2out <= 1'b0;
				b3out <= 1'b0;
				b4out <= 1'b0;
				b5out <= 1'b0;
				color <= 3'b0;
				plot_s <= 1'b0;
				between_display <= between_display - 1'b1;
			end
			// countdown for the display
			else
				display_delay <= display_delay - 1'b1;
		end
		// wait til a button is pressed and move on to the next state
		else if (w)
		begin
			display_counter <= 4'b0000;
			color <= 3'b0;
			b1out <= 1'b0;
			b2out <= 1'b0;
			b3out <= 1'b0;
			b4out <= 1'b0;
			b5out <= 1'b0;
			input_given <= 1'b1;
			add_score <= 1'b1;
			if (in != 5'b00000)
				to_input <= 1'b1;
		end
		else if (get)
		begin
			// if there is a button pressed, remember which button is pressed
			if (in != 5'b00000 && user_num < seq_length)
			begin
			input_given <= 1'b1;
				if (in == 5'b00001)
				begin
					input_sequence[user_num * 3] <= 0;
					input_sequence[user_num * 3 + 1] <= 0;
					input_sequence[user_num * 3 + 2] <= 1;
				end
				else if (in == 5'b00010)
				begin
					input_sequence[user_num * 3] <= 1;
					input_sequence[user_num * 3 + 1] <= 1;
					input_sequence[user_num * 3 + 2] <= 0;
				end
				else if (in == 5'b00100)
				begin
					input_sequence[user_num * 3] <= 0;
					input_sequence[user_num * 3 + 1] <= 1;
					input_sequence[user_num * 3 + 2] <= 0;
				end
				else if (in == 5'b01000)
				begin
					input_sequence[user_num * 3] <= 1;
					input_sequence[user_num * 3 + 1] <= 0;
					input_sequence[user_num * 3 + 2] <= 0;
				end
				else if (in == 5'b10000)
				begin
					input_sequence[user_num * 3] <= 0;
					input_sequence[user_num * 3 + 1] <= 0;
					input_sequence[user_num * 3 + 2] <= 0;
				end
			end
			// add to the number of buttons pressed
			else if (in == 5'b00000 && input_given == 1'b1)
			begin
				input_given <= 1'b0;
				user_num <= user_num + 1'b1;
			end
			// if the sequence length is reached, add the rest of the random sequence generated to the input sequence given
			else if (user_num >= seq_length && user_num < 4'b1000)
			begin
				input_sequence[user_num * 3] <= random_sequence[user_num * 3];
				input_sequence[user_num * 3 + 1] <= random_sequence[user_num * 3 + 1];
				input_sequence[user_num * 3 + 2] <= random_sequence[user_num * 3 + 2];
				user_num <= user_num + 1'b1;
			end
			// after all that, move on to the next state
			else if (user_num == 4'b1000)
				to_input <= 1'b0;
		end
		else if (checker)
		begin
			// if the correct sequence is inputted, add to the score and to the high score if the score is greater than the high score
			if (input_sequence == random_sequence)
			begin
			if (score > high_score)
					high_score <= {tens, ones};
				if (add_score)
				begin
					if(ones == 4'b1001)
					begin
						tens <= tens + 1;
						ones <= 4'b0000;
					end
					else
						ones <= ones + 1;
					add_score <= 1'b0;
				end
				determine <= 1'b1;
				to_input <= 1'b1;
				// if the sequence is less than 8, increase the sequence every time and speed every three correct sequences
				if (seq_length < 4'b1000)
				begin
					if (increase_speed == 2'b10)
					begin
						seq_length <= seq_length + 1;
						increase_speed <= 2'b00;
						if (seq_length == 4'b0100)
						begin
							seq_speed <= EASY_SPEED;
						end
						else if (seq_length == 4'b0110)
						begin
							seq_speed <= MED_SPEED;
						end
						else if (seq_length == 4'b0111)
						begin
							seq_speed <= HARD_SPEED;
						end
					end
					else
						increase_speed <= increase_speed + 1;
				end
			end
			// otherwise, give the appropriate signal to end the game
			else
			begin
				determine <= 1'b0;
				to_input <= 1'b1;
			end
		end
		else if (det)
		begin
			to_input <= 1'b0;
			restart <= 1'b1;
		end
		// reset appropriate variables
		else if (endgame)
		begin
			b1out <= 1'b0;
			b2out <= 1'b0;
			b3out <= 1'b0;
			b4out <= 1'b0;
			b5out <= 1'b0;
			user_num <= 4'b0000;
			if (endgame_delay == 28'b1110111001101011001001111111)
				restart <= 1'b0;
			else
				endgame_delay <= endgame_delay + 1;
		end
	end
	
	// concatenate tens and ones of score
	always @(*)
	begin
		score <= {tens, ones};
		
	end
	
	// concatenation of the buttons everytime a button is pressed and released
	always @(b1, b2, b3, b4, b5)
	begin
		in <= {b1, b2, b3, b4, b5};
	end

endmodule

module RandomSeq(square_number, clk, go, sequence, load);
	input [2:0] square_number;
	input clk;
	input go;

	output reg [23:0] sequence;
	output reg load;
	wire [2:0] count;
	wire [2:0] sq;
	reg [2:0] mainsq;
		
	// for counting the number of sequences	
	counter counting(.clock(clk), .reset(go), .out(count));
	
	// to generate 3 bits of random sequences  
	rng_5bit rng_rng(.clk(clk), .rst_n(go), .data(sq));
	
	// replace invalid 3 bit sequences with valid sequences
	always @(*)
	begin
		if(square_number == 3'b101)
		begin
			if(sq == 3'b111)
				mainsq = 3'b100;
			else if (sq == 3'b110)
				mainsq = 3'b011;
			else if (sq == 3'b101)
				mainsq = 3'b000;
			else
				mainsq = sq;
		end
	end
		
	// create a 24 bit random sequence
	always @(*)
	begin
		case(count)
			3'b000: sequence[2:0] = mainsq[2:0];
			3'b001: sequence[5:3] = mainsq[2:0];
			3'b010: sequence[8:6] = mainsq[2:0];
			3'b011: sequence[11:9] = mainsq[2:0];
			3'b100: sequence[14:12] = mainsq[2:0];
			3'b101: sequence[17:15] = mainsq[2:0];
			3'b110: sequence[20:18] = mainsq[2:0];
			3'b111: sequence[23:21] = mainsq[2:0];
		endcase
	end

	// load when not generating a random sequence
	always@(*)
	begin
		if (go == 1'b0)
			load <= 1'b1;
		else
			load <= 1'b0;
	end

endmodule

// counter
module counter(
	input clock, 
	input reset,
	output reg [2:0] out
	);
 
	always @ (posedge clock) begin
    if (!reset)
      out <= 0;
    else 
      out <= out + 1;
	end
endmodule
	
/* 5 bit randomizer from Stack Overflow
	https://stackoverflow.com/questions/14497877/how-to-implement-a-pseudo-hardware-random-number-generator
*/	
module rng_5bit(
  input clk,
  input rst_n,
  output reg [4:0] data
);

reg [4:0] data_next;

always @(*) begin
  data_next[4] = data[4]^data[1];	
  data_next[3] = data[3]^data[0];
  data_next[2] = data[2]^data_next[4];
  data_next[1] = data[1]^data_next[3];
  data_next[0] = data[0]^data_next[2];
end

always @(posedge clk or negedge rst_n)
  if(!rst_n)
    data <= 5'h1f;
  else
    data <= data_next;

endmodule

// hex display module from lab
// edited 4'hF to show H in the hex

module hex_display(IN, OUT);
    input [3:0] IN;
    output reg [6:0] OUT;
   
    always @(*)
        case (IN[3:0])
            4'h0: OUT = 7'b100_0000;
            4'h1: OUT = 7'b111_1001;
            4'h2: OUT = 7'b010_0100;
            4'h3: OUT = 7'b011_0000;
            4'h4: OUT = 7'b001_1001;
            4'h5: OUT = 7'b001_0010;
            4'h6: OUT = 7'b000_0010;
            4'h7: OUT = 7'b111_1000;
            4'h8: OUT = 7'b000_0000;
            4'h9: OUT = 7'b001_1000;
            4'hA: OUT = 7'b000_1000;
            4'hB: OUT = 7'b000_0011;
            4'hC: OUT = 7'b100_0110;
            4'hD: OUT = 7'b010_0001;
            4'hE: OUT = 7'b000_0110;
            4'hF: OUT = 7'b000_1001;    
            default: OUT = 7'h7f;
        endcase
endmodule
