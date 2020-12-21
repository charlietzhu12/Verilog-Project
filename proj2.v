module fullsubtractor( input [15:0] x, input [15:0] y, output [15:0] O);
 
assign O = y -x ;
 
endmodule




module mymodfunc(output reg [15:0] O, output reg Done, 
input [15:0] A, B, input Load,Clk,Reset);

reg[1:0] state;
reg[15:0] B_reg, A_reg, B_temp, O_reg, A_temp;
wire [15:0] O_temp;





fullsubtractor dut1(B_temp, A_temp, O_temp);

always@(posedge Clk)
begin
if(Reset)
state <= 0;
else
case(state)
	0: if(Load)begin
		A_reg <= A; 
		B_reg <= B; 
		O_reg <= A; 
		state <= 1;
                Done <= 0;
                O <= 0;
		end
	1: begin
		A_temp <= O_reg; 
		B_temp <= B_reg; 
		//B_reg <= B_reg - 1; 
		state <= 2;
		end
	2: begin
		O_reg <= O_temp;
		if(O_temp < B) 
		begin
			state <= 3; 
			Done <= 1;
			O <= O_temp;
			end
			else 
		    state <=1;
		end
		
	3: begin
		Done <= 0; 
		state <= 0;
		end

endcase
end
endmodule



//Declare the ports of Half adder module
module half_adder(A, B, S, C);
    //what are the input ports.
    input A;
    input B;
    //What are the output ports.
    output S;
    output C;
    //reg S,C; NOT NEEDED will cause error
     //Implement the Sum and Carry equations using Structural verilog
     xor x1(S,A,B); //XOR operation (S is output not a input)
     and a1(C,A,B); //AND operation 
 endmodule

//Declare the ports for the full adder module
module full_adder( A, B, Cin, S, Cout); // (Port names)

    //what are the input ports.
    input A;
    input B;
    input Cin;

    //What are the output ports.
    output S;
    output Cout;
    //reg  A,B,Cin; //(Doesn't need to be reg, it will cause an error
    wire Sa1, Ca1, Ca2;    
     //Two instances of half adders used to make a full adder
     half_adder a1(.A(A),.B(B),.S(Sa1),.C(Ca1)); //S to Sa1 to match diagram
     half_adder a2(.A(Sa1),.B(Cin),.S(S),.C(Ca2));
     or  o1(Cout,Ca1,Ca2);
 endmodule

module my32bitadder(output [31:0] O, output c, input [31:0] A, B, input S);
//wires for connecting output to input
	wire Cout0in1, Cout1in2, Cout2in3, Cout3in4, Cout4in5, Cout5in6, Cout6in7, Cout7in8,
 	Cout8in9, Cout9in10, Cout10in11, Cout11in12, Cout12in13, Cout13in14, Cout14in15,C15,C16,C17,
	C18,C19,C20,C21,C22,C23,C24,C25,C26,C27,C28,C29,C30;

	full_adder Add0 ( A[0], B[0], S, O[0], Cout0in1); 
	full_adder Add1 ( A[1], B[1], Cout0in1, O[1], Cout1in2); 
	full_adder Add2 ( A[2], B[2], Cout1in2, O[2], Cout2in3); 
	full_adder Add3 ( A[3], B[3], Cout2in3, O[3], Cout3in4); 
	full_adder Add4 ( A[4], B[4], Cout3in4, O[4], Cout4in5); 
	full_adder Add5 ( A[5], B[5], Cout4in5, O[5], Cout5in6); 
	full_adder Add6 ( A[6], B[6], Cout5in6, O[6], Cout6in7); 
	full_adder Add7 ( A[7], B[7], Cout6in7, O[7], Cout7in8); 
	full_adder Add8 ( A[8], B[8], Cout7in8, O[8], Cout8in9); 
	full_adder Add9 ( A[9], B[9], Cout8in9, O[9], Cout9in10); 
	full_adder Add10 ( A[10], B[10], Cout9in10, O[10], Cout10in11); 
	full_adder Add11 ( A[11], B[11], Cout10in11, O[11], Cout11in12);
	full_adder Add12 ( A[12], B[12], Cout11in12, O[12], Cout12in13); 
	full_adder Add13 ( A[13], B[13], Cout12in13, O[13], Cout13in14); 
	full_adder Add14 ( A[14], B[14], Cout13in14, O[14], Cout14in15);
	full_adder Add15 ( A[15], B[15], Cout9in10, O[15], C15); 
	full_adder Add16 ( A[16], B[16], Cout10in11, O[16], C16);
	full_adder Add17 ( A[17], B[17], Cout11in12, O[17], C17); 
	full_adder Add18 ( A[18], B[18], Cout12in13, O[18], C18); 
	full_adder Add19 ( A[19], B[19], Cout13in14, O[19], C19);  
	full_adder Add20 ( A[20], B[20], Cout9in10, O[20], C20); 
	full_adder Add21 ( A[21], B[21], Cout10in11, O[21], C21);
	full_adder Add22 ( A[22], B[22], Cout11in12, O[22], C22); 
	full_adder Add23 ( A[23], B[23], Cout12in13, O[23], C23); 
	full_adder Add24 ( A[24], B[24], Cout13in14, O[24], C24);
	full_adder Add25 ( A[25], B[25], Cout9in10, O[25], C25); 
	full_adder Add26 ( A[26], B[26], Cout10in11, O[26], C26);
	full_adder Add27 ( A[27], B[27], Cout11in12, O[27], C27);
	full_adder Add28 ( A[28], B[28], Cout12in13, O[28], C28); 
	full_adder Add29 ( A[29], B[29], Cout13in14, O[29], C29);
	full_adder Add30 ( A[30], B[30], Cout9in10, O[30], C30);
	full_adder Add31 ( A[31], B[31], Cout14in15, O[31], c);
endmodule

module myfulladder32bit (input [15:0] x, input [15:0] y,output [15:0] O);
assign O =   x +  y;
endmodule

module myexp(output reg[15:0] O, output reg Done, input [15:0] A, B, input Load,Clk,Reset);
reg[2:0] state;
reg[15:0] A_reg, B_reg, A_temp, O_reg, B_temp,C_reg;
wire[15:0] O_temp;

myfulladder32bit dut1( A_temp, B_temp,O_temp);

always@(posedge Clk)
begin
if(Reset)
state <= 0;
else
case(state)
	0: if(Load)begin
		A_reg <= A; 
		B_reg <= A; 
		O_reg <= A; 
		C_reg <= B; // This is the exponet Checker
		state <= 1;
                Done <= 0;
                O <= 0;
		end
	1: begin
		A_temp <= A_reg; 
		B_temp <= O_reg; 
		B_reg <= B_reg - 1; 
		state <= 2;   // Does same mult statement 
		end
	2: begin
		O_reg <= O_temp;
		if(B_reg>1) state <= 1; // Checks mult checker
		else state <= 3; 
		end
	3: begin 
		A_reg <= O_reg;
		B_reg <= A;
		C_reg <= C_reg-1; 
		state <= 4;
	   end
	4: begin
		if(C_reg>1) state <= 1; // Checks exponet calculator 
		else begin
			state <= 5;
			Done <= 1;
			O <= O_reg;
	   end
	   end
	5: begin
		Done <= 0;  //finish
		state <= 0;
	   end

endcase
end
endmodule


/*
module proj2(input reg[15:0] private_key, public_key, message_val, input  clk, start,rst, output reg cal_done, output reg [15:0] cal_value);
reg [3:0]state;
reg load_exp,load_mod,rst_exp,rst_mod;
reg [15:0] private, public, message;
wire myexpo_done, mymod_done;
wire[15:0] exp_out,mod_out;

myexp exponet(exp_out, myexpo_done, message, private, load_exp, clk, rst_exp);
mymodfunc modoluo(mod_out, mymod_done, exp_out, public, load_mod, clk, rst_mod);
*/

module myencryption(output reg Cal_done, output reg [15:0] Cal_val, input Rst,Start,clk,input [15:0] private_key, input [15:0] public_key,
 input [15:0] message_val);
 
//module proj2(input [15:0] private_key, public_key, message_val, input  clk, Start,rst, output reg Cal_done, output reg [15:0] Cal_val);

 parameter Capture_state = 8'b0000;
 parameter Exponent_state1 = 8'b0001;
 parameter Exponent_state2 =8'b0010;
 parameter Exponent_state3 =8'b0011;
 parameter Mod_state1 =8'b0100;
 parameter Mod_state2 =8'b0101;
 parameter Mod_state3 =8'b0110;
 parameter Cal_done_state1 =8'b0111;
 parameter Cal_done_state2 = 8'b1000;
 reg [3:0] currentState;

 
 reg [15:0] temp_private_key; //passed in functions
 reg [15:0] temp_public_key; //passed
 reg [15:0] temp_cal_val; //
 //wire cin = 0;
 //wire cout;
 wire [15:0] exp_out;
 wire [15:0] mod_out;
 wire exp_Done;
 wire mod_Done;
 reg exp_Load;
 reg mod_Load;
 reg Rst_mod;
 reg Rst_exp;
 /*
 myexp exponet(exp_out, myexpo_done, message, private, load_exp, clk, rst_exp);
mymodfunc modoluo(mod_out, mymod_done, exp_out, public, load_mod, clk, rst_mod);
*/
myexp tomato(exp_out, exp_Done, temp_cal_val, temp_private_key, exp_Load,clk,Rst_exp);
 
mymodfunc potato(mod_out, mod_Done, exp_out, temp_public_key, mod_Load,clk,Rst_mod);

 
 always @(posedge clk)
begin	
	if (Rst)
		begin
		 mod_Load <=0;
		 exp_Load <=0;
		 //cout <= 0;
		 currentState <=0;
		 Cal_val <=0;
		 Cal_done <=0;
		 Rst_mod <= 0;
		 Rst_exp <=0;
		 temp_public_key <=0;
		 temp_private_key <=0;
		 temp_cal_val <=0;
		 //currentState <= Capture_state;
		end
	else
		case (currentState)
		Capture_state:
		begin
		if (Start)
		    begin
			//temp_cal_val <=0;
			Cal_done <= 0;
			temp_public_key <= public_key;
			temp_private_key <= private_key;
			temp_cal_val <= message_val;
			Rst_exp <=1;
			Rst_mod <=1;
			currentState <= Exponent_state1;
			end
		else currentState <= Capture_state;
		    end
		    
	Exponent_state1:
	begin
	exp_Load <= 1;
	Rst_exp <= 0;
	currentState <= Exponent_state2;
	end
	
	Exponent_state2:
	begin
	exp_Load <= 0;
	currentState <= Exponent_state3;
	end
	
	Exponent_state3:
	begin
	if (exp_Done) begin
	currentState <= Mod_state1;
	Rst_mod <= 1;
	end
	else 
	//temp_cal_val <= exp_out;
	currentState <= Exponent_state2;
	end
	
	Mod_state1:
	begin
	mod_Load <= 1'b1;
	Rst_mod <= 0;
	currentState <= Mod_state2;
	end
	
	Mod_state2:
	begin
	mod_Load <= 1'b0;
	currentState <= Mod_state3;
	end
	
	Mod_state3:
	begin
	if (mod_Done == 1)
	currentState <= Cal_done_state1;
	//currentState <= Mod_state2;
	else 
	//temp_cal_val <= mod_out;
	currentState <= Mod_state2;
	//currentState <= Cal_done_state1;
	end
	
	Cal_done_state1:
	begin
	Cal_done <= 1;
	Cal_val <= mod_out;
	currentState <= Cal_done_state2;
	//currentState <= Capture_state;
	end
	
	Cal_done_state2:
	begin 
	currentState <= Capture_state;
	end
	endcase
	
end
 endmodule
 
 
module proj2_tb();
reg clk, Rst, Start;
reg [15:0] private_key, public_key,message_val;
wire Cal_done;
wire[15:0] Cal_val;

//myencryption dut(Cal_done, Cal_val, Rst,Start,clk, private_key, public_key, message_val);
 //myencryption dut(Cal_done, Cal_val, Rst,Start,clk, private_key,  public_key, message_val);
 //myencryption dut(private_key, public_key, message_val, clk, start,rst, Cal_done,Cal_val);
 proj2 dut(Cal_done, Cal_val, Rst,Start,clk, private_key, public_key, message_val);
 //proj2 dut(private_key, public_key, message_val, clk, start,rst, Cal_done,Cal_val);
always #5 clk = ~clk;
initial
begin

clk = 0;
Rst = 0;
#4 
Rst = 1;
#6
Rst = 0;

Start = 1;


private_key = 16'd3; 
public_key = 16'd33;
message_val = 16'd9;

//#10 Rst = 0;
//#10 load = 0; 

#7000 $display (" private_key = %d, public_key = %d, message_val = %d, cal_val = %d", private_key, public_key, message_val, Cal_val);
#10 Start = 0;

/*
  clk = 0;
   Rst = 0;
   #4 Rst = 1; #6 Rst = 0; Start = 1;
   message_val = 16'd9; private_key = 16'd3; public_key = 16'd33;
   #10000 $display ("Encryption \nC = %0d^%0d mod %0d = %0d", message_val, private_key, public_key, Cal_val);
   #10 Start = 0;
  */
#100 $finish;
end
endmodule
 
 
 