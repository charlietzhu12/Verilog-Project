//@author Charles Zhu
// overview of what this code does
// This is a basic cpu simulation code
//The memory module:
module  ram(
       we,
       d,
       q,
       addr
	  );


input we;
input[15:0] d;
output reg[15:0] q;
input [7:0] addr;

reg [15:0] mem256x16 [255:0];

always @ (addr or d or we)
begin
if ( we == 1'b1)
mem256x16[addr] <= d;
else
q <= mem256x16[addr];
end
endmodule


//hw6 module for alu math calculation for multiplying and adding

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

module my16bitadder(output [15:0] O, output c, input [15:0] A, B, input S);
//wires for connecting output to input
	wire Cout0in1, Cout1in2, Cout2in3, Cout3in4, Cout4in5, Cout5in6, Cout6in7, Cout7in8,
 	Cout8in9, Cout9in10, Cout10in11, Cout11in12, Cout12in13, Cout13in14, Cout14in15;

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
	full_adder Add15 ( A[15], B[15], Cout14in15, O[15], c);
endmodule

module my8bitmultiplier(output reg[15:0] O, output reg Done, output Cout, input [7:0] A, B, input Load,Clk,Reset,Cin);
reg[1:0] state;
reg[15:0] A_reg, B_reg, A_temp, O_reg, B_temp;
wire[15:0] O_temp;

my16bitadder dut1(O_temp, Cout, A_temp, B_temp, Cin);

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
		A_temp <= A_reg; 
		B_temp <= O_reg; 
		B_reg <= B_reg - 1; 
		state <= 2;
		end
	2: begin
		O_reg <= O_temp;
		if(B_reg>1) state <= 1;
		else begin
			state <= 3; 
			Done <= 1;
			O <= O_temp;
			end
		end
	3: begin
		Done <= 0; 
		state <= 0;
		end

endcase
end
endmodule






module alu(
 A,
 B,
 opALU,
 Rout,
 clk
);
input [15:0] A;
input [15:0] B;
input [1:0] opALU;
input wire clk;
output reg [15:0] Rout;
wire [15:0] product0utput = A[7:0]*B[7:0];
wire something_add;
wire something_prod;
wire another_prod;
wire [15:0] productOutput;
wire [15:0] SumOutput;

my16bitadder addie(SumOutput, something_add, A, B, 1'b0);
my8bitmultiplier multiplie(productOutput, something_prod, another_prod,  A[7:0],  B[7:0], 1'b0, clk,1'b0, 1'b0);



always @(opALU)
begin
case(opALU)

	0: Rout <= A|B;
	1: Rout <= SumOutput;
	2: Rout <= product0utput;
	3: Rout <= ~A;
	
endcase
end
endmodule 


module ctr (
           clk,
           rst,
	   zflag,
           opcode,
	   muxPC,
           muxMAR,
           muxACC,
           loadMAR,
           loadPC,
           loadACC,
           loadMDR,
           loadIR,
           opALU,
           MemRW
);

	   input clk;
           input rst;
           input zflag;
           input [7:0]opcode;
           output reg muxPC;
           output reg muxMAR;
           output reg muxACC;
           output reg loadMAR;
           output reg loadPC;
           output reg loadACC;
           output reg loadMDR;
           output reg loadIR;
           output reg [1:0] opALU;
           output reg MemRW;

//These suggested opcode representation for proper operation
parameter op_add=8'b001;
parameter op_or= 8'b010;
parameter op_jump=8'b011;
parameter op_jumpz=8'b100;
parameter op_load=8'b101;
parameter op_store=8'b110;
parameter op_mull=8'b1001;
parameter op_neg=8'b1010;
parameter Fetch_1 = 8'b0000_0000;
parameter Fetch_2 = 8'b0000_0001;
parameter Fetch_3 = 8'b0000_0010;
parameter Decode = 8'b0000_0011;
parameter ExecAdd_1 = 8'b0000_0100;
parameter ExecAdd_2 = 8'b0000_0101;
parameter ExecOR_1 = 8'b0000_0110;
parameter ExecOR_2 = 8'b0000_0111;
parameter ExecLoad_1 = 8'b0000_1000;
parameter ExecLoad_2 = 8'b0000_1001;
parameter ExecStore_1 = 8'b0000_1010;
parameter ExecJump = 8'b0000_1011;
parameter ExecMULL_1 = 8'b0000_1100;
parameter ExecMULL_2 = 8'b0000_1110;
parameter ExecNeg = 8'b0000_1101;
parameter ExecNeg_2 = 8'b0000_1111;
parameter ExecJumpZ = 8'b0001_0000;



reg [3:0] present_state;
reg [3:0] next_state;


always@(posedge clk) begin
if (rst) present_state <= 0;
else present_state <= next_state;
end

always@(present_state or opcode or zflag) begin
case (present_state) 		
Fetch_1: begin	

    muxPC = 1'b0; 
    muxMAR = 1'b0; 
    muxACC = 1'b0; 
    loadACC = 1'b0; 
    loadMDR = 1'b0;
loadMAR = 1'b1; 
    loadPC = 1'b1; 
    loadIR = 1'b0; 
    opALU = 1'b0; 
    MemRW = 1'b0; 
    next_state <= Fetch_2;
end

Fetch_2: begin		
	MemRW <= 1'b0;
	muxPC <= 1'b0;
	muxMAR <= 1'b0;
	muxACC <= 1'b0;
	loadMAR <= 1'b0;
	loadPC <= 1'b0;
	loadACC <= 1'b0;
	loadIR <= 1'b0;
	opALU <= 1'b0;
	loadMDR <= 1'b1;
	next_state <= Fetch_3;
end

Fetch_3:
begin			
	loadIR <= 1'b1;
	muxPC <= 1'b0;
	muxMAR <= 1'b0;
	muxACC <= 1'b0;
	loadMAR <= 1'b0;
	loadPC <= 1'b0;
	loadACC <= 1'b0;
	loadMDR <= 1'b0;
	opALU <= 1'b0;
	MemRW <= 1'b0;
	next_state <= Decode;
end

Decode:			
begin
	
	muxPC <= 1'b0;
	muxACC <= 1'b0;
	loadPC <= 1'b0;
	loadMDR <= 1'b0;
	muxMAR <= 1'b1;
	loadMAR <= 1'b1;
	loadACC <= 1'b0;	
	loadIR <= 1'b0;
	opALU <= 1'b0;
	MemRW <= 1'b0;

if (opcode == op_add)next_state <= ExecAdd_1;
else if (opcode == op_or) next_state <= ExecOR_1;
else if (opcode == op_load) next_state <= ExecLoad_1;
else if (opcode == op_store) next_state <= ExecStore_1;
else if (opcode == op_jump) next_state <= ExecJump;
else if (opcode == op_jumpz) next_state <= ExecJumpZ; 
else if (opcode == op_neg) next_state <= ExecNeg;
else next_state <= ExecMULL_1;
end

ExecAdd_1:				
begin

muxPC <= 1'b0;
muxMAR <= 1'b0;
MemRW <= 1'b0;
loadMDR <= 1'b1;
muxACC <= 1'b0;
loadMAR <= 1'b0;
loadPC <= 1'b0;
loadACC <= 1'b0;
loadIR <= 1'b0;
opALU <= 1'b0;
next_state <= ExecAdd_2;
end

ExecAdd_2:
begin				

opALU <= 1'b1;
muxPC <= 1'b0;
loadACC <= 1'b1;
muxACC <= 1'b0;
muxMAR <= 1'b0;
loadMAR <= 1'b0;
loadPC <= 1'b0;
loadMDR <= 1'b0;
loadIR <= 1'b0;
MemRW <= 1'b0;
next_state <= Fetch_1;
end

ExecOR_1:
begin			

muxPC <= 1'b0;
muxMAR <= 1'b0;
muxACC <= 1'b0;
loadMAR <= 1'b0;
MemRW <= 1'b0;
loadMDR <= 1'b1;
loadPC <= 1'b0;
loadACC <= 1'b0;
loadMDR <= 1'b0;
loadIR <= 1'b0;
opALU <= 1'b0;
MemRW <= 1'b0;
next_state <= ExecOR_2;
end


ExecOR_2:
begin				

opALU <= 1'b0;
muxPC <= 1'b0;
muxMAR <= 1'b0;
loadMAR <= 1'b0;
loadACC <= 1'b1;
muxACC <= 1'b0;
loadPC <= 1'b0;
loadMDR <= 1'b0;
loadIR <= 1'b0;
MemRW <= 1'b0;
next_state <= Fetch_1;
end


ExecLoad_1: 
begin			

muxPC <= 1'b0;
muxMAR <= 1'b0;
muxACC <= 1'b0;
loadMAR <= 1'b0;
loadPC <= 1'b0;
loadACC <= 1'b0;
MemRW <= 1'b0;
loadMDR <= 1'b1;
loadMDR <= 1'b0;
loadIR <= 1'b0;
opALU <= 1'b0;
MemRW <= 1'b0;
next_state <= ExecLoad_2;
end

ExecLoad_2:
begin			

muxPC <= 1'b0;
muxMAR <= 1'b0;
loadMAR <= 1'b0;
loadPC <= 1'b0;
loadACC <= 1'b1;
muxACC <= 1'b1;
loadMDR <= 1'b0;
loadIR <= 1'b0;
MemRW <= 1'b0;
next_state <= Fetch_1;
end

ExecStore_1:
begin			

muxPC <= 1'b0;
muxMAR <= 1'b0;
muxACC <= 1'b0;
loadMAR <= 1'b0;
loadPC <= 1'b0;
MemRW <= 1'b1;
loadACC <= 1'b0;
loadMDR <= 1'b0;
loadIR <= 1'b0;
opALU <= 1'b0;
next_state <= Fetch_1;
end


ExecJump:
begin			
muxPC <= 1'b1;

muxMAR <= 1'b0;
muxACC <= 1'b0;
loadMAR <= 1'b0;
loadACC <= 1'b0;
loadMDR <= 1'b0;
loadIR <= 1'b0;
opALU <= 1'b0;
MemRW <= 1'b0;
loadPC <= 1'b1;
next_state <= Fetch_1;
end


ExecMULL_1:
begin
loadMDR <= 1'b1;						
MemRW <= 1'b0;
muxPC <= 1'b0;
muxMAR <= 1'b0;
muxACC <= 1'b0;
loadMAR <= 1'b0;
loadPC <= 1'b0;
loadACC <= 1'b0;
loadIR <= 1'b0;
opALU <= 1'b0;
next_state <= ExecMULL_2;
end

ExecMULL_2:
begin			

muxACC <= 1'b0;
opALU <= 2;
muxPC <= 1'b0;
muxMAR <= 1'b0;
loadMAR <= 1'b0;
loadPC <= 1'b0;
loadMDR <= 1'b0;
loadIR <= 1'b0;
MemRW <= 1'b0;
loadACC <= 1'b1;
next_state <= Fetch_1;
end



ExecNeg:
begin			
MemRW <= 1'b0;

muxPC <= 1'b0;
muxMAR <= 1'b0;
muxACC <= 1'b0;
loadMAR <= 1'b0;
loadPC <= 1'b0;
loadACC <= 1'b0;
loadMDR <= 1'b1;
loadIR <= 1'b0;
opALU <= 1'b0;
next_state <= ExecNeg_2;
end

ExecNeg_2:
begin			

muxACC <= 1'b0;
opALU <= 3;
muxPC <=1'b 0;
muxMAR <= 1'b0;
loadMAR <= 1'b0;
loadPC <= 1'b0;
loadMDR <= 1'b0;
loadIR <= 1'b0;
MemRW <= 1'b0;
loadACC <= 1'b1;
next_state <= Fetch_1;
end

ExecJumpZ:			
begin
if (zflag == 1'b1) next_state <= ExecJump;
else next_state <= Fetch_1;
end




endcase
end
endmodule


module registers (
           clk,
           rst,
           PC_reg, 
           PC_next,
           IR_reg,  
           IR_next,  
           ACC_reg,  
           ACC_next,  
           MDR_reg,  
           MDR_next,  
           MAR_reg,  
           MAR_next,  
           Zflag_reg,
           zflag_next
		    );

input wire clk;
input wire rst;
output reg  [7:0]PC_reg; 
input wire  [7:0]PC_next;
 
output reg  [15:0]IR_reg;  
input wire  [15:0]IR_next;  

output reg  [15:0]ACC_reg;  
input wire  [15:0]ACC_next;  

output reg  [15:0]MDR_reg;  
input wire  [15:0]MDR_next;  

output reg  [7:0]MAR_reg;  
input wire  [7:0]MAR_next;  

output reg Zflag_reg;
input wire zflag_next;

parameter zero=1'b0;



always @ ( posedge clk )
 begin
   if( rst )
begin
     	PC_reg <= zero;
 	IR_reg <= zero;
 	ACC_reg <= zero;
 	MDR_reg <= zero;
	 MAR_reg <= zero;
 	Zflag_reg <= zero;
end
   else
begin
    	PC_reg <= PC_next;
 	IR_reg <= IR_next;
 	ACC_reg <= ACC_next;
 	MDR_reg <= MDR_next;
	 MAR_reg <= MAR_next;
 	Zflag_reg <= zflag_next;
end
 end
 endmodule


module datapath(
           clk,
           rst,
           muxPC,
           muxMAR,
           muxACC,
           loadMAR,
           loadPC,
           loadACC,
           loadMDR,
           loadIR,
           opALU,
           zflag,
           opcode,
           MemAddr,
           MemD,
           MemQ
);

          input clk;
          input  rst;
          input  muxPC;
          input  muxMAR;
          input  muxACC;
          input  loadMAR;
          input  loadPC;
          input  loadACC;
          input  loadMDR;
          input  loadIR;
          input [1:0] opALU; 
          output reg  zflag;
          output  reg [7:0]opcode;
          output  reg [7:0]MemAddr;
          output reg  [15:0]MemD;
          input   [15:0]MemQ;

reg  [7:0]PC_next;
//wire  [15:0]IR_next;  //had to change to reg
reg [15:0]IR_next;  
reg  [15:0]ACC_next;  
//wire  [15:0]MDR_next;  
reg [15:0]MDR_next;  
reg  [7:0]MAR_next;  
reg zflag_next;

wire  [7:0]PC_reg;
wire  [15:0]IR_reg;  
wire  [15:0]ACC_reg;  
wire  [15:0]MDR_reg;  
wire  [7:0]MAR_reg;  
//wire zflag_reg;
reg   zflag_reg;
wire  [15:0]ALU_out;  
//wire opALU;

//one instance of ALU
alu start1(ACC_reg,MDR_reg,opALU,ALU_out,clk);
// one instance of registers.
registers another_one(clk,
rst,
PC_reg,
PC_next,
IR_reg,
IR_next,
ACC_reg,
ACC_next,
MDR_reg,
MDR_next,
MAR_reg,
MAR_next,
Zflag_reg,
zflag_next
);


always@*
begin
//[7:0]PC_next;
//Only change if loadpc is enabled.
//muxPC decides between pc+1 or branch address
//Reset address is 0, Hence nothing for the datapath to do at reset.
if(loadPC) 
PC_next=muxPC?IR_reg[15:8]:(PC_reg+1'b1);
else PC_next=PC_reg;

//[15:0]IR_next;
//Gets value of mdr_reg if loadir is set
if(loadIR)IR_next= MDR_reg;
else IR_next=IR_reg;

//[15:0]ACC_next;
//Only change when loaddacc is enabled.
//Muxacc decides between mdr_reg and alu out

if(loadACC) ACC_next=muxACC?MDR_reg:ALU_out;
//if(loadACC) ACC_next=muxACC?ALU_out:MDR_reg;
else ACC_next=ACC_reg;

//[15:0]MDR_next;
//Gets value from memeory, if load mdr is set
if(loadMDR)MDR_next=MemQ;
else MDR_next=MDR_reg;

//[7:0]MAR_next;
//Only change if loadmar is enabled.
//Mux mar decides between pcreg or IR[15:8]reg
if(loadMAR)MAR_next=muxMAR?IR_reg[15:8]:PC_reg;
else MAR_next=MAR_reg;

//zflag_next<=!ACC_reg; or if(!ACC_reg)zflag_next<=1; else zflag_next<=0;
//zflag_next=(ACC_reg==0);
//Decide based on the content of acc_reg



if (ACC_reg == 0) zflag_next <= 1;
else zflag_next <= 0;




//needs to generate the following outputs
//set this outputs based on the registered value and not the next value to prevent glitches.
zflag=(ACC_reg==0); //=> based on ACC reg
opcode=IR_reg[7:0]; //=> based on IR_reg
MemAddr=MAR_reg; //=> Same as MAR_reg
MemD =ACC_reg; //=> Same as ACC reg

end


endmodule




module proj1(
           clk,
           rst,
            MemRW_IO,
            MemAddr_IO,
           MemD_IO
            );

           input clk;
           input rst;
           output MemRW_IO;
           output [7:0]MemAddr_IO;
           output [15:0]MemD_IO;
		wire [1:0] opALU;

wire zflag,  muxPC, muxMAR, muxACC, loadMAR,  loadPC, loadACC,  loadMDR, loadIR,
            MemRW ;

wire  [7:0] opcode, MemAddr;
wire  [15:0] MemD,  MemQ;


ram  tomato(MemRW,  MemD,  MemQ ,  MemAddr  );

ctr  ctr1( clk,  rst, zflag,
           opcode, muxPC, muxMAR,
           muxACC, loadMAR,  loadPC,
           loadACC,  loadMDR, loadIR,
           opALU, MemRW );

datapath  datapath_name( clk, rst,  muxPC,  muxMAR,
         muxACC,  loadMAR,  loadPC,
         loadACC,  loadMDR,  loadIR,
         opALU,  zflag,  opcode,
         MemAddr,  MemD,  MemQ  );


 
 assign MemAddr_IO = MemAddr;
 assign MemD_IO = MemD;
 assign MemRW_IO = MemRW; 

endmodule

module proj1_tb;
           reg clk;
           reg rst;
	   wire MemRW_IO;
	   wire [7:0]MemAddr_IO;
           wire [15:0]MemD_IO;
	 
	   
proj1 dut(
           clk,
           rst,
		   MemRW_IO,
		   MemAddr_IO,
           MemD_IO
		    );



always @(proj1_tb.dut.ctr1.present_state)
case (proj1_tb.dut.ctr1.present_state)
0: $display($time," Fetch_1");
1: $display($time," Fetch_2");
2: $display($time," Fetch_3");
3: $display($time," Decode");
4: $display($time," ExecADD_1");
5: $display($time," ExecADD_2");
6: $display($time," ExecOR_1");
7: $display($time," ExecOR_2");
8: $display($time," ExecLoad_1");
9: $display($time," ExecLoad_2");
10: $display($time," ExecStore_1");
11: $display($time," ExecJump");
12: $display($time," ExecMULL_1");
13: $display($time," ExecMULL_2");
14: $display($time," ExecNeg_1");
15: $display($time," ExecNeg_2");
16: $display($time," Jump_Z");
default: $display($time," Unrecognized State");
endcase 





always 
      #5  clk =  !clk; 
		
initial begin
clk=1'b0;
rst=1'b1;
$readmemh("memory.list", proj1_tb.dut.tomato.mem256x16);
#20 rst=1'b0;
#7000
$display("Final value\n");
$display("0x000e %h\n",proj1_tb.dut.tomato.mem256x16[16'h000e]);



$finish;
end

endmodule





