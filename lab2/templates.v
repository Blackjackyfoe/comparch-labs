

module alu(a, b, control, res);
  input [3:0] a, b;
  input [2:0] control;
  output [3:0] res;

  wire [3:0] andRes, andNBRes, orRes, orNBRes, notB;
  wire [3:0] subRes, addRes, sltRes;
  wire [3:0] carry, borrow;
  
  assign res = (control == 3'b000) ? andRes :
               (control == 3'b001) ? andNBRes :
               (control == 3'b010) ? orRes :
               (control == 3'b011) ? orNBRes :
               (control == 3'b100) ? subRes :
               (control == 3'b101) ? addRes :
               (control == 3'b110) ? sltRes :
               4'b0000;

  not_gate bit0B(b[0], notB[0]);
  not_gate bit1B(b[1], notB[1]);
  not_gate bit2B(b[2], notB[2]);
  not_gate bit3B(b[3], notB[3]);
  
  and_gate bit0And(a[0], b[0], andRes[0]);
  and_gate bit1And(a[1], b[1], andRes[1]);
  and_gate bit2And(a[2], b[2], andRes[2]);
  and_gate bit3And(a[3], b[3], andRes[3]);

  and_gate bit0AndNB(a[0], notB[0], andNBRes[0]);
  and_gate bit1AndNB(a[1], notB[1], andNBRes[1]);
  and_gate bit2AndNB(a[2], notB[2], andNBRes[2]);
  and_gate bit3AndNB(a[3], notB[3], andNBRes[3]);

  or_gate bit0_Or(a[0], b[0], orRes[0]);
  or_gate bit1_Or(a[1], b[1], orRes[1]);
  or_gate bit2_Or(a[2], b[2], orRes[2]);
  or_gate bit3_Or(a[3], b[3], orRes[3]);

  or_gate bit0_OrNB(a[0], notB[0], orNBRes[0]);
  or_gate bit1_OrNB(a[1], notB[1], orNBRes[1]);
  or_gate bit2_OrNB(a[2], notB[2], orNBRes[2]);
  or_gate bit3_OrNB(a[3], notB[3], orNBRes[3]);

  fullSummator sub0(a[0], notB[0], 1'b1, borrow[0], subRes[0]);
  fullSummator sub1(a[1], notB[1], borrow[0], borrow[1], subRes[1]);
  fullSummator sub2(a[2], notB[2], borrow[1], borrow[2], subRes[2]);
  fullSummator sub3(a[3], notB[3], borrow[2], borrow[3], subRes[3]);

  fullSummator add0(a[0], b[0], 1'b0, carry[0], addRes[0]);
  fullSummator add1(a[1], b[1], carry[0], carry[1], addRes[1]);
  fullSummator add2(a[2], b[2], carry[1], carry[2], addRes[2]);
  fullSummator add3(a[3], b[3], carry[2], carry[3], addRes[3]);

  assign sltRes = ($signed(a) < $signed(b)) ? 4'b0001 : 4'b0000;

endmodule

module fullSummator(a, b, in, carry, sum);
    output sum, carry;
    input a, b, in;

    wire xorOut, and1Out, and2Out;

    xor_gate xor1(a, b, xorOut);
    xor_gate xor2(xorOut, in, sum);
    and_gate and1(a, b, and1Out);
    and_gate and2(in, xorOut, and2Out);
    or_gate orG(and1Out, and2Out, carry);

endmodule

module d_latch(clk, d, we, q);
  input clk;
  input d;
  input we;

  output reg q;
  initial begin
    q <= 0;
  end
  always @ (negedge clk) begin
    if (we) begin
      q <= d;
    end
  end
endmodule

module register_file(clk, rd_addr, we_addr, we_data, rd_data, we);
  input clk;
  input [1:0] rd_addr, we_addr; 
  input [3:0] we_data;
  input we;
  output [3:0] rd_data;
  
  wire [3:0] r0, r1, r2, r3, weReg;

  assign rd_data = (rd_addr == 2'b00) ? r0 :
                   (rd_addr == 2'b01) ? r1 :
                   (rd_addr == 2'b10) ? r2 :
                                        r3;

  d_latch r0_0(.clk(clk), .d(we_data[0]), .we(we & (we_addr == 2'b00)), .q(r0[0]));
  d_latch r0_1(.clk(clk), .d(we_data[1]), .we(we & (we_addr == 2'b00)), .q(r0[1]));
  d_latch r0_2(.clk(clk), .d(we_data[2]), .we(we & (we_addr == 2'b00)), .q(r0[2]));
  d_latch r0_3(.clk(clk), .d(we_data[3]), .we(we & (we_addr == 2'b00)), .q(r0[3]));

  d_latch r1_0(.clk(clk), .d(we_data[0]), .we(we & (we_addr == 2'b01)), .q(r1[0]));
  d_latch r1_1(.clk(clk), .d(we_data[1]), .we(we & (we_addr == 2'b01)), .q(r1[1]));
  d_latch r1_2(.clk(clk), .d(we_data[2]), .we(we & (we_addr == 2'b01)), .q(r1[2]));
  d_latch r1_3(.clk(clk), .d(we_data[3]), .we(we & (we_addr == 2'b01)), .q(r1[3]));

  d_latch r2_0(.clk(clk), .d(we_data[0]), .we(we & (we_addr == 2'b10)), .q(r2[0]));
  d_latch r2_1(.clk(clk), .d(we_data[1]), .we(we & (we_addr == 2'b10)), .q(r2[1]));
  d_latch r2_2(.clk(clk), .d(we_data[2]), .we(we & (we_addr == 2'b10)), .q(r2[2]));
  d_latch r2_3(.clk(clk), .d(we_data[3]), .we(we & (we_addr == 2'b10)), .q(r2[3]));

  d_latch r3_0(.clk(clk), .d(we_data[0]), .we(we & (we_addr == 2'b11)), .q(r3[0]));
  d_latch r3_1(.clk(clk), .d(we_data[1]), .we(we & (we_addr == 2'b11)), .q(r3[1]));
  d_latch r3_2(.clk(clk), .d(we_data[2]), .we(we & (we_addr == 2'b11)), .q(r3[2]));
  d_latch r3_3(.clk(clk), .d(we_data[3]), .we(we & (we_addr == 2'b11)), .q(r3[3]));

endmodule

module counter(clk, addr, control, immediate, data);
  input clk;
  input [1:0] addr; 
  input [3:0] immediate; 
  input control;
  output [3:0] data;

  wire [3:0] val0, val1, val2, val3, nextVal1, nextVal2, nextVal3, nextVal0;

  assign nextVal0 = control ? (val0 - immediate) : (val0 + immediate);
  assign nextVal1 = control ? (val1 - immediate) : (val1 + immediate);
  assign nextVal2 = control ? (val2 - immediate) : (val2 + immediate);
  assign nextVal3 = control ? (val3 - immediate) : (val3 + immediate);

  assign data = (addr == 2'b00) ? val0 :
                (addr == 2'b01) ? val1 :
                (addr == 2'b10) ? val2 :
                                  val3;

  d_latch d0_0(.clk(clk), .d(nextVal0[0]), .we(addr == 2'b00), .q(val0[0]));
  d_latch d0_1(.clk(clk), .d(nextVal0[1]), .we(addr == 2'b00), .q(val0[1]));
  d_latch d0_2(.clk(clk), .d(nextVal0[2]), .we(addr == 2'b00), .q(val0[2]));
  d_latch d0_3(.clk(clk), .d(nextVal0[3]), .we(addr == 2'b00), .q(val0[3]));

  d_latch d1_0(.clk(clk), .d(nextVal1[0]), .we(addr == 2'b01), .q(val1[0]));
  d_latch d1_1(.clk(clk), .d(nextVal1[1]), .we(addr == 2'b01), .q(val1[1]));
  d_latch d1_2(.clk(clk), .d(nextVal1[2]), .we(addr == 2'b01), .q(val1[2]));
  d_latch d1_3(.clk(clk), .d(nextVal1[3]), .we(addr == 2'b01), .q(val1[3]));

  d_latch d2_0(.clk(clk), .d(nextVal2[0]), .we(addr == 2'b10), .q(val2[0]));
  d_latch d2_1(.clk(clk), .d(nextVal2[1]), .we(addr == 2'b10), .q(val2[1]));
  d_latch d2_2(.clk(clk), .d(nextVal2[2]), .we(addr == 2'b10), .q(val2[2]));
  d_latch d2_3(.clk(clk), .d(nextVal2[3]), .we(addr == 2'b10), .q(val2[3]));

  d_latch d3_0(.clk(clk), .d(nextVal3[0]), .we(addr == 2'b11), .q(val3[0]));
  d_latch d3_1(.clk(clk), .d(nextVal3[1]), .we(addr == 2'b11), .q(val3[1]));
  d_latch d3_2(.clk(clk), .d(nextVal3[2]), .we(addr == 2'b11), .q(val3[2]));
  d_latch d3_3(.clk(clk), .d(nextVal3[3]), .we(addr == 2'b11), .q(val3[3]));

endmodule

//========================================================
//Based on the lecture, kinda modified (cringe removed, xnor added)

module nor_gate(a, b, out);
  input wire a, b;
  output wire out;

  supply1 vdd;
  supply0 gnd;

  wire pmos1_out;

  pmos pmos1(pmos1_out, vdd, a);
  pmos pmos2(out, pmos1_out, b);
  nmos nmos1(out, gnd, a);
  nmos nmos2(out, gnd, b);
endmodule

module nand_gate(a, b, out);
    input wire a, b;
    output wire out;

    wire out_in;

    supply1 vdd;
    supply0 gnd;

    pmos pmos1(out, vdd, a);
    pmos pmos2(out, vdd, b);

    nmos nmos1(out_in, gnd, b);
    nmos nmos2(out, out_in, a);

endmodule

module not_gate(in, out);
    input wire in;
    output wire out;

    supply1 vdd;
    supply0 gnd;

    pmos pmos1(out, vdd, in);
    nmos nmos1(out, gnd, in);

endmodule

module and_gate(a, b, out);
    input wire a, b;
    output wire out;

    wire nand_out;

    nand_gate ng(a, b, nand_out);
    not_gate ng2(nand_out, out);

endmodule

module or_gate(a, b, out);
  input wire a, b;
  output wire out;

  wire nor_out;

  nor_gate nor_gate(a, b, nor_out);
  not_gate not_gate(nor_out, out);
endmodule

module xor_gate(a, b, out);
  input wire a, b;
  output wire out;

  wire not_in1;
  wire not_in2;

  wire and_out1;
  wire and_out2;

  wire or_out1;

  not_gate notA(a, not_in1);
  not_gate notB(b, not_in2);

  and_gate and1(a, not_in2, and_out1);
  and_gate and2(not_in1, b, and_out2);

  or_gate or1(and_out1, and_out2, out);

endmodule

module nxor_gate(a, b, out);
  input wire a, b;
  output wire out;

  wire nxor_out;

  xor_gate xg(a, b, nxor_out);
  not_gate ng(nxor_out, out);

endmodule