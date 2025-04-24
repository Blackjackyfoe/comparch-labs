module ternary_min(a, b, out);
  input [1:0] a;
  input [1:0] b;
  output [1:0] out;

  wire aMinus;
  wire bMinus;
  wire aPlus;
  wire bPlus;
  wire outMinus;
  wire outPlus;
  wire invA0;
  wire invB0;

  not_gate ngA0(a[0], invA0);
  not_gate ngB0(b[0], invB0);
  
  and_gate isAPlus(a[1], invA0, aPlus);
  and_gate isBPlus(b[1], invB0, bPlus);

  nor_gate isAMinus(a[1], a[0], aMinus);
  nor_gate isBMinus(b[1], b[0], bMinus);

  or_gate outM(aMinus, bMinus, outMinus);

  and_gate outP(aPlus, bPlus, outPlus);
  and_gate out1(outPlus, outPlus, out[1]);
  nor_gate out0(outPlus, outMinus, out[0]);
  

endmodule

module ternary_max(a, b, out);
  input [1:0] a;
  input [1:0] b;
  output [1:0] out;

  wire aMinus;
  wire bMinus;
  wire aPlus;
  wire bPlus;
  wire outMinus;
  wire outPlus;
  wire invA0;
  wire invB0;

  not_gate ngA0(a[0], invA0);
  not_gate ngB0(b[0], invB0);
  
  and_gate isAPlus(a[1], invA0, aPlus);
  and_gate isBPlus(b[1], invB0, bPlus);

  nor_gate isAMinus(a[1], a[0], aMinus);
  nor_gate isBMinus(b[1], b[0], bMinus);

  and_gate outM(aMinus, bMinus, outMinus);

  or_gate outP(aPlus, bPlus, outPlus);
  and_gate out1(outPlus, outPlus, out[1]);
  nor_gate out0(outPlus, outMinus, out[0]);

  
endmodule

module ternary_any(a, b, out);
  input [1:0] a;
  input [1:0] b;
  output [1:0] out;

  wire aMinus;
  wire bMinus;
  wire aPlus;
  wire bPlus;
  wire outMinus;
  wire outPlus;
  wire invA0;
  wire invB0;
  wire isMCond1;
  wire isMCond2;
  wire isPCond1;
  wire isPCond2;

  not_gate ngA0(a[0], invA0);
  not_gate ngB0(b[0], invB0);
  
  and_gate isAPlus(a[1], invA0, aPlus);
  and_gate isBPlus(b[1], invB0, bPlus);

  nor_gate isAMinus(a[1], a[0], aMinus);
  nor_gate isBMinus(b[1], b[0], bMinus);
  or_gate  cond1Plus(aPlus, bPlus, isPCond1);
  nor_gate cond2Plus(aMinus, bMinus, isPCond2);
  or_gate  cond1Minus(aMinus, bMinus, isMCond1);
  nor_gate cond2Minus(aPlus, bPlus, isMCond2);
  and_gate outM(isMCond1, isMCond2, outMinus);

  and_gate outP(isPCond1, isPCond2, outPlus);
  and_gate out1(isPCond1, isPCond2, out[1]);
  nor_gate out0(outPlus, outMinus, out[0]);

endmodule

module ternary_consensus(a, b, out);
  input [1:0] a;
  input [1:0] b;
  output [1:0] out;

  wire isPMWire;
  wire nxor0;
  wire nxor1;
  wire bitCheck;
  wire nZero;

  nxor_gate nxorg0(a[0], b[0], nxor0);
  nxor_gate nxorg1(a[1], b[1], nxor1);

  nor_gate bitChecker(a[0], b[0], bitCheck);
  and_gate isPlusMinus(nxor0, nxor1, isPMWire);
  and_gate notZero(bitCheck, isPMWire, nZero);

  not_gate out0(nZero, out[0]);
  and_gate out1(a[1], b[1], out[1]);
  
endmodule

//========================================================
//Based on the lecture, kinda modified

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