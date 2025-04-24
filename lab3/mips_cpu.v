`include "util.v"

module mips_cpu(clk, pc, pc_new, instruction_memory_a, instruction_memory_rd, data_memory_a, data_memory_rd, data_memory_we, data_memory_wd,
                register_a1, register_a2, register_a3, register_we3, register_wd3, register_rd1, register_rd2);
  // сигнал синхронизации
  input clk;
  // текущее значение регистра PC
  inout [31:0] pc;
  // новое значение регистра PC (адрес следующей команды)
  output [31:0] pc_new;
  // we для памяти данных
  output data_memory_we;
  // адреса памяти и данные для записи памяти данных
  output [31:0] instruction_memory_a, data_memory_a, data_memory_wd;
  // данные, полученные в результате чтения из памяти
  inout [31:0] instruction_memory_rd, data_memory_rd;
  // we3 для регистрового файла
  output register_we3;
  // номера регистров
  output [4:0] register_a1, register_a2, register_a3;
  // данные для записи в регистровый файл
  output [31:0] register_wd3;
  // данные, полученные в результате чтения из регистрового файла
  inout [31:0] register_rd1, register_rd2;

  //Получаем инструкцию -> записываем в память -> инициализируем control unit
  // -> Выставляем нужные флаги и передаём их в АЛУ -> Парсим что надо сделать ->
  // -> Записываем данные -> Прокидываем нужные флаги и начинаем заново

  //R
  wire [5:0] opcode = instruction_memory_rd[31:26];
  wire [4:0] r1 = instruction_memory_rd[25:21]; //source
  wire [4:0] r2 = instruction_memory_rd[20:16]; //target
  wire [4:0] rd = instruction_memory_rd[15:11]; //destination
  wire [4:0] shamt = instruction_memory_rd[10:6];
  wire [5:0] funct = instruction_memory_rd[5:0];

  //I
  wire [15:0] const = instruction_memory_rd[15:0];
  wire [31:0] extended_const;
  sign_extend se1(const, extended_const);

  //J
  wire[25:0] addr = instruction_memory_rd[25:0];


  wire [2:0] ALUControl;
  wire MemToReg, MemWrite, RegDst, RegWrite;
  wire ALUSrc, Branch, BranchCond;
  wire Jump, Jal, Jr;

  control_unit cu1(opcode, funct,
                   MemToReg, MemWrite, RegDst, RegWrite,
                   ALUSrc, Branch, BranchCond, Jump, Jal, Jr, ALUControl);

  assign register_a1 = r1; // rs
  assign register_a2 = r2; // rt
  assign register_a3 = Jal ? 5'd31 : (RegDst ? rd : r2);
  assign register_we3 = RegWrite || Jal;

  wire [31:0] finalConst = (opcode == 6'b001100) ? {16'b0, const} : extended_const;
  wire [31:0] ALUResult;
  wire [31:0] aluA = register_rd1;
  wire [31:0] aluB;
  mux2_32 chooseB(register_rd2, finalConst, ALUSrc, aluB);
  wire zero;
  alu alu1(aluA, aluB, ALUControl, ALUResult, zero);

  assign data_memory_a = ALUResult;
  assign data_memory_wd = register_rd2;
  assign data_memory_we = MemWrite;


  wire [31:0] constOffset2;
  shl_2 sh2(extended_const, constOffset2);
  wire [31:0] pc4, pcBranch, jump_address, branch_address;
  wire [31:0] jrAddr, pcBranch4, pcJBranch, pcJJr;
  wire [31:0] memAlu = MemToReg ? data_memory_rd : ALUResult;
  adder pc_adder1(pc, 32'd4, pc4);
  adder pc_adder2(pc, 32'd4, pcBranch);
  assign register_wd3 = Jal ? pc4 : memAlu;

  wire eqRd1Rd2 = (register_rd1 == register_rd2);
  wire do_branch = Branch & (BranchCond ? (~eqRd1Rd2) : eqRd1Rd2);
  assign branch_address = pcBranch + constOffset2;
  assign jump_address = {pcBranch[31:28], addr, 2'b00};
  assign jrAddr = register_rd1;
  assign pcBranch4 = do_branch ? branch_address : pcBranch;
  assign pcJBranch = Jump ? jump_address : pcBranch4;
  assign pcJJr = Jr ? jrAddr : pcJBranch;

  assign pc_new = pcJJr;
  assign instruction_memory_a = pc;
endmodule

module control_unit(opcode, funct,
                    MemToReg, MemWrite, RegDst, RegWrite,
                    ALUSrc, Branch, BranchCond, Jump, Jal, Jr, ALUControl);

  input wire [5:0] opcode, funct;
  output reg MemToReg, MemWrite, RegDst, RegWrite;
  output reg ALUSrc, Branch, BranchCond;
  output reg Jump, Jal, Jr;
  output reg [2:0] ALUControl;

 always @(*) begin
    case (opcode)
      6'b000000: begin // R
        RegDst = 1; ALUSrc = 0; MemToReg = 0; RegWrite = 1;
        MemWrite = 0; Branch = 0; BranchCond = 0;
        Jump = 0; Jal = 0; Jr = 0;
        case (funct)
          6'b100000: ALUControl = 3'b010; // add
          6'b100010: ALUControl = 3'b110; // sub
          6'b100100: ALUControl = 3'b000; // and
          6'b100101: ALUControl = 3'b001; // or
          6'b101010: ALUControl = 3'b111; // slt
          6'b001000: begin
            Jr = 1; RegWrite = 0;
          end
        endcase
      end
      6'b100011: begin // lw
        RegDst = 0; ALUSrc = 1; MemToReg = 1; RegWrite = 1;
        MemWrite = 0; Branch = 0; BranchCond = 0;
        Jump = 0; Jal = 0; Jr = 0;
        ALUControl = 3'b010;
      end
      6'b101011: begin // sw
        RegDst = 0; ALUSrc = 1; MemToReg = 0; RegWrite = 0;
        MemWrite = 1; Branch = 0; BranchCond = 0;
        Jump = 0; Jal = 0; Jr = 0;
        ALUControl = 3'b010;
      end
      6'b000100: begin // beq
        RegDst = 0; ALUSrc = 0; MemToReg = 0; RegWrite = 0;
        MemWrite = 0; Branch = 1; BranchCond = 0;
        Jump = 0; Jal = 0; Jr = 0;
        ALUControl = 3'b110;
      end
      6'b001000: begin // addi
        RegDst = 0; ALUSrc = 1; MemToReg = 0; RegWrite = 1;
        MemWrite = 0; Branch = 0; BranchCond = 0;
        Jump = 0; Jal = 0; Jr = 0;
        ALUControl = 3'b010;
      end
      6'b001100: begin // andi
        RegDst = 0; ALUSrc = 1; MemToReg = 0; RegWrite = 1;
        MemWrite = 0; Branch = 0; BranchCond = 0;
        Jump = 0; Jal = 0; Jr = 0;
        ALUControl = 3'b000;
      end
      6'b000101: begin // bne
        RegDst = 0; ALUSrc = 0; MemToReg = 0; RegWrite = 0;
        MemWrite = 0; Branch = 1; BranchCond = 1;
        Jump = 0; Jal = 0; Jr = 0;
        ALUControl = 3'b110;
      end
      6'b000010: begin // j
        RegDst = 0; ALUSrc = 0; MemToReg = 0; RegWrite = 0;
        MemWrite = 0; Branch = 0; BranchCond = 0;
        Jump = 1; Jal = 0; Jr = 0;
        ALUControl = 3'b010;
      end
      6'b000011: begin // jal
        RegDst = 0; ALUSrc = 0; MemToReg = 0; RegWrite = 0;
        MemWrite = 0; Branch = 0; BranchCond = 0;
        Jump = 1; Jal = 1; Jr = 0;
        ALUControl = 3'b010;
      end
    endcase
  end

endmodule

module alu(a, b, alu_control, alu_result, zero);
    input [31:0] a;
    input [31:0] b;
    input [2:0]  alu_control;
    output reg [31:0] alu_result;
    output reg zero;

    always @(*) begin
    case (alu_control)
        3'b000: alu_result = a & b;
        3'b001: alu_result = a | b;
        3'b010: alu_result = a + b;
        3'b110: alu_result = a - b;
        3'b111: alu_result = (a < b) ? 1 : 0; // slt
    endcase
    zero = (alu_result == 0) ? 1 : 0;
    end

endmodule