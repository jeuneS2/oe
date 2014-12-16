/*
   Copyright 2014 Technical University of Denmark, DTU Compute. 
   All rights reserved.
   
   This file is part of the processor Ø.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

      1. Redistributions of source code must retain the above copyright notice,
         this list of conditions and the following disclaimer.

      2. Redistributions in binary form must reproduce the above copyright
         notice, this list of conditions and the following disclaimer in the
         documentation and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER ``AS IS'' AND ANY EXPRESS
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
   NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
   THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

   The views and conclusions contained in the software and documentation are
   those of the authors and should not be interpreted as representing official
   policies, either expressed or implied, of the copyright holder.
 */

/*
 * A simple assembler for Ø
 *
 * Author: Wolfgang Puffitsch (wpuffitsch@gmail.com)
 *
 */

package oe

import scala.util.parsing.combinator._

import Opcode._
import Config._

object Assembler extends RegexParsers {

  def intOf(x: Chisel.Bits) = x.litValue().intValue

  override val whiteSpace = """(//.*|\s)+""".r

  def number: Parser[Int] = """-?\d+""".r ^^ { _.toInt }

  def opc: Parser[Int] = ("ldi" ^^^ intOf(LDI) |
                          "ldl" ^^^ intOf(LDL) |
                          "ldo" ^^^ intOf(LDO) |
                          "sto" ^^^ intOf(STO) |
                          "add" ^^^ intOf(ADD) |
                          "sub" ^^^ intOf(SUB) |
                          "and" ^^^ intOf(AND) |
                          "ior" ^^^ intOf(IOR) |
                          "xor" ^^^ intOf(XOR) |
                          "mul" ^^^ intOf(MUL) |
                          "bnz" ^^^ intOf(BNZ))

  def uop: Parser[Int] = ("srl" ^^^ intOf(SRL) |
                          "sra" ^^^ intOf(SRA) |
                          "not" ^^^ intOf(NOT) |
                          "neg" ^^^ intOf(NEG) |
                          "cmp" ^^^ intOf(CMP) |
                          "adc" ^^^ intOf(ADC) |
                          "sx8" ^^^ intOf(SX8) |
                          "zx8" ^^^ intOf(ZX8) |
                          "sl8" ^^^ intOf(SL8) |
                          "sr8" ^^^ intOf(SR8) |
                          "sx16" ^^^ intOf(SX16) |
                          "zx16" ^^^ intOf(ZX16) |
                          "sl16" ^^^ intOf(SL16) |
                          "sr16" ^^^ intOf(SR16))

  def jop: Parser[Int] = ("jez" ^^^ intOf(JEZ) |
                          "jlt" ^^^ intOf(JLT) |
                          "jle" ^^^ intOf(JLE) |
                          "jnz" ^^^ intOf(JNZ) |
                          "jge" ^^^ intOf(JGE) |
                          "jgt" ^^^ intOf(JGT) |
                          "jmp" ^^^ intOf(JMP) |
                          "jal" ^^^ intOf(JAL) |
                          "nop" ^^^ intOf(NOP))

  def xop: Parser[Int] = ("xad" ^^^ intOf(XAD) |
                          "xsd" ^^^ intOf(XSD) |
                          "xld" ^^^ intOf(XLD) |
                          "xsm" ^^^ intOf(XSM) |
                          "xsa" ^^^ intOf(XSA) |
                          "xla" ^^^ intOf(XLA))

  def instr: Parser[Int] = (opc ~ number ^^ { case o~n => ((o << OPERAND_BITS) |
                                                           (n & ((1 << OPERAND_BITS)-1))) } |
                            uop ^^ { intOf(UOP) << OPERAND_BITS | _ } |
                            jop ^^ { intOf(JOP) << OPERAND_BITS | _ } |
                            xop ^^ { intOf(XOP) << OPERAND_BITS | _ } )

  def prog: Parser[List[Int]] = rep(instr)

  def parseFile(fileName: String): List[Int] = {
    val file = scala.io.Source.fromFile(fileName)
    parseAll(prog, file.reader) match {
      case Success(result, _) => result
      case failure : NoSuccess => scala.sys.error(failure.msg)
    }
  }

  def main(args: Array[String]) {
    val data = parseFile(args(0))
    val out = new java.io.PrintStream(new java.io.FileOutputStream(args(1)))
    val pat = "%0"+((INSTR_BITS+3)/4)+"x"
    data.map { i => out.println(pat.format(i)) }
  }
}
