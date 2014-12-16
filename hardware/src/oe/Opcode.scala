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
 * Opcode definitions for Ø
 * 
 * Author: Wolfgang Puffitsch (wpuffitsch@gmail.com)
 *
 */

package oe

import Chisel._
import Node._

object Opcode {
  val LDI = Bits("b0000")
  val LDL = Bits("b0001")

  val LDO = Bits("b0010")
  val STO = Bits("b0011")

  val ADD = Bits("b0100")
  val SUB = Bits("b0101")
  val AND = Bits("b0110")
  val IOR = Bits("b0111")
  val XOR = Bits("b1000")
  val MUL = Bits("b1001")

  val UOP = Bits("b1010")

  val BNZ = Bits("b1101")
  val JOP = Bits("b1110")
  val XOP = Bits("b1111")


  val SRL = Bits("b0000")
  val SRA = Bits("b0001")
  val NOT = Bits("b0010")
  val NEG = Bits("b0011")

  val CMP = Bits("b0100")
  val ADC = Bits("b0101")

  val SX8 = Bits("b1000")
  val ZX8 = Bits("b1001")
  val SL8 = Bits("b1010")
  val SR8 = Bits("b1011")

  val SX16 = Bits("b1100")
  val ZX16 = Bits("b1101")
  val SL16 = Bits("b1110")
  val SR16 = Bits("b1111")


  val JEZ = Bits("b0000")
  val JLT = Bits("b0001")
  val JLE = Bits("b0010")
  val JNZ = Bits("b0011")
  val JGE = Bits("b0100")
  val JGT = Bits("b0101")
  val JMP = Bits("b0110")
  val JAL = Bits("b0111")
  val NOP = Bits("b1111")


  val XAD = Bits("b0000")
  val XSD = Bits("b0001")
  val XLD = Bits("b0010")
  val XSM = Bits("b0011")
  val XSA = Bits("b0100")
  val XLA = Bits("b0101")
}
