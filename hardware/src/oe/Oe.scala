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
 * Implementation of Ø
 * 
 * Author: Wolfgang Puffitsch (wpuffitsch@gmail.com)
 *
 */

package oe

import Chisel._
import Node._

import Config._
import Opcode._
import Assembler._
import ocp._
import io._

class OeCore(asmFile : String) extends Module {
  val io = new Bundle {
    val ocp = new OcpCoreMasterPort(ADDR_WIDTH, DATA_WIDTH)
  }

  // constants for sub-word handling
  val ext8Bit = if (ACC_WIDTH > 8) 7 else ACC_WIDTH-1
  val ext16Bit = if (ACC_WIDTH > 16) 15 else ACC_WIDTH-1
  val sh8Bits = UInt(if (ACC_WIDTH > 8) 8 else ACC_WIDTH)
  val sh16Bits = UInt(if (ACC_WIDTH > 16) 16 else ACC_WIDTH)
  val shAddrBits = UInt(if (ACC_WIDTH > ADDR_WIDTH) ADDR_WIDTH else ACC_WIDTH)
  val shDataBits = UInt(if (ACC_WIDTH > DATA_WIDTH) DATA_WIDTH else ACC_WIDTH)

  // the accumulator
  val accReg = Reg(UInt(width = ACC_WIDTH))
  // the registers
  val regMem = Mem(UInt(width = REG_WIDTH), REG_COUNT)
  // the instruction ROM
  val instrRom = Vec(parseFile(asmFile).map(UInt(_, width = INSTR_BITS)))
  // the program counter
  val pcNext = UInt()
  val pcReg = Reg(init = UInt(-1, width = log2Up(instrRom.length)), next = pcNext)
  // flag registers
  val carryReg    = Reg(UInt(width = 1))
  val zeroReg     = Reg(Bool())
  val negativeReg = Reg(Bool())
  // the memory interface registers
  val addrReg = Reg(UInt(width = ADDR_WIDTH))
  val dataReg = Reg(UInt(width = DATA_WIDTH))
  val maskReg = Reg(UInt(width = (DATA_WIDTH+7)/8))

  // function to conditionally include operations
  def hasOp(op: Bits) =
    instrRom.map(_(INSTR_BITS-1, OPERAND_BITS).litValue().intValue
                 == op.litValue().intValue).reduce(_||_)
  def isOp(op: Bits)(block: => Unit) = 
    if (hasOp(op)) {
      println("OP " + op.litValue())
      is(op)(block)
    }

  def hasSubOp(op: Bits, sub: Bits) =
    instrRom.map(i => (i(INSTR_BITS-1, OPERAND_BITS).litValue().intValue
                       == op.litValue().intValue &&
                       i(OPERAND_BITS-1, 0).litValue().intValue
                       == sub.litValue().intValue)).reduce(_||_)
  def isSubOp(op: Bits, sub: Bits)(block: => Unit) =
    if (hasSubOp(op, sub)) {
      println("SUBOP " + op.litValue() + " " + sub.litValue())
      is(sub)(block)
    }
  
  def isUOp(op: Bits)(block: => Unit) = isSubOp(UOP, op)(block)
  def isJOp(op: Bits)(block: => Unit) = isSubOp(JOP, op)(block)
  def isXOp(op: Bits)(block: => Unit) = isSubOp(XOP, op)(block)

  // fetch
  val instrRegBuf = Reg(next = instrRom(pcNext)) // place register at end of ROM
  val instrReg = Reg(next = instrRegBuf)

  // decode/register access
  val regReg = Reg(next = regMem(instrReg(OPERAND_BITS-1, 0)))
  val immReg = Reg(next = instrReg(OPERAND_BITS-1, 0))
  val opReg = Reg(next = instrReg(INSTR_BITS-1, OPERAND_BITS))
  val stallOpReg = Reg(next = instrReg === XOP ## XSA || instrReg === XOP ## XLA)

  // execute
  val store = new Bool()
  store := Bool(false)

  val jump = Bool()
  jump := Bool(false)

  val stall = Bool()
  val stallReg = Reg(next = stall)
  stall := Bool(false)

  io.ocp.M.Cmd := OcpCmd.IDLE
  io.ocp.M.Addr := addrReg
  io.ocp.M.Data := dataReg
  io.ocp.M.ByteEn := maskReg

  when (stallOpReg) {
    stall := io.ocp.S.Resp != OcpResp.DVA
  }

  switch (opReg) {
    isOp(LDI) { accReg := immReg.toSInt }
    isOp(LDL) { accReg := (accReg << UInt(OPERAND_BITS)) | immReg }

    isOp(LDO) { accReg := regReg }
    isOp(STO) { store := Bool(true) }

    isOp(ADD) { val add = (UInt("b0") ## accReg) + regReg
               accReg := add(ACC_WIDTH-1, 0); carryReg := add(ACC_WIDTH) }
    isOp(SUB) { val sub = (UInt("b0") ## accReg) - regReg
               accReg := sub(ACC_WIDTH-1, 0); carryReg := sub(ACC_WIDTH) }
    isOp(AND) { accReg := accReg & regReg }
    isOp(IOR) { accReg := accReg | regReg }
    isOp(XOR) { accReg := accReg ^ regReg }
    isOp(MUL) { accReg := accReg * regReg }

    isOp(UOP) {
      switch(immReg) {
        isUOp(SRL) { accReg := accReg >> UInt(1) }
        isUOp(SRA) { accReg := accReg.toSInt >> UInt(1) }
        isUOp(NOT) { accReg := ~accReg }
        isUOp(NEG) { accReg := -accReg }

        isUOp(CMP) { zeroReg := accReg === UInt(0);
                     negativeReg := accReg(ACC_WIDTH-1) != UInt(0) }
        isUOp(ADC) { val add = (UInt("b0") ## accReg) + carryReg
                    accReg := add(ACC_WIDTH-1, 0); carryReg := add(ACC_WIDTH) }

        isUOp(SX8) { accReg := accReg(ext8Bit, 0).toSInt }
        isUOp(ZX8) { accReg := accReg(ext8Bit, 0) }
        isUOp(SL8) { accReg := accReg << sh8Bits }
        isUOp(SR8) { accReg := accReg >> sh8Bits }

        isUOp(SX16) { accReg := accReg(ext16Bit, 0).toSInt }
        isUOp(ZX16) { accReg := accReg(ext16Bit, 0) }
        isUOp(SL16) { accReg := accReg << sh16Bits }
        isUOp(SR16) { accReg := accReg >> sh16Bits }
      }
    }

    isOp(JOP) {
      switch(immReg) {
        isJOp(JEZ) { jump := zeroReg }
        isJOp(JLT) { jump := negativeReg }
        isJOp(JLE) { jump := negativeReg || zeroReg }
        isJOp(JNZ) { jump := !zeroReg }
        isJOp(JGE) { jump := !negativeReg }
        isJOp(JGT) { jump := !negativeReg && !zeroReg }

        isJOp(JMP) { jump := Bool(true) }
        isJOp(JAL) { accReg := pcReg; jump := Bool(true) }
        isJOp(NOP) { }
      }
    }

    isOp(XOP) {
      switch(immReg) {
        isXOp(XAD) { addrReg := (addrReg << shAddrBits) | accReg }
        isXOp(XSD) { dataReg := (dataReg << shDataBits) | accReg }
        isXOp(XLD) { accReg := dataReg; dataReg := dataReg >> shDataBits }
        isXOp(XSM) { maskReg := accReg }
        isXOp(XSA) {
          io.ocp.M.Cmd := Mux(stallReg, OcpCmd.IDLE, OcpCmd.WR)
        }
        isXOp(XLA) {
          io.ocp.M.Cmd := Mux(stallReg, OcpCmd.IDLE, OcpCmd.RD)
          dataReg := io.ocp.S.Data
        }
      }
    }
  }

  // update program counter
  pcNext := Mux(jump, accReg, pcReg + !stall)

  // flush instructions after jump
  when (jump) {
    instrReg := JOP ## NOP
  }

  // write-back
  when (store) {
    regMem(immReg) := accReg
  }

  // stall for memory access
  when (stall) {
    accReg := accReg
    instrReg := instrReg
    regReg := regReg
    immReg := immReg
    opReg := opReg
    stallOpReg := stallOpReg
  }

  // suppress memory commands while resetting
  when (reset) {
    io.ocp.M.Cmd := OcpCmd.IDLE
  }
}

class Oe(asmFile : String) extends Module {
  val io = new Bundle with Leds.Pins

  val core = Module(new OeCore(asmFile))

  val ledsParams = Map("ledCount" -> "8")
  Leds.init(ledsParams)
  val leds = Leds.create(ledsParams)

  core.io.ocp <> leds.io.ocp
  leds.io.ledsPins <> io.ledsPins
}

class OeTop(asmFile : String) extends Module {
  val io = new Bundle with Leds.Pins

  val resetExt = Reg(next = reset)
  val resetInt = Reg(next = !resetExt)

  val oe = Module(new Oe(asmFile))
  oe.reset := resetInt
  io <> oe.io
}

class OeTester(c: OeTop) extends Tester(c) {
  for (t <- 0 to 1000) {
    peek(c.oe.core.pcReg)
    peek(c.oe.core.accReg)
    peek(c.io.ledsPins)
    step(1)
  }
}

object OeMain {
  def main(args: Array[String]): Unit = {

    val asmFile = args(0)
    val chiselArgs = args.slice(1, args.length)

    // chiselMain(chiselArgs, () => Module(new Oe(asmFile)))
    chiselMainTest(chiselArgs, () => Module(new OeTop(asmFile))) { c => new OeTester(c) }
  }
}
