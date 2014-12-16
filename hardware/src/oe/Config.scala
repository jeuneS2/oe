package oe

import Chisel._
import Node._

object Config {
  val ACC_WIDTH = 8
  val REG_WIDTH = 8

  val OPERAND_BITS = 4
  val INSTR_BITS = 4 + OPERAND_BITS
  val REG_COUNT = 4

  val ADDR_WIDTH = 2
  val DATA_WIDTH = 8
}
