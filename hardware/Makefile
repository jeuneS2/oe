# Get the hardware build dir from the parent make
HWBUILDDIR?=$(CURDIR)/build
SBT=java -Xmx1024M -Xss8M -XX:MaxPermSize=128M -jar sbt/sbt-launch.jar

# consider everything a source file
SRC=$(shell find src -name '*.scala')

# main class for Ø
MAIN=oe.OeMain

# file containing the assembly program
ASMFILE=../asm/blink8.asm

# build C++ and Verilog versions by default
all: verilog

verilog:
	$(SBT) "run-main $(MAIN) $(ASMFILE) --targetDir $(HWBUILDDIR) --backend v"

test:
	$(SBT) "run-main $(MAIN) $(ASMFILE) --targetDir $(HWBUILDDIR) --backend c --genHarness --compile --test --vcd"

assemble: $(ASMFILE:asm=hex) 

%.hex: %.asm
	$(SBT) "run-main oe.Assembler $< $@"

.FORCE:

.PHONY: all verilog
