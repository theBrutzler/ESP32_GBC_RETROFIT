# Command-line client
CMDLINE = GBCartRead_v1.8_Rev1.3_C_Interface.exe

# By default, build the firmware and command-line client
all: $(CMDLINE)

# One-liner to compile the command-line client
$(CMDLINE): GBCartRead_v1.8_Rev1.3_C_Interface.c
	gcc -O1 -Wall -lm -v -save-temps -g3 -ggdb3 -pg GBCartRead_v1.8_Rev1.3_C_Interface.c rs232.c rs232.h -o $(CMDLINE)

# Housekeeping if you want it
clean:
	$(RM) *.exe
	$(RM) *.i
	$(RM) *.o
	$(RM) *.s
