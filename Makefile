
TARGET = fly


TOUCH		:= $(shell touch *)
CC		:= gcc
LINKER		:= gcc -o
CFLAGS		:= -c -Wall -g
LFLAGS		:= -ljson-c -lm -lrt -lpthread -lroboticscape

SOURCES		:= $(wildcard *.c)
INCLUDES	:= $(wildcard *.h)
OBJECTS		:= $(SOURCES:$%.c=$%.o)

prefix		:= /usr/local
RM		:= rm -f
INSTALL		:= install -o root -g root -m 4755
INSTALLDIR	:= install -d -m 755 

LINK		:= ln -s -f
LINKDIR		:= /etc/roboticscape
LINKNAME	:= link_to_startup_program


# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) $(@) $(OBJECTS) $(LFLAGS)


# compiling command
$(OBJECTS): %.o : %.c
	@$(TOUCH) $(CC) $(CFLAGS) -c $< -o $(@)
	@echo "Compiled: "$<


all:
	$(TARGET)

debug:
	$(MAKE) $(MAKEFILE) DEBUGFLAG="-g -D DEBUG"
	@echo " "
	@echo "$(TARGET) Make Debug Complete"
	@echo " "

install: 
	@$(MAKE) --no-print-directory
	@$(INSTALL) $(TARGET) $(DESTDIR)$(prefix)/bin
	@echo "$(TARGET) Install Complete"
	
clean:
	@$(RM) $(OBJECTS)
	@$(RM) $(TARGET)
	@echo "$(TARGET) Clean Complete"

uninstall:
	@$(RM) $(DESTDIR)$(prefix)/bin/$(TARGET)
	@echo "$(TARGET) Uninstall Complete"

runonboot:
	@$(MAKE) install --no-print-directory
	@$(LINK) $(DESTDIR)$(prefix)/bin/$(TARGET) $(LINKDIR)/$(LINKNAME)
	@echo "$(TARGET) Set to Run on Boot"
	
