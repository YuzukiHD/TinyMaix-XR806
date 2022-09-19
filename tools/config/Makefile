MENUBUILD   := ./build

MENUCFLAGS  := -I/usr/include/ncurses -DCURSES_LOC="<ncurses.h>" -DLOCALE -Wall -Wmissing-prototypes -Wstrict-prototypes -O2 -fomit-frame-pointer

ifeq ($(shell uname -s),Linux)
MENULDFLAGS := -lncurses
else
MENULDFLAGS := -lncurses -lintl
endif

HOSTCC      := gcc

common-objs := $(addprefix $(MENUBUILD)/, confdata.o expr.o lexer.lex.o \
		   parser.tab.o preprocess.o symbol.o util.o)

lxdialog    := $(addprefix $(MENUBUILD)/lxdialog/, \
		     checklist.o inputbox.o menubox.o textbox.o util.o yesno.o)

all: mconf

$(MENUBUILD)/%.o: %.c
	@mkdir -p $(MENUBUILD)
	@mkdir -p $(MENUBUILD)/lxdialog
	@echo "HOSTCC $<"; $(HOSTCC) -c $(MENUCFLAGS) $< -o $@

mconf: $(MENUBUILD)/mconf.o $(lxdialog) $(common-objs)
	@echo "HOSTLD $<"; $(HOSTCC) -o $@ $^ $(MENULDFLAGS); \
	chmod 777 $@

conf: $(MENUBUILD)/conf.o $(common-objs)
	@echo "HOSTLD $<"; $(HOSTCC) -o $@ $^ $(MENULDFLAGS); \
	chmod 777 $@

clean :
	-@rm mconf conf; rm -rf $(MENUBUILD)/*
