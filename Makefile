PROJECTS_DIR = projects
BUILD_DIR = build
export BOARD ?= murata_discovery


APP_LIST = $(wildcard $(PROJECTS_DIR)/*)

all:
	@echo "Usage:"
	@echo "   $$ make [BOARD=<board>] <target>								to build <target>"
	@echo "   $$ make [BOARD=<board>] [PICOFLASH_TTY=<tty>] <target>-flash					to build and flash <target>"
	@echo "   $$ make [BOARD=<board>] buildall								to build all targets"
	@echo "   $$ make [BOARD=<board>] CREDENTIALS=\"<credentials>\" FlashPicoCred-flash			to inject the PicoWAN credentials into the board"
	@echo "   $$ make <target>-clean									to clean <target>"
	@echo "   $$ make clean											to clean all targets"
	@echo
	@echo "   Available boards: \"discovery_stm32l1\", \"picotag\", \"murata_sychip\", \"murata_discovery\", \"murata_module\", \"nucleo\", \"picoshield\""
	@echo
	@echo "   Possible targets: $(patsubst $(PROJECTS_DIR)/%/,\"%\",$(filter %/, $(wildcard $(PROJECTS_DIR)/*/)))"
	@echo

buildall:	$(patsubst $(PROJECTS_DIR)/%/,%,$(filter %/, $(wildcard $(PROJECTS_DIR)/*/)))
	@echo "Done."

%-flash:
	$(MAKE) -C $(PROJECTS_DIR)/$(patsubst %-flash,%,$@) flash
	@echo
	@echo " -> Project $(patsubst %-flash,%,$@) has been flashed"
	@echo

%-clean:
	$(MAKE) -C $(PROJECTS_DIR)/$(patsubst %-clean,%,$@) clean
	@echo
	@echo " -> Project $(patsubst %-clean,%,$@) has been cleaned"
	@echo

%:
	$(MAKE) -C $(PROJECTS_DIR)/$@
	@echo
	@echo " -> Binary available at $(PWD)/$(PROJECTS_DIR)/$@/$(BUILD_DIR)/$(BOARD)/$@.bin"
	@echo

clean:	$(patsubst $(PROJECTS_DIR)/%/,%-clean,$(filter %/, $(wildcard $(PROJECTS_DIR)/*/)))

.PHONY: all clean
