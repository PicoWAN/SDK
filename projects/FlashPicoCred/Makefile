# Firmware configuration (FWCFG)
# Available defines are -DCFG_CONSOLE_RX_ENABLED -DCFG_DISABLE_LOW_POWER_MODE -DCFG_CONSOLE_AT_PREFIX -DCFG_DISABLE_RADIO_IRQ_HANDLER -DCFG_CONSOLE_DEBUG_MAC_DISABLED
# FWCFG =

ifneq ($(MAKECMDGOALS),clean)
ifeq ($(CREDENTIALS),)
$(error CREDENTIALS must be set with the proper PicoWAN credentials !)
endif
FWCFG += -DCREDENTIALS=\"$(CREDENTIALS)\"
else
FWCFG += -DCREDENTIALS
endif

include ../projects.gmk

# Force main.c to be rebuilt to be sure the requested CREDENTIALS are used
build/$(BOARD)/main.o: .FORCE
.PHONY: .FORCE
FORCE:
