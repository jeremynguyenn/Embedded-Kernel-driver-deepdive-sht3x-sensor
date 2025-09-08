# Makefile for SHT3x kernel module
# Enhanced with better cross-compile support and dependencies

obj-m += sht3x.o

# Default kernel directory if not specified
KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build

# Default architecture
ARCH ?= $(shell uname -m)

# Default cross-compile prefix
CROSS_COMPILE ?=

# Build targets
all: modules

modules:
	$(MAKE) -C $(KERNEL_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) clean
	$(RM) Module.symvers Module.markers modules.order

install: modules
	$(MAKE) -C $(KERNEL_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) modules_install
	depmod -a

uninstall:
	$(RM) /lib/modules/$(shell uname -r)/kernel/drivers/$(SLAVE_DEVICE_NAME).ko.gz
	depmod -a

help:
	@echo "Available targets:"
	@echo "  all       : Alias for 'modules' - Build the kernel module"
	@echo "  modules   : Build the kernel module"
	@echo "  clean     : Clean build artifacts"
	@echo "  install   : Install the kernel module and update depmod"
	@echo "  uninstall : Remove the installed module"
	@echo "Variables:"
	@echo "  KERNEL_DIR    : Kernel source directory (default: /lib/modules/\`uname -r\`/build)"
	@echo "  ARCH          : Target architecture (default: current, e.g., arm, arm64, x86_64)"
	@echo "  CROSS_COMPILE : Cross-compiler prefix (e.g., arm-linux-gnueabihf-)"
	@echo "Example for ARM cross-compile:"
	@echo "  make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- all"

.PHONY: all modules clean install uninstall help
