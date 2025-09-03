# Makefile for SHT3x kernel module
obj-m += sht3x.o

# Default kernel directory if not specified
KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build

# Build targets
all:
	$(MAKE) -C $(KERNEL_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) clean

install:
	$(MAKE) -C $(KERNEL_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) modules_install

help:
	@echo "Available targets:"
	@echo "  all       : Build the kernel module"
	@echo "  clean     : Clean build artifacts"
	@echo "  install   : Install the kernel module"
	@echo "Variables:"
	@echo "  KERNEL_DIR    : Kernel source directory (default: /lib/modules/`uname -r`/build)"
	@echo "  ARCH          : Target architecture (e.g., arm, arm64)"
	@echo "  CROSS_COMPILE : Cross-compiler prefix (e.g., arm-linux-gnueabihf-)"