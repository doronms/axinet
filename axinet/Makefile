obj-m += mcu_pcie.o
obj-m += mcu_axienet.o

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

# Default debug level for loading
DEBUG ?= 2
POLL ?= 1
POLL_MS ?= 1

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

# Load with polling mode (default)
load:
	sudo rmmod mcu_axienet 2>/dev/null; true
	sudo rmmod mcu_pcie 2>/dev/null; true
	sleep 1
	sudo insmod mcu_pcie.ko debug=$(DEBUG)
	sudo insmod mcu_axienet.ko debug=$(DEBUG) poll_mode=$(POLL) poll_interval_ms=$(POLL_MS)

# Load with IRQ mode
load-irq:
	sudo rmmod mcu_axienet 2>/dev/null; true
	sudo rmmod mcu_pcie 2>/dev/null; true
	sleep 1
	sudo insmod mcu_pcie.ko debug=$(DEBUG)
	sudo insmod mcu_axienet.ko debug=$(DEBUG) poll_mode=0

unload:
	sudo rmmod mcu_axienet 2>/dev/null; true
	sudo rmmod mcu_pcie 2>/dev/null; true

reload: unload load

# Quick test
test:
	@echo "=== Interface Status ==="
	ip link show eth0 2>/dev/null || echo "eth0 not found"
	@echo ""
	@echo "=== Ethtool ==="
	sudo ethtool eth0 2>/dev/null || true
	@echo ""
	@echo "=== DMA State ==="
	cat /sys/class/net/eth0/device/dma_state 2>/dev/null || echo "dma_state not available"
	@echo ""
	@echo "=== Recent dmesg ==="
	dmesg | tail -30

.PHONY: all clean load load-irq unload reload test
