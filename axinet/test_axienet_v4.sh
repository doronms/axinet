#!/bin/bash
# test_axienet_v4.sh - Test script for mcu_axienet v4.0-poll
# Run as root or with sudo

set -e

IFACE="eth0"
LOCAL_IP="192.168.1.100"
REMOTE_IP="192.168.1.1"
DEBUG=2

echo "============================================"
echo "  AXI Ethernet v4.0 Test (Polling Mode)"
echo "============================================"
echo ""

# Step 1: Build
echo "[1/8] Building..."
make clean && make
echo "      OK"
echo ""

# Step 2: Unload old
echo "[2/8] Unloading old modules..."
rmmod mcu_axienet 2>/dev/null || true
rmmod mcu_pcie 2>/dev/null || true
sleep 1
echo "      OK"
echo ""

# Step 3: Load new
echo "[3/8] Loading modules (poll_mode=1, debug=$DEBUG)..."
insmod mcu_pcie.ko debug=$DEBUG
insmod mcu_axienet.ko debug=$DEBUG poll_mode=1 poll_interval_ms=1
sleep 2
echo "      OK"
echo ""

# Step 4: Check interface
echo "[4/8] Checking interface..."
ip link show $IFACE
echo ""

# Step 5: Configure IP
echo "[5/8] Configuring IP..."
ip addr flush dev $IFACE 2>/dev/null || true
ip addr add ${LOCAL_IP}/24 dev $IFACE
ip link set $IFACE up
sleep 2
echo "      Configured: $LOCAL_IP"
echo ""

# Step 6: Check link state
echo "[6/8] Checking link & PHY..."
ethtool $IFACE 2>/dev/null || echo "      (ethtool not available)"
echo ""

# Step 7: Check DMA state
echo "[7/8] DMA state:"
cat /sys/class/net/$IFACE/device/dma_state 2>/dev/null || echo "      (not available)"
echo ""

# Step 8: Ping test
echo "[8/8] Ping test to $REMOTE_IP..."
echo "      (Press Ctrl+C to stop)"
echo "      Watch dmesg in another terminal: dmesg -w | grep mcu_axienet"
echo ""
ping -c 5 -W 2 $REMOTE_IP || echo "      Ping failed (check dmesg for TX/RX traces)"
echo ""

echo "============================================"
echo "  Debug commands:"
echo "============================================"
echo ""
echo "  # Watch live dmesg:"
echo "  dmesg -w | grep -E 'mcu_|axienet|ADIN'"
echo ""
echo "  # Check DMA state:"
echo "  cat /sys/class/net/$IFACE/device/dma_state"
echo ""
echo "  # Change debug level at runtime:"
echo "  echo 3 > /sys/module/mcu_axienet/parameters/debug"
echo ""
echo "  # Check MAC registers:"
echo "  dmesg | grep -E 'MAC|SPEED|PPST|RCW|TC '"
echo ""
echo "  # If RX still not working, check:"
echo "  #   1. dmesg for 'C2H_0 ERRORS' lines"
echo "  #   2. DMA state: C2H STAT should show BUSY (0x01)"
echo "  #   3. Credits posted should be > 0"
echo "  #   4. Are ARP replies coming from remote? (tcpdump on remote)"
echo ""
