#!/bin/sh

echo "*** load axienet & pcie drivers *****"
insmod mcu_pcie.ko debug=1
insmod mcu_axienet.ko debug=2

echo "*** config ip 192.168.0.1 (10Mb/FD/ANEG=off) ***"
#ethtool -s eth0 speed 10 duplex full autoneg off
ifconfig eth0 192.168.0.1 netmask 255.255.255.0 up

echo "Done."
