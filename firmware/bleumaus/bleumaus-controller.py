#!/usr/bin/env python

from bluepy import btle
CMD = '\x21\x42\x35\x30\x37'
MAC = 'C7:2B:96:A5:A6:AC'
IFACE = 1
UUID_SRV     = '00001530-1212-efde-1523-785feabcd123'
UUID_RX_CHAR = '00001532-1212-efde-1523-785feabcd123'
RX_HANDLE = 15

dev = btle.Peripheral(deviceAddr=MAC, addrType="random", iface=IFACE)
dev.writeCharacteristic(15, CMD)

def main():
    pass

if __name__ == '__main__':
    main()
