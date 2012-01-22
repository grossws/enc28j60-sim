# Copyright (c) 2012, Konstantin Gribov. All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# This file defines internal model for enc28j60
# It's memory, registers etc

import fcntl
import os
import tuntap

class Device(object):
    # States:
    #           idle - waiting for CS
    SPI_IDLE = 0
    #           cmd_wait - waiting for command
    SPI_CMD = 1
    #           reset - after SC command, don't respond at all
    SPI_RESET = 2
    #           rbm_data - waiting dummy byte to read from buffer
    SPI_RBM = 3
    #           wbm_data - waiting byte to write into buffer
    SPI_WBM = 4
    #           wcr/bfs/bfc_data - waiting data byte for corresponging command
    SPI_WCR = 5
    SPI_BFS = 6
    SPI_BFC = 7
    #           rcr_eth_data - waiting dummy byte to read command register
    SPI_RCR_ETH = 8
    #           rcr_mac/phy_dummy - waiting dummy byte when reading MAC/PHY register to return dummy byte
    SPI_RCR_MACPHY_DUMMY = 9
    #           rcr_mac/phy_data - waiting dummy byte to read MAC/PHY register
    SPI_RCR_MACPHY = 10


    # command code/mask
    #           SC, RBM, WBM, WCR, BFS, BFC, RCR_ETH, RCR_MAC/PHY
    CMD_SC = 0xff
    CMD_RBM = 0x3a
    CMD_WBM = 0x7a
    CMD_WCR = 0x40 # mask
    CMD_RCR = 0x00 # mask
    CMD_BFS = 0x80 # mask
    CMD_BFC = 0xa0 # mask


    COMMON_REGISTER_START = 0x1B

    RTU_FRAME = 1500



    def __init__(self, tap_dev="tap0"):
        # 0x0000 to 0x1FFF -- internal buffer
        self._buffer = bytearray(8192)

        # registers: 0x1B to 0x1F are common registers,
        # so 0x3B-0x3F, 0x5B-0x5F, 0x7B-0x7F are mapped to 0x1B-0x1F
        self._regs = bytearray(128)

        self._rx_start = 0
        self._rx_ptr = 0
        self._rx_end = 0

        self._tx_start = 0
        self._tx_ptr = 0
        self._tx_end = 0

        self._rx_rd_ptr = 0
        self._rx_wr_ptr = 0

        # current state:
        self._spi_state = Device.SPI_IDLE
        self._spi_reg = 0x00
        self._spi_data = 0xff

        self._init_tap(tap_dev)
        self._init_reg_callback()
        self._rx_num = 0

        #self.reset()

    def _init_tap(self, dev):
        f = tuntap.tap_open(dev)
        fd = f.fileno()

        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

        self._tap = f

    def clock(self):
        self._rx_packet()

    # Low-level ops:
    def read_control_register(self, reg):
        return self._regs[self._get_register_addr(reg)]

    def read_buffer_data(self):
        data = self._buffer[self._rx_ptr]

        self._rx_ptr += 1
        if self._rx_ptr > self._rx_end:
            self._rx_ptr = self._rx_start

        return data

    def write_control_register(self, reg, data):
        self._regs[self._get_register_addr(reg)] = data & 0xff
        self._analyze_register_write(reg)

    def write_buffer_data(self, data):
        self._buffer[self._tx_ptr] = data & 0xff

        self._tx_ptr += 1
        if self._tx_ptr > self._tx_end:
            self._tx_ptr = self._tx_start

    def bit_field_set(self, reg, data):
        mask = data & 0xff
        self._regs[self._get_register_addr(reg)] |= mask
        self._analyze_register_write(reg)

    def bit_field_clear(self, reg, data):
        mask = ~(data & 0xff)
        self._regs[self._get_register_addr(reg)] &= mask
        self._analyze_register_write(reg)

    def soft_reset(self):
        # set reset flag (as min)
        pass

    def _get_register_addr(self, adr):
        if adr & 0x1f >= COMMON_REGISTER_START:
            return adr & 0x1f
        else:
            return adr & 0x7f

    def _analyze_register_write(self, reg):
        adr = self._get_register_addr(reg)
        if adr in self._reg_callback:
            self._reg_callback[adr]()


    # SPI interactions:
    # chip was selected (if cs = True)
    def spi_cs(self, cs):
        if cs and self._spi_state == Device.SPI_IDLE:
            self._spi_state = Device.SPI_CMD
        elif not cs:
            prev_spi_state = self._spi_state
            self._spi_state = Device.SPI_IDLE

    # recieve a byte from SPI (returns sended byte or None if can't send)
    def spi_rxtx(self, data):
        spi_data = self._spi_data
        self._spi_data = self._spi_parse_state(data)
        print "rx:%02x tx:%02x" % (data, spi_data)
        return spi_data

    def _spi_parse_state(self, data):
        if self._spi_state == Device.SPI_IDLE:
            pass
        elif self._spi_state == Device.SPI_CMD:
            return self._spi_parse_cmd(data)
        elif self._spi_state == Device.SPI_RESET:
            self._spi_state = Device.SPI_IDLE
        elif self._spi_state == Device.SPI_RBM:
            return self.read_buffer_data()
        elif self._spi_state == Device.SPI_WBM:
            self.write_buffer_data(data)
            return 0xff
        elif self._spi_state == Device.SPI_WCR:
            self.write_control_register(self._spi_reg, data)
            self._spi_state = Device.SPI_CMD
            return 0xff
        elif self._spi_state == Device.SPI_BFS:
            self.bit_field_set(self._spi_reg, data)
            self._spi_state = Device.SPI_CMD
            return 0xff
        elif self._spi_state == Device.SPI_BFC:
            self.bit_field_clear(self._spi_reg, data)
            self._spi_state = Device.SPI_CMD
            return 0xff
        elif self._spi_state == Device.SPI_RCR_ETH:
            self._spi_state = Device.SPI_CMD
            return 0xff
        elif self._spi_state == Device.SPI_RCR_MACPHY_DUMMY:
            self._spi_state = Device.SPI_RCR_MACPHY
            return self.read_control_register(self._spi_reg)
        elif self._spi_state == Device.SPI_RCR_MACPHY:
            self._spi_state = Device.SPI_CMD
            return 0xff

    def _spi_parse_cmd(self, data):
        if data == Device.CMD_SC:
            self._spi_state = Device.SPI_RESET
            self.soft_reset()
            return 0xff
        elif data == Device.CMD_RBM:
            self._spi_state = Device.SPI_RBM
            return self.read_buffer_data()
        elif data == Device.CMD_WBR:
            self._spi_state = Device.SPI_WBM
            return 0xff
        elif data & 0xe0 == Device.CMD_WCR:
            self._spi_reg = self._get_current_bank() | (data & 0x1f)
            self._spi_state = Device.SPI_WCR
            return 0xff
        elif data & 0xe0 == Device.CMD_BFS:
            self._spi_reg = self._get_current_bank() | (data & 0x1f)
            self._spi_state = Device.SPI_BFS
            return 0xff
        elif data & 0xe0 == Device.CMD_BFC:
            self._spi_reg = self._get_current_bank() | (data & 0x1f)
            self._spi_state = Device.SPI_BFC
            return 0xff
        elif data & 0xe0 == Device.CMD_RCR:
            self._spi_reg = self._get_current_bank() | (data & 0x1f)
            if self._is_macphy_reg(self._spi_reg):
                self._spi_state = Device.SPI_RCR_MACPHY_DUMMY
                return 0xff
            else:
                self._spi_state = Device.SPI_RCR_ETH
                return self.read_control_register(self._spi_reg)

        raise Exception('unknown command')


    def _get_current_bank(self):
        econ1 = self.read_control_register(0x1f)
        return (econ1 & 0x03) << 5

    def _is_macphy_reg(self, adr):
        if adr >= 0x40 and adr <= 0x65:
            return True
        elif adr == 0x6A:
            return True
        else:
            return False

    def _init_reg_callback(self):
        cb = {}
        cb[0x1f] = Device._cb_ECON1
        cb[0x1e] = Device._cb_ECON2

        cb[0x60] = Device._cb_MAADR
        cb[0x61] = Device._cb_MAADR
        cb[0x62] = Device._cb_MAADR
        cb[0x63] = Device._cb_MAADR
        cb[0x64] = Device._cb_MAADR
        cb[0x65] = Device._cb_MAADR

        cb[0x00] = cb[0x01] = Device._cb_ERDPT
        cb[0x08] = cb[0x09] = Device._cb_ERXST
        cb[0x0a] = cb[0x0b] = Device._cb_ERXND

        cb[0x0c] = cb[0x0d] = Device._cb_ERXRDPT
        cb[0x0e] = cb[0x0f] = Device._cb_ERXWRPT

        cb[0x02] = cb[0x03] = Device._cb_EWRPT
        cb[0x04] = cb[0x05] = Device._cb_ETXST
        cb[0x06] = cb[0x07] = Device._cb_ETXND

        self._reg_callback = cb

    def _cb_ECON1(self):
        if self._regs[0x1f] & 0x08:
            self._tx_packet()

    def _cb_ECON2(self):
        if self._regs[0x1e] & 0x40 and self._rx_num > 0:
            self._rx_num -= 1


    def _cb_MAADR(self):
        self._mac_addr[0] = self.regs[0x61]
        self._mac_addr[1] = self.regs[0x60]
        self._mac_addr[2] = self.regs[0x63]
        self._mac_addr[3] = self.regs[0x62]
        self._mac_addr[4] = self.regs[0x65]
        self._mac_addr[5] = self.regs[0x64]


    def _cb_ERDPT(self):
        self._rx_ptr = self._regs[0x01] << 8 | self._regs[0x00]

    def _cb_ERXST(self):
        self._rx_start = self._regs[0x09] << 8 | self._regs[0x08]

    # including bounds?
    def _cb_ERXND(self):
        self._rx_end = self._regs[0x0b] << 8 | self._regs[0x0a]


    def _cb_EWRPT(self):
        self._tx_ptr = self._regs[0x03] << 8 | self._regs[0x02]

    def _cb_ETXST(self):
        self._tx_start = self._regs[0x05] << 8 | self._regs[0x04]

    # including bounds?
    def _cb_ETXND(self):
        self._tx_end = self._regs[0x07] << 8 | self._regs[0x06]


    def _cb_ERXRDPT(self):
        self._rx_rd_ptr = self._regs[0x0d] << 8 | self._regs[0x0c]

    def _cb_ERXWRPT(self):
        self._rx_wr_ptr = self._regs[0x0f] << 8 | self._regs[0x0e]


    def _tx_packet(self):
        packet = self._buffer[self._tx_start+1:self._tx_end+1]
        self._tap.write(packet)

    def _rx_packet(self):
        packet = self._tap.read(Device.RTU_FRAME)

        if packet is None:
            return

        packet = bytearray(packet)

        if self._regs[0x39] == 0xff:
            pass

        rxlen = len(packet)
        ptr = self._rx_wr_ptr

        if ptr & 0x01 != 0:
            ptr += 1

        ptr_next_raw = ptr + 6 + rxlen

        if ptr_next_raw & 0x01 != 0:
            ptr_next_raw += 1

        ptr_next = ptr_next_raw
        if ptr_next_raw > self._rx_end:
            ptr_next = ptr_next_raw + self._rx_start - self._rx_end - 1

        for i in range(6):
            packet.insert(0, 0)

        packet[0] = ptr_next & 0xff
        packet[1] = (ptr_next >> 8) & 0xff
        packet[2] = rxlen & 0xff
        packet[3] = (rxlen >> 8) & 0xff
        packet[4] = 0x80 # OK status ONLY
        packet[5] = 0x00 #

        if ptr < self._rx_rd_ptr and ptr_next < self._rx_rd_ptr:
            write = True
        elif ptr > self._rx_rd_ptr and ptr_next_raw <= self._rx_end:
            write = True
        elif ptr > self._rx_rd_ptr and ptr_next < self._rx_rd_ptr:
            write = True
        else:
            write = False

        if write:
            self._regs[0x39] += 1
            ptr_next_raw = ptr + 6 + rxlen
            if ptr_next_raw <= self._rx_end:
                self._buffer[ptr : ptr_next_raw] = packet
            else:
                self._buffer[ptr : self._rx_end + 1] = packet[0 : self._rx_end + 1 - ptr]
                self._buffer[self._rx_start : ptr_next_raw + self._rx_start - self._rx_end - 1] = packet[self._rx_end + 1 - ptr : ]


# vim:ts=4:sw=4:sts=4
