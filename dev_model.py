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

class Device(object):
    COMMON_REGISTER_START = 0x1B



    def __init__(self):
        # 0x0000 to 0x1FFF -- internal buffer
        self._buffer = bytearray(4096)

        # registers: 0x1B to 0x1F are common registers,
        # so 0x3B-0x3F, 0x5B-0x5F, 0x7B-0x7F are mapped to 0x1B-0x1F
        self._regs = bytearray(128)

        self._rx_start = 0
        self._rx_ptr = 0
        self._rx_size = 0

        self._tx_start = 0
        self._tx_ptr = 0
        self._tx_size = 0

        # current state:
        self._spi_state = SPI_IDLE
        self._spi_reg = 0x00
        self._spi_data = 0xff
        reset(self)


    # Low-level ops:
    def read_control_register(self, reg):
        return self._regs[self._get_register_addr(reg)]

    def read_buffer_data(self):
        data = self._buffer[self._rx_ptr]

        self._rx_ptr += 1
        if self._rx_ptr - self._rx_start >= self._rx_size:
            self._rx_ptr = self._rx_size

        return data

    def write_control_register(self, reg, data):
        self._regs[self._get_register_addr(reg)] = data & 0xff

    def write_buffer_data(self, data):
        self._buffer[self._tx_ptr] = data & 0xff

        self._tx_ptr += 1
        if self._tx_ptr - self._tx_start >= self._tx_size:
            self._tx_ptr = self._tx_size

    def bit_field_set(self, adr, data):
        mask = data & 0xff
        self._regs[self._get_register_addr(reg)] |= mask

    def bit_field_clear(self, adr, data):
        mask = ~(data & 0xff)
        self._regs[self._get_register_addr(reg)] &= mask

    def _get_register_addr(self, adr):
        if adr & 0x1f >= COMMON_REGISTER_START:
            return adr & 0x1f
        else:
            return adr & 0x7f

    def soft_reset(self):
        # set reset flag (as min)
        pass


    # SPI interactions:
    # chip was selected (if cs = True)
    def spi_cs(self, cs):
        if cs and _spi_state == SPI_IDLE:
            _spi_state = SPI_CMD
        elif not cs:
            _spi_state = SPI_IDLE

    # recieve a byte from SPI (returns sended byte or None if can't send)
    def spi_rxtx(self, data):
        spi_data = self._spi_data
        self._spi_parse_state(data)
        return spi_data

    def _spi_parse_state(self, data):
        if self._spi_state == SPI_IDLE:
            pass
        elif self._spi_state == SPI_CMD:
            return self._spi_parse_cmd(data)
        elif self._spi_state == SPI_RESET:
            self._spi_state = SPI_IDLE
        elif self._spi_state == SPI_RBM:
            return self.read_buffer_data()
        elif self._spi_state == SPI_WBM:
            self.write_buffer_data(data)
            return 0xff
        elif self._spi_state == SPI_WCR:
            self.write_control_register(self._spi_reg, data)
            self._spi_state = SPI_CMD
            return 0xff
        elif self._spi_state == SPI_BFS:
            self.bit_field_set(self._spi_reg, data)
            self._spi_state = SPI_CMD
            return 0xff
        elif self._spi_state == SPI_BFC:
            self.bit_field_clear(self._spi_reg, data)
            self._spi_state = SPI_CMD
            return 0xff
        elif self._spi_state == SPI_RCR_ETH:
            self._spi_state = SPI_CMD
            return 0xff
        elif self._spi_state == SPI_RCR_MACPHY_DUMMY:
            self._spi_state = SPI_RCR_MACPHY
            return self.read_control_register(self._spi_reg)
        elif self._spi_state == SPI_RCR_MACPHY:
            self._spi_state = SPI_CMD
            return 0xff

    def _spi_parse_cmd(self, data):
        if data == CMD_SC:
            self._spi_state = SPI_RESET
            self.soft_reset()
            return 0xff
        elif data == CMD_RBM:
            self._spi_state = SPI_RBM
            return self.read_buffer_data()
        elif data == CMD_WBR:
            self._spi_state = SPI_WBM
            return 0xff
        elif data & 0xe0 == CMD_WCR:
            self._spi_reg = self._get_current_bank() | (data & 0x1f)
            self._spi_state = SPI_WCR
            return 0xff
        elif data & 0xe0 == CMD_BFS:
            self._spi_reg = self._get_current_bank() | (data & 0x1f)
            self._spi_state = SPI_BFS
            return 0xff
        elif data & 0xe0 == CMD_BFC:
            self._spi_reg = self._get_current_bank() | (data & 0x1f)
            self._spi_state = SPI_BFC
            return 0xff
        elif data & 0xe0 == CMD_RCR:
            self._spi_reg = self._get_current_bank() | (data & 0x1f)
            if self._is_macphy_reg(self._spi_reg):
                self._spi_state = SPI_RCR_MACPHY_DUMMY
                return 0xff
            else:
                self._spi_state = SPI_RCR_ETH
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


# vim:ts=4:sw=4:sts=4
