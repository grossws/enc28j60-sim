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

class Spi(object):
    def __init__(self, device, msb_first=True):
        self._set(False, False, False)
        self._so = True

        self._msb_first = msb_first

        self._dev = device
        device.spi_cs(self._cs)

        self._tx = 0xff
        self._rx = 0x00
        self._tx_n = 0 # current bit
        self._rx_n = 0 # current bit

    # return so bit
    def clock(self, cs, si, sck):
        if self._is_re(self._cs, cs):
            print "cs"
            self._dev.spi_cs(True)
        elif self._is_fe(self._cs, cs):
            print "!cs"
            self._dev.spi_cs(False)

        if not cs:
            return True

        if self._is_re(self._sck, sck):
            print "rd"
            self._read(si)

        if self._is_fe(self._sck, sck):
            print "wr"
            self._write()

        if self._tx_n == 8 and self._rx_n == 8:
            tx = self._dev.spi_rxtx(self._rx)
            print "tx:%02x rx:%02x" % (sel._tx, self._rx)
            self._tx = tx
            self._rx_n = self._tx_n = 0
            self._rx = 0x00

        self._set(cs, si, sck)
        return self._so

    def _set(self, cs, si, sck):
        self._cs = cs
        self._si = si
        self._sck = sck

    def _read(self, si):
        if si:
            data = 1
        else:
            data = 0

        if self._msb_first:
            self._rx |= data << (7 - self._rx_n)
        else:
            self._rx |= data << self._rx_n

    def _write(self):
        data = self._tx
        if self._msb_first:
            res = (data & (1 << (7 - self._tx_n))) != 0
        else:
            res = (data & (1 << self._tx_n)) != 0

        self._tx_n += 1
        return res

    # is raising edge
    def _is_re(self, old_pin, new_pin):
        return not old_pin and new_pin

    # is falling edge
    def _is_fe(self, old_pin, new_pin):
        return old_pin and not new_pin


# vim:ts=4:sw=4:sts=4
