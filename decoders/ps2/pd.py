##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2016 Daniel Schulte <trilader@schroedingers-bit.net>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

import sigrokdecode as srd
from collections import namedtuple

class Ann:
    BIT, START, STOP, PARITY_OK, ERROR, DATA, WORD, INHIBIT, ACK = range(9)

Bit = namedtuple('Bit', 'val cl ms ss es')

class Decoder(srd.Decoder):
    api_version = 3
    id = 'ps2'
    name = 'PS/2'
    longname = 'PS/2'
    desc = 'PS/2 keyboard/mouse interface.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = []
    tags = ['PC']
    channels = (
        {'id': 'clk', 'name': 'Clock', 'desc': 'Clock line'},
        {'id': 'data', 'name': 'Data', 'desc': 'Data line'},
    )
    annotations = (
        ('bit', 'Bit'),
        ('start-bit', 'Start bit'),
        ('stop-bit', 'Stop bit'),
        ('parity-ok', 'Parity OK bit'),
        ('error', 'Error'),
        ('data-bit', 'Data bit'),
        ('word', 'Word'),
        ('inhibit', 'Inhibit'),
        ('ack', 'Acknowledge'),
    )
    annotation_rows = (
        ('bits', 'Bits', (0,)),
        ('fields', 'Fields', (1, 2, 3, 4, 5, 6, 7, 8)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.bits = []
        self.bitcount = 0
        self.samplerate = None
        self.inhibited = False

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def putb(self, bit, ann_idx):
        self.put(bit.ss, bit.es, self.out_ann, [ann_idx, [str(bit.val)]])

    def putx(self, bit, ann):
        self.put(bit.ss, bit.es, self.out_ann, ann)

    def handle_bits(self, datapin, clockpin, previous_sample, microseconds, transition):
        bit = Bit(datapin, clockpin, microseconds, previous_sample, self.samplenum)
        if (microseconds > 50) and (clockpin == 1):
            self.inhibited = False
            self.bitcount = 0
            return

        if (microseconds > 55) and (clockpin == 0):
            self.inhibited = True
            self.bitcount = 0
            self.put(bit.ss, bit.es, self.out_ann, [Ann.INHIBIT, ['Inhibit']])
            return

        if (self.inhibited ^ clockpin):
            return

        if transition:
            self.putx(bit, [Ann.ERROR, ['Unexpected Transition', 'UT']])

        self.putb(bit, Ann.BIT)

        if self.bitcount == 0:
            if datapin:
                self.putx(bit, [Ann.ERROR, ['Start bit error', 'SB-E']])
            else:
                self.putx(bit, [Ann.START, ['Start bit', 'Start', 'S']])
            self.word = 0
            self.parity = 0
            self.wordstart = self.samplenum
        elif self.bitcount < 9:
            self.word |= datapin << (self.bitcount-1)
            self.parity ^= datapin
        elif self.bitcount == 9:
            self.put(self.wordstart, bit.ss, self.out_ann, [Ann.WORD,
                    ['%s Data: %02x' % ("Host" if self.inhibited else "Device", self.word), '%sD: %02x' % ("H" if self.inhibited else "D", self.word), '%02x' % (self.word)]])
            if self.parity != datapin:
                self.putx(bit, [Ann.PARITY_OK, ['Parity OK', 'Par OK', 'P']])
            else:
                self.putx(bit, [Ann.ERROR, ['Parity error', 'Par ERR', 'PE']])
        elif self.bitcount == 10:
            if datapin:
                self.putx(bit, [Ann.STOP, ['Stop bit', 'Stop', 'SB', 'S']])
            else:
                self.putx(bit, [Ann.ERROR, ['Stop bit error', 'SB-ERR']])
            self.inhibited = False
        elif self.bitcount == 11:
            if datapin:
                self.putx(bit, [Ann.ERROR, ['Device Acknowledge Error', 'Ack Err', 'A-E']])
            else:
                self.putx(bit, [Ann.ACK, ['Device Acknowledge', 'D ACK', 'A']])

        self.bitcount += 1

        return

    def decode(self):
        previous_data, previous_clock = self.wait([])
        previous_sample = self.samplenum
        while True:
            clock_pin, data_pin = self.wait([{0: 'e'}, {1:'e'}])
            transition = not(self.matched[0])
            if transition:
                clock_pin, data_pin = self.wait([{0: 'e'}])
            self.handle_bits(previous_data, previous_clock, previous_sample, (self.samplenum - previous_sample) / self.samplerate * 1000000, transition)
            previous_data = data_pin
            previous_clock = clock_pin
            previous_sample = self.samplenum
