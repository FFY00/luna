#
# This file is part of LUNA.
#
# Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
#
# Code adapted from ``litex`` and ``usb3_pipe``.
# SPDX-License-Identifier: BSD-3-Clause


from .           import stream
from .common     import *
from .lfps       import LFPSTransceiver
from .training   import TSUnit
from .ltssm      import LTSSM
from .scrambling import Scrambler, Descrambler

from nmigen      import *


class USB3PIPE(Elaboratable):
    """USB3.0 PIPE Core

    Wrap an FPGA transceiver exposing 2 TX and RX data/ctrl streams into a USB3.0 PIPE by adding:
    - LFPS detection/generation.
    - Training Sequence Ordered Sets detection/generation.
    - Clock compensation Ordered Sets removing/insertion.
    - Convertion to/from a 32-bit/4-bit data/ctrl stream.
    - Clock domain crossing to/from sys_clk (>=125MHz).
    - RX word alignment.
    - TX scrambling/RX descrambling.
    - Link Training State Machine.
    """
    def __init__(self, serdes, sys_clk_freq, with_endianness_swap=True):
        assert sys_clk_freq >= 125e6

        self._serdes               = serdes
        self._clock_frequency      = sys_clk_freq
        self._with_endianness_swap = with_endianness_swap

        #
        # I/O port
        #
        self.ready  = Signal() # o

        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])


    def elaborate(self, platform):
        m = Module()

         # If necessary, handle stream endianness swapping.
        if self._with_endianness_swap:
            sink        = stream.Endpoint([("data", 32), ("ctrl", 4)])
            source      = stream.Endpoint([("data", 32), ("ctrl", 4)])
            sink_swap   = EndiannessSwap(self.sink, sink)
            source_swap = EndiannessSwap(source, self.source)
            m.submodules += sink_swap, source_swap
        else:
            sink   = self.sink
            source = self.source

        #
        # Subcomponents.
        #

        # - A handler for Low-frequency Periodic Signaling (LPFS)
        # - A producer/emitter for Training Sequences.
        # - A Link Training and Status State Machine handler.
        m.submodules.lfps  = lfps  = LFPSTransceiver(serdes=self._serdes, sys_clk_freq=self._clock_frequency)
        m.submodules.ts    = ts    = TSUnit(serdes=self._serdes)
        m.submodules.ltssm = ltssm = LTSSM(serdes=self._serdes, lfps_unit=lfps, ts_unit=ts, sys_clk_freq=self._clock_frequency)
        m.d.comb += self.ready.eq(ltssm.polling.idle)


        #
        # Transmit data scrambling.
        #
        m.submodules.scrambler = scrambler = ResetInserter(~ltssm.polling.tx_ready)(Scrambler())
        m.d.comb += sink.connect(scrambler.sink),

        with m.If(ltssm.polling.tx_ready):
            m.d.comb += scrambler.source.connect(self._serdes.sink)


        #
        # Receive data de-scrambling.
        #
        m.submodules.descrambler = descrambler = Descrambler()
        m.d.comb += descrambler.source.connect(source)

        with m.If(ltssm.polling.rx_ready):
            m.d.comb += self._serdes.source.connect(descrambler.sink, omit={"data", "ctrl"})
        with m.Else():
            m.d.comb += self._serdes.source.connect(descrambler.sink, keep={"data", "ctrl"}),

        return m
