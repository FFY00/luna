#
# This file is part of LUNA.
#
# Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
#
# Code based on ``litex`` and ``usb3_pipe``.
# SPDX-License-Identifier: BSD-3-Clause
""" Abstract SerDes interfacing code.

See also the FPGA family-specific SerDes backends located in the `backends` subfolder.
"""

from nmigen.compat import *
from nmigen.lib.cdc import FFSynchronizer, PulseSynchronizer, ResetSynchronizer

from . import stream
from .common import K, COM, SKP
from .encoding import Encoder, Decoder
from .compat import WaitTimer


# RX SKP Remover (6.4.3) ---------------------------------------------------------------------------

class RXSKPRemover(Module):
    """RX SKP Remover

    SKP Ordered Sets are inserted in the stream for clock compensation between partners with an
    average of 1 SKP Ordered Set every 354 symbols. This module removes SKP Ordered Sets from
    the RX stream.
    """
    def __init__(self):
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.skip   = Signal()

        # # #

        # Find SKP symbols -------------------------------------------------------------------------
        skp = Signal(4)
        for i in range(4):
            self.comb += skp[i].eq(sink.ctrl[i] & (sink.data[8*i:8*(i+1)] == SKP.value))
        self.comb += self.skip.eq(self.sink.valid & self.sink.ready & (skp != 0))

        # Select valid Data/Ctrl fragments ---------------------------------------------------------
        frag_data  = Signal(32)
        frag_ctrl  = Signal(4)
        frag_bytes = Signal(3)
        cases = {}
        for i in range(2**4):
            datas = []
            ctrls = []
            for j in range(4):
                if (i & 2**j) == 0:
                    datas.append(sink.data[8*j:8*(j+1)])
                    ctrls.append(sink.ctrl[1*j:1*(j+1)])
            cases[i] = [
                frag_data.eq(Cat(*datas) if len(datas) else 0),
                frag_ctrl.eq(Cat(*ctrls) if len(ctrls) else 0),
                frag_bytes.eq(len(ctrls)),
            ]
        self.comb += Case(skp, cases)

        # Store Data/Ctrl in a 64/8-bit Shift Register ---------------------------------------------
        sr_data  = Signal(64)
        sr_ctrl  = Signal(8)
        sr_bytes = Signal(4)
        cases = {}
        cases[0] = [
            sr_data.eq(sr_data),
            sr_ctrl.eq(sr_ctrl),
        ]
        for i in range(1, 5):
            cases[i] = [
                sr_data.eq(Cat(sr_data[8*i:], frag_data[0:8*i])),
                sr_ctrl.eq(Cat(sr_ctrl[1*i:], frag_ctrl[0:1*i])),
            ]
        self.comb += sink.ready.eq(sr_bytes <= 7)
        self.sync += [
            If(sink.valid & sink.ready,
                If(source.valid & source.ready,
                    sr_bytes.eq(sr_bytes + frag_bytes - 4)
                ).Else(
                    sr_bytes.eq(sr_bytes + frag_bytes)
                ),
                Case(frag_bytes, cases)
            ).Elif(source.valid & source.ready,
                sr_bytes.eq(sr_bytes - 4)
            )
        ]

        # Output Data/Ctrl when there is a full 32/4-bit word --------------------------------------
        self.comb += source.valid.eq(sr_bytes >= 4)
        cases = {}
        for i in range(4, 8):
            cases[i] = [
                source.data.eq(sr_data[8*(8-i):8*(8-i+4)]),
                source.ctrl.eq(sr_ctrl[1*(8-i):1*(8-i+4)]),
            ]
        self.comb += Case(sr_bytes, cases)


# RX Aligner ---------------------------------------------------------------------------------------

class RXWordAligner(Module):
    """RX Word Aligner

    Align RX Words by analyzing the location of the COM/K-codes (configurable) in the RX stream.
    """
    def __init__(self, check_ctrl_only=False):
        self.enable = Signal(reset=1)
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        alignment   = Signal(2)
        alignment_d = Signal(2)

        buf = stream.Buffer([("data", 32), ("ctrl", 4)])
        self.submodules += buf
        self.comb += [
            sink.connect(buf.sink),
            source.valid.eq(sink.valid & buf.source.valid),
            buf.source.ready.eq(sink.valid & source.ready),
        ]

        # Alignment detection
        for i in reversed(range(4)):
            self.comb += [
                If(sink.valid & sink.ready,
                    If(sink.ctrl[i] & (check_ctrl_only | (sink.data[8*i:8*(i+1)] == COM.value)),
                        alignment.eq(i)
                    )
                )
            ]
        self.sync += [
            If(sink.valid & sink.ready,
                If(self.enable,
                    If((sink.ctrl != 0) & (buf.source.ctrl == 0),
                        alignment_d.eq(alignment),
                    )
                )
            ),
        ]

        # Data selection
        data = Cat(buf.source.data, sink.data)
        ctrl = Cat(buf.source.ctrl, sink.ctrl)
        cases = {}
        for i in range(4):
            cases[i] = [
                source.data.eq(data[8*i:]),
                source.ctrl.eq(ctrl[i:]),
            ]
        self.comb += Case(alignment_d, cases)

# RXErrorSubstitution (6.3.5) ----------------------------------------------------------------------

class RXErrorSubstitution(Module):
    """RX Error Substitution

    Substitutes 8b/10b decoder errors with K28.4 symbols.
    """
    def __init__(self, serdes, clock_domain):
        self.sink   = stream.Endpoint([("data", 16), ("ctrl", 2)])
        self.source = stream.Endpoint([("data", 16), ("ctrl", 2)])

        # # #

        self.comb += self.sink.connect(self.source)
        for i in range(2):
            self.comb += [
                If(serdes.decoders[i].invalid,
                    self.source.ctrl[i].eq(1),
                    self.source.data[8*i:8*(i+1)].eq(K(28, 4)),
                )
            ]

# TX SKP Inserter (6.4.3) --------------------------------------------------------------------------

class TXSKPInserter(Module):
    """TX SKP Inserter

    SKP Ordered Sets are inserted in the stream for clock compensation between partners with an
    average of 1 SKP Ordered Set every 354 symbols. This module inserts SKP Ordered Sets to the
    TX stream. SKP Ordered Sets shall not be inserted inside a packet, so this packet delimiters
    (first/last) should be used to ensure SKP are inserted only between packets and not inside.

    Note: To simplify implementation and keep alignment, 2 SKP Ordered Sets are inserted every 708
    symbols, which is a small deviation from the specification (good average, but 2x interval between
    SKPs). More tests need to be done to ensure this deviation is acceptable with all hardwares.
    """
    def __init__(self):
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        data_count   = Signal(8)
        skip_grant   = Signal(reset=1)
        skip_queue   = Signal()
        skip_dequeue = Signal()
        skip_count   = Signal(16)

        # Queue one 2 SKP Ordered Set every 176 Data/Ctrl words
        self.sync += [
            skip_queue.eq(0),
            If(sink.valid & sink.ready,
                data_count.eq(data_count + 1),
                If(data_count == 175,
                    data_count.eq(0),
                    skip_queue.eq(1)
                )
            )
        ]

        # SKP grant: SKP should not be inserted inside packets
        self.sync += [
            If(sink.valid & sink.ready,
                If(sink.last,
                    skip_grant.eq(1)
                ).Elif(sink.first,
                    skip_grant.eq(0)
                )
            )
        ]

        # SKP counter
        self.sync += [
            If(skip_queue & ~skip_dequeue,
                skip_count.eq(skip_count + 1)
            ),
            If(~skip_queue &  skip_dequeue,
                skip_count.eq(skip_count - 1)
            )
        ]

        # SKP insertion
        self.comb += [
            If(skip_grant & (skip_count != 0),
                source.valid.eq(1),
                source.data.eq(Replicate(Signal(8, reset=SKP.value), 4)),
                source.ctrl.eq(Replicate(Signal(1, reset=1)        , 4)),
                skip_dequeue.eq(source.ready)
            ).Else(
                sink.connect(source)
            )
        ]

# Datapath (Clock Domain Crossing & Converter) -----------------------------------------------------

class TXDatapath(Module):
    """TX Datapath

    This module realizes the:
    - Clock compensation (SKP insertion).
    - Clock domain crossing (from system clock to transceiver's TX clock).
    - Data-width adaptation (from 32-bit to transceiver's data-width).
    """
    def __init__(self, clock_domain="sync", phy_dw=16):
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", phy_dw), ("ctrl", phy_dw//8)])

        # # #

        # Clock compensation
        skip_inserter = TXSKPInserter()
        self.submodules += skip_inserter

        # Clock domain crossing
        cdc = stream.AsyncFIFO([("data", 32), ("ctrl", 4)], 8, buffered=True)
        cdc = ClockDomainsRenamer({"write": "sync", "read": clock_domain})(cdc)
        self.submodules.cdc = cdc

        # Data-width adaptation
        converter = stream.StrideConverter(
            [("data", 32), ("ctrl", 4)],
            [("data", phy_dw), ("ctrl", phy_dw//8)],
            reverse=False)
        #converter = stream.BufferizeEndpoints({"source": stream.DIR_SOURCE})(converter)
        converter = ClockDomainsRenamer(clock_domain)(converter)
        self.submodules.converter = converter

        # Flow
        self.comb += [
            self.sink.connect(skip_inserter.sink),
            skip_inserter.source.connect(cdc.sink),
            cdc.source.connect(converter.sink),
            converter.source.connect(self.source)
        ]

class RXDatapath(Module):
    """RX Datapath

    This module realizes the:
    - Data-width adaptation (from transceiver's data-width to 32-bit).
    - Clock domain crossing (from transceiver's RX clock to system clock).
    - Clock compensation (SKP removing).
    - Words alignment.
    """
    def __init__(self, clock_domain="sync", phy_dw=16):
        self.sink   = stream.Endpoint([("data", phy_dw), ("ctrl", phy_dw//8)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        # Data-width adaptation
        converter = stream.StrideConverter(
            [("data", phy_dw), ("ctrl", phy_dw//8)],
            [("data", 32), ("ctrl", 4)],
            reverse=False)
        #converter = stream.BufferizeEndpoints({"sink":   stream.DIR_SINK})(converter)
        converter = ClockDomainsRenamer(clock_domain)(converter)
        self.submodules.converter = converter

        # Clock domain crossing
        cdc = stream.AsyncFIFO([("data", 32), ("ctrl", 4)], 8, buffered=True)
        cdc = ClockDomainsRenamer({"write": clock_domain, "read": "sync"})(cdc)
        self.submodules.cdc = cdc

        # Clock compensation
        skip_remover = RXSKPRemover()
        self.submodules.skip_remover = skip_remover

        # Words alignment
        word_aligner = RXWordAligner()
        #word_aligner = stream.BufferizeEndpoints({"source": stream.DIR_SOURCE})(word_aligner)
        self.submodules.word_aligner = word_aligner

        # Flow
        self.comb += [
            self.sink.connect(converter.sink),
            converter.source.connect(cdc.sink),
            cdc.source.connect(skip_remover.sink),
            skip_remover.source.connect(word_aligner.sink),
            word_aligner.source.connect(self.source),
        ]


class SerdesRXInit(Module):
    def __init__(self, tx_lol, rx_lol, rx_los, rx_lsm):
        self.rrst        = Signal()
        self.lane_rx_rst = Signal()

        # # #

        _tx_lol      = Signal()
        _rx_lol      = Signal()
        _rx_los      = Signal()
        _rx_lsm      = Signal()
        _rx_lsm_seen = Signal()
        self.specials += [
            FFSynchronizer(tx_lol, _tx_lol),
            FFSynchronizer(rx_lol, _rx_lol),
            FFSynchronizer(rx_los, _rx_los),
            FFSynchronizer(rx_lsm, _rx_lsm),
        ]

        timer = WaitTimer(int(4e5))
        self.submodules += timer

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self.lane_rx_rst.eq(1),
            If(~_tx_lol,
                NextState("RESET-ALL")
            )
        )
        fsm.act("RESET-ALL",
            self.rrst.eq(1),
            self.lane_rx_rst.eq(1),
            NextState("RESET-PCS")
        )
        fsm.act("RESET-PCS",
            self.lane_rx_rst.eq(1),
            timer.wait.eq(~_rx_lol & ~_rx_los),
            If(timer.done,
                timer.wait.eq(0),
                NextValue(_rx_lsm_seen, 0),
                NextState("CHECK-LSM")
            )
        )
        fsm.act("CHECK-LSM",
            NextValue(_rx_lsm_seen, _rx_lsm_seen | _rx_lsm),
            timer.wait.eq(1),
            If(_rx_lsm_seen & ~_rx_lsm,
                NextState("IDLE")
            ),
            If(timer.done,
                If(_rx_lsm,
                    NextState("READY")
                ).Else(
                    NextState("IDLE")
                )
            )
        )
        fsm.act("READY",
            If(_tx_lol | _rx_lol | _rx_los,
                NextState("IDLE")
            )
        )
