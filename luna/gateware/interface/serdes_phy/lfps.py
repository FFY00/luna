#
# This file is part of LUNA.
#
# Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
#
# Code based on ``litex`` and ``usb3_pipe``.
# SPDX-License-Identifier: BSD-3-Clause
""" Low-frequency periodic signaling gateware.

LFPS is the first signaling to happen during the initialization of the USB3.0 link.

LFPS allows partners to exchange Out Of Band (OOB) controls/commands and consists of bursts where
a "slow" clock is generated (between 10M-50MHz) for a specific duration and with a specific repeat
period. After the burst, the transceiver is put in electrical idle mode (same electrical level on
P/N pairs while in nominal mode P/N pairs always have an opposite level):

Transceiver level/mode: _=0, -=1 x=electrical idle
|-_-_-_-xxxxxxxxxxxxxxxxxxxx|-_-_-_-xxxxxxxxxxxxxxxxxxxx|...
|<burst>                    |<burst>                    |...
|<-----repeat period------->|<-----repeat period------->|...

A LFPS pattern is identified by a burst duration and repeat period.

To be able generate and receive LFPS, a transceiver needs to be able put its TX in electrical idle
and to detect RX electrical idle.
"""

import unittest

from math import ceil
from nmigen.lib.cdc import FFSynchronizer

from .compat import WaitTimer
from nmigen import *

from ...test.utils import LunaSSGatewareTestCase, ss_domain_test_case
from ...utils.cdc import synchronize

__all__ = ['LFPSTransceiver']

#
# LPFS timing "constants", and collection classes that represent them.
#

class LFPSTiming:
    """LPFS timings with typical, minimum and maximum timing values."""

    def __init__(self, t_typ=None, t_min=None, t_max=None):
        self.t_typ = t_typ
        self.t_min = t_min
        self.t_max = t_max
        assert t_min is not None
        assert t_max is not None

        self.range = (t_min, t_max)


class LFPS:
    """LPFS patterns with burst and repeat timings."""

    def __init__(self, burst, repeat=None, cycles=None):
        self.burst  = burst
        self.repeat = repeat
        self.cycles = None


def _ns_to_cycles(clk_freq, t):
    return ceil(t*clk_freq)


_DEFAULT_LFPS_FREQ = 30e6

_LFPS_CLK_FREQ_MIN = 1/100e-9
_LFPS_CLK_FREQ_MAX = 1/20e-9


# Our actual pattern constants; as specified by the USB3 specification.
_PollingLFPSBurst  = LFPSTiming(t_typ=1.0e-6,  t_min=0.6e-6, t_max=1.4e-6)
_PollingLFPSRepeat = LFPSTiming(t_typ=10.0e-6, t_min=6.0e-6, t_max=14.0e-6)
_PollingLFPS       = LFPS(burst=_PollingLFPSBurst, repeat=_PollingLFPSRepeat)

_ResetLFPSBurst    = LFPSTiming(t_typ=100.0e-3, t_min=80.0e-3,  t_max=120.0e-3)
_ResetLFPS         = LFPS(burst=_ResetLFPSBurst)


#
# Core LFPS gateware.
#

class LFPSDetector(Elaboratable):
    """ Low-Frequecy Period Signaling (LFPS) detector.

    Generic LFPS checker.

    This module is able to detect a specific LFPS pattern by analyzing the RX electrical idle signal
    of the transceiver. It generates the pattern internally and tries to lock it to the RX electrical
    idle signal. When locked, a detection is reported.

    """
    def __init__(self, lfps_pattern, sys_clk_freq):
        self._pattern         = lfps_pattern
        self._clock_frequency = sys_clk_freq

        self._burst_cycles    = _ns_to_cycles(self._clock_frequency, self._pattern.burst.t_typ)
        self._repeat_cycles   = _ns_to_cycles(self._clock_frequency, self._pattern.repeat.t_typ)
        needed_counter_max    = max(self._burst_cycles, self._repeat_cycles)

        #
        # I/O port
        #
        self.idle   = Signal() # i
        self.detect = Signal() # o

        self.count = Signal(range(0, needed_counter_max + 1))
        self.found = Signal()


    def elaborate(self, platform):
        m = Module()

        count = self.count
        found = self.found

        # Get an in-domain copy of our idle signal.
        idle  = synchronize(m, self.idle, o_domain="ss")

        #
        # Detector state machine.
        #
        with m.FSM(domain="ss"):

            # TBURST -- check to see if we have an LPFS burst pattern
            with m.State("TBURST"):
                m.d.ss += count.eq(count - 1)

                # Check once per bit period for electrical idle; this helps us to "lock on"
                # to points of prolonged electrical idle.
                with m.If(count == 0):

                    # If we see electrical idle, we'll assume we've just entered the non-burst
                    # section of our repeating LPFS signal.
                    with m.If(idle):
                        m.d.ss += count.eq(self._repeat_cycles - 2*self._burst_cycles - 1)
                        m.next = "TREPEAT"

                    # Otherwise, we'll poll again in after a given amount of cycles.
                    with m.Else():
                        m.d.ss += count.eq(self._burst_cycles - 1)


                # If we're not idle, and we came here after experiecing the proper repeat interval,
                # we've detected what seems to be our LFPS burst.
                with m.If(found & (idle == 0)):
                    m.d.comb += self.detect.eq(1)
                    m.d.ss   += found.eq(0)


            # TREPEAT -- we're in electrical idle; time how long we remain there.
            with m.State("TREPEAT"):
                m.d.ss += count.eq(count - 1)

                # Once we finish a repeat interval, we'll move back to checking for
                # our active signal.
                with m.If((count == 0) | (idle == 0)):
                    m.d.ss += [
                        found  .eq(count == 0),
                        count  .eq(self._burst_cycles - 1)
                    ]
                    m.next = "TBURST"

        return m


class LFPSBurstGenerator(Elaboratable):
    """LFPS Burst Generator

    Generate a LFPS burst of configurable length on the TX lane. The LFPS clock is generated by
    sending an alternating ones/zeroes data pattern on the parallel interface of the transceiver.
    """
    def __init__(self, sys_clk_freq, lfps_clk_freq):
        self._clock_frequency = sys_clk_freq
        self._lfps_clk_freq   = lfps_clk_freq

        # Validate that our frequency is within the allowed bounds.
        assert lfps_clk_freq >= _LFPS_CLK_FREQ_MIN
        assert lfps_clk_freq <= _LFPS_CLK_FREQ_MAX

        #
        # I/O ports
        #
        self.start      = Signal()   # i
        self.done       = Signal()   # o
        self.length     = Signal(32) # i
        

        self.tx_idle    = Signal(reset=1) # o
        self.tx_pattern = Signal(20)      # o


    def elaborate(self, platform):
        m = Module()

        #
        # LFPS square-wave generator.
        #
        timer_max = ceil(self._clock_frequency/(2*self._lfps_clk_freq)) - 1

        square_wave  = Signal()
        period_timer = Signal(range(0, timer_max + 1))

        with m.If(period_timer == 0):
            m.d.ss += [
                square_wave    .eq(~square_wave),
                period_timer   .eq(timer_max - 1)
            ]
        with m.Else():
            m.d.ss += period_timer.eq(period_timer - 1)


        #
        # Burst generator.
        #
        cycles_left_in_burst = Signal.like(self.length)

        with m.FSM(domain="ss"):

            # IDLE -- we're currently waiting for an LFPS burst to be requested.
            with m.State("IDLE"):
                m.d.comb += self.done.eq(1)
                m.d.ss   += [
                    period_timer          .eq(0),
                    cycles_left_in_burst  .eq(self.length)
                ]

                with m.If(self.start):
                    m.next = "BURST"

            # BURST -- we've started a burst; and now we'll wait here until the burst is complete.
            with m.State("BURST"):
                m.d.comb += [
                    self.tx_idle     .eq(0),
                    self.tx_pattern  .eq(Repl(square_wave, 20)),
                ]
                m.d.ss += cycles_left_in_burst.eq(cycles_left_in_burst - 1)

                with m.If(cycles_left_in_burst == 0):
                    m.next = "IDLE"

        return m

class LFPSBurstGeneratorTest(LunaSSGatewareTestCase):
    FRAGMENT_UNDER_TEST = LFPSBurstGenerator
    FRAGMENT_ARGUMENTS = dict(
        sys_clk_freq  = 125e6,
        lfps_clk_freq = _DEFAULT_LFPS_FREQ
    )

    def stimulus(self):
        """ Test stimulus for our burst clock generator. """
        dut = self.dut

        # Create eight bursts of 256 pulses.
        for _ in range(8):
            yield dut.length.eq(256)
            yield dut.start.eq(1)
            yield
            yield dut.start.eq(0)
            while not (yield dut.done):
                yield
            for __ in range(256):
                yield


    def setUp(self):
        # Hook our setup function, and add in our stimulus.
        super().setUp()
        self.sim.add_sync_process(self.stimulus, domain="ss")


    @ss_domain_test_case
    def test_lpfs_burst_duty_cycle(self):
        dut = self.dut

        transitions  = 0
        ones_ticks   = 0
        zeroes_ticks = 0
        tx_pattern   = 0

        # Wait a couple of cycles for our stimulus to set up our burst.
        yield from self.advance_cycles(2)

        # Wait for a burst cycle...
        while not (yield dut.done):

            # While we're bursting...
            if not (yield dut.tx_idle):

                # ... measure how many clock transitions we see...
                if (yield dut.tx_pattern != tx_pattern):
                    transitions += 1

                # ... as well as what percentage of the time the clock is 1 vs 0.
                if (yield dut.tx_pattern != 0):
                    ones_ticks   += 1
                else:
                    zeroes_ticks += 1

                tx_pattern = (yield dut.tx_pattern)
            yield

        # Figure out the total length that we've run for.
        total_ticks = ones_ticks + zeroes_ticks

        # We should measure a duty cycle that's within 10% of 50%...
        self.assertLess(abs(ones_ticks   / (total_ticks) - 50e-2), 10e-2)
        self.assertLess(abs(zeroes_ticks / (total_ticks) - 50e-2), 10e-2)

        # ... and our total length should be within 10% of nominal.
        expected_cycles = self.SS_CLOCK_FREQUENCY/_DEFAULT_LFPS_FREQ
        computed_cycles = 2*total_ticks/transitions
        self.assertLess(abs(expected_cycles/computed_cycles - 1.0), 10e-2)


class LFPSGenerator(Elaboratable):
    """LFPS Generator

    Generate a specific LFPS pattern on the TX lane. This module handles LFPS clock generation, LFPS
    burst generation and repetition.
    """
    def __init__(self, lfps_pattern, sys_clk_freq, lfps_clk_freq):
        self._pattern         = lfps_pattern
        self._clock_frequency = sys_clk_freq
        self._lpfs_frequency  = lfps_clk_freq

        #
        # I/O port
        #

        # Control
        self.generate = Signal()      # i
        self.count    = Signal(16)    # o

        # Transceiver
        self.tx_idle    = Signal()   # o
        self.tx_pattern = Signal(20) # o

        # Diagnostic
        self.busy       = Signal()


    def elaborate(self, platform):
        m = Module()

        #
        # LFPS burst generator.
        #
        m.submodules.burst_gen = burst_generator = LFPSBurstGenerator(
            sys_clk_freq=self._clock_frequency,
            lfps_clk_freq=self._lpfs_frequency
        )

        #
        # Controller
        #
        burst_repeat_count = Signal(32)
        full_burst_length  = int(self._clock_frequency*self._pattern.burst.t_typ)
        full_repeat_length = int(self._clock_frequency*self._pattern.repeat.t_typ)

        with m.FSM(domain="ss"):

            # IDLE -- wait for an LFPS burst request.
            with m.State("IDLE"):
                m.d.comb += self.tx_idle  .eq(0)
                m.d.ss   += self.count    .eq(0)

                # Once we get one, start a burst.
                with m.If(self.generate):
                    m.d.comb += self.tx_idle.eq(1)
                    m.d.ss   += [
                        burst_generator.start   .eq(1),
                        burst_generator.length  .eq(full_burst_length),
                        burst_repeat_count      .eq(full_repeat_length),
                    ]
                    m.next = "BURST_AND_WAIT"


            # BURST_AND_WAIT -- we've now triggered a burst; we'll wait until
            # the interval between bursts (the "repeat" interal) before allowing
            # another burst.
            with m.State("BURST_AND_WAIT"):
                m.d.comb += [
                    self.busy              .eq(1),
                    self.tx_idle           .eq(burst_generator.tx_idle),
                    self.tx_pattern        .eq(burst_generator.tx_pattern)
                ]
                m.d.ss   += [
                    burst_generator.start  .eq(0),
                    burst_repeat_count     .eq(burst_repeat_count - 1)
                ]

                # Once we've waited the repeat interval, return to ready.
                with m.If(burst_repeat_count == 0):
                    m.d.ss += self.count.eq(self.count + 1)
                    m.next = "IDLE"

        return m


class LFPSGeneratorTest(LunaSSGatewareTestCase):
    FRAGMENT_UNDER_TEST = LFPSGenerator
    FRAGMENT_ARGUMENTS  = dict(
        lfps_pattern = _PollingLFPS,
        sys_clk_freq = 125e6,
        lfps_clk_freq= _DEFAULT_LFPS_FREQ
    )

    @ss_domain_test_case
    def test_polling_lfps_sequence(self):
        dut = self.dut

        burst_length=int(self.SS_CLOCK_FREQUENCY * _PollingLFPSBurst.t_typ)
        burst_repeat=int(self.SS_CLOCK_FREQUENCY * _PollingLFPSRepeat.t_typ)

        # Trigger a burst...
        yield dut.generate.eq(1)
        yield
        yield

        # Wait for a whole burst-repeat cycle...
        burst_ticks = 0
        total_ticks = 0
        while (yield dut.busy):

            # ... and measure how long our burst lasts...
            if not (yield dut.tx_idle):
                burst_ticks += 1

            # ... as well as the length of our whole interval.
            total_ticks += 1
            yield

        # Our observed burst length should be within 10% of our specification...
        self.assertLess(abs(burst_ticks)/burst_length - 1.0, 10e-2)

        # ... as should our observed total length between bursts.
        self.assertLess(abs(total_ticks)/burst_repeat - 1.0, 10e-2)



class LFPSTransceiver(Elaboratable):
    """ Low-Frequency Periodic Signaling (LFPS) Transciever.

    Transmits and receives the LPFS required for a USB3.0 link.

    Attributes
    ----------

    rx_polling: Signal(), output
        Strobes high when Polling LFPS is detected.

    tx_idle: Signal(), input
        When asserted, indicates that the output lines should be held in electrical idle.
    tx_polling: Signal(), input
        Strobe. When asserted, begins an LFPS burst.

    tx_count: Signal(16), output
        [FIXME Document better] -> Indicates the current position in the LFPS transmission.
    """

    def __init__(self, serdes, sys_clk_freq, lfps_clk_freq=_DEFAULT_LFPS_FREQ):
        self._serdes          = serdes
        self._clock_frequency = sys_clk_freq
        self._lpfs_frequency  = lfps_clk_freq

        #
        # I/O port
        #
        self.rx_polling = Signal()
        self.tx_idle    = Signal()
        self.tx_polling = Signal()
        self.tx_count   = Signal(16)


    def elaborate(self, platform):
        m = Module()
        serdes = self._serdes

        #
        # LFPS Receiver.
        #
        polling_checker = LFPSDetector(_PollingLFPS, self._clock_frequency)
        m.submodules += polling_checker
        m.d.comb += [
            polling_checker.idle.eq(serdes.rx_idle),
            self.rx_polling.eq(polling_checker.detect)
        ]

        #
        # LFPS Transmitter(s).
        #
        polling_generator = LFPSGenerator(_PollingLFPS, self._clock_frequency, self._lpfs_frequency)
        m.submodules += polling_generator
        m.d.comb += self.tx_count.eq(polling_generator.count)

        # While we're issuing LFPS polling commands, give control of the SerDes's idling
        # to our LFPS burst generator; and provide it with pattern of 1's and 0's necessary
        # to generate the desired bursts.
        with m.If(self.tx_polling):
            m.d.comb += [
                polling_generator.generate.eq(1),
                serdes.tx_idle.eq(polling_generator.tx_idle),
                serdes.tx_pattern.eq(polling_generator.tx_pattern)
            ]
        with m.Else():
            m.d.comb += serdes.tx_idle.eq(self.tx_idle)

        return m


if __name__ == "__main__":
    unittest.main()
