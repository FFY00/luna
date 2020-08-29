#!/usr/bin/env python3
#
# This file is part of LUNA.
#
# Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause
""" Incomplete example for working the SerDes-based a PIPE PHY. """

from nmigen import *

from nmigen_boards.ecpix5 import ECPIX585Platform

from luna                         import top_level_cli
from luna.gateware.platform       import NullPin

from luna.gateware.interface.serdes_phy.backends.ecp5 import LunaECP5SerDes
from luna.gateware.interface.serdes_phy.core          import USB3PIPE



class PIPEPhyExample(Elaboratable):
    """ Hardware module that demonstrates grabbing a PHY resource with gearing. """

    def elaborate(self, platform):
        m = Module()

        sync_frequency = 125e6
        fast_frequency = 200e6

        serdes_channel = 1
        serdes_io      = platform.request("serdes", serdes_channel, dir={'tx':"-", 'rx':"-"})

        # Generate our domain clocks/resets.
        m.submodules.car = platform.clock_domain_generator()

        # Create our SerDes interface...
        m.submodules.serdes = serdes = LunaECP5SerDes(platform,
            sys_clk      = ClockSignal("sync"),
            sys_clk_freq = sync_frequency,
            refclk_pads  = ClockSignal("fast"),
            refclk_freq  = fast_frequency,
            tx_pads      = serdes_io.tx,
            rx_pads      = serdes_io.rx,
            channel      = serdes_channel
        )

        # ... and our PIPE PHY.
        m.submodules.phy = phy = USB3PIPE(serdes=serdes, sys_clk_freq=sync_frequency)

        # Return our elaborated module.
        return m


if __name__ == "__main__":
    top_level_cli(PIPEPhyExample)
