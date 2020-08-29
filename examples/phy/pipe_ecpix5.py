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

from luna.gateware.interface.serdes_phy.serdes import ECP5USB3SerDes
from luna.gateware.interface.serdes_phy.core   import USB3PIPE



class PIPEPhyExample(Elaboratable):
    """ Hardware module that demonstrates grabbing a PHY resource with gearing. """

    def elaborate(self, platform):
        m = Module()

        # Generate our domain clocks/resets.
        m.submodules.car = platform.clock_domain_generator()

        # XXX
        m.domains.sync   = ClockDomain()
        m.domains.clk200 = ClockDomain()
        m.d.comb += ClockSignal("clk200").eq(ClockSignal())


        # FIXME: actually set up the frequency this way
        sync_frequency   = 125e6

        # USB3 SerDes ------------------------------------------------------------------------------
        usb3_serdes = DomainRenamer({'sys': 'sync'})(ECP5USB3SerDes(platform,
            sys_clk      = ClockSignal("sync"),
            sys_clk_freq = sync_frequency,
            refclk_pads  = ClockSignal("clk200"),
            refclk_freq  = 200e6,
            tx_pads      = platform.request("usbc_tx", dir="-"),
            rx_pads      = platform.request("usbc_rx", dir="-"),
            channel      = 1))
        m.submodules += usb3_serdes

        # USB3 PIPE --------------------------------------------------------------------------------
        usb3_pipe = DomainRenamer({'sys': 'sync'})(USB3PIPE(serdes=usb3_serdes, sys_clk_freq=sync_frequency))
        m.submodules.usb3_pipe = usb3_pipe


        #m.d.comb += [
        #    # Enable our PHY outputs.
        #    pipe.reset           .eq(0),
        #    pipe.phy_reset       .eq(0),
        #    pipe.out_enable      .eq(1),
        #    pipe.power_down      .eq(0),
        #    pipe.rate            .eq(1),

        #    # Transmit a simple pattern...
        #    pipe.rx_termination  .eq(1),
        #    pipe.tx_bytes[0]     .eq(0xBC),
        #    pipe.tx_bytes[1]     .eq(0xBB),
        #    pipe.tx_bytes[2]     .eq(0xCC),
        #    pipe.tx_bytes[3]     .eq(0xDD),
        #    pipe.tx_data_k[0]    .eq(1),

        #    #platform.request("led", 0).o.eq(pipe.phy_status.i),
        #    #platform.request("led", 1).o.eq(1),
        #    #platform.request("led", 2).o.eq(pipe.pwrpresent),

        #]


        # Return our elaborated module.
        return m


if __name__ == "__main__":
    top_level_cli(PIPEPhyExample)
