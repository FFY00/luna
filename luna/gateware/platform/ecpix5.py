#
# This file is part of LUNA.
#
# Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

""" ecpix5 platform definitions.

This is a non-core platform. To use it, you'll need to set your LUNA_PLATFORM variable:

    > export LUNA_PLATFORM="luna.gateware.platform.ecpix5:ECPIX5_45F_Platform"
    > export LUNA_PLATFORM="luna.gateware.platform.ecpix5:ECPIX5_85F_Platform"
"""

from nmigen import *
from nmigen.build import *
from nmigen.vendor.lattice_ecp5 import LatticeECP5Platform

from nmigen_boards.ecpix5 import ECPIX545Platform as _ECPIX545Platform
from nmigen_boards.ecpix5 import ECPIX585Platform as _ECPIX585Platform

from .core import LUNAPlatform

__all__ = ["ECPIX5_45F_Platform", "ECPIX5_85F_Platform"]


class ECPIX5DomainGenerator(Elaboratable):
    """ Stub clock domain generator; stands in for the typical LUNA one.

    This generator creates domains; but currently does not configure them.
    """

    def __init__(self, *, clock_frequencies=None, clock_signal_name=None):
        pass

    def elaborate(self, platform):
        m = Module()

        # Create our domains; but don't do anything else for them, for now.
        m.domains.usb    = ClockDomain()
        m.domains.fast   = ClockDomain()

        m.d.comb += [
            ClockSignal("fast")    .eq(ClockSignal("sync")),
        ]

        return m



class _ECPIXExtensions:

    additional_resources = [
        # SerDes signals.
        Resource("usbc_rx", 0, DiffPairs("Y5", "Y6")),
        Resource("usbc_tx", 0, DiffPairs("W4", "W5"))


        ## SerDes pads.
        ## Note that while these are differnetial signals, Diamond requires these to be 
        #Resource("usbc_rx", 0,
        #    Subsignal("p", Pins("Y5", dir="i")),
        #    Subsignal("n", Pins("Y6", dir="i")),
        #),
        #Resource("usbc_tx", 0,
        #    Subsignal("p", Pins("W4", dir="o")),
        #    Subsignal("n", Pins("W5", dir="o")),
        #)
    ]



class ECPIX5_45F_Platform(_ECPIX545Platform, _ECPIXExtensions, LUNAPlatform):
    name                   = "ECPIX-5 (45F)"

    clock_domain_generator = ECPIX5DomainGenerator
    default_usb_connection = "ulpi"

    # Create our semantic aliases.
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.add_resources(self.additional_resources)



class ECPIX5_85F_Platform(_ECPIX585Platform, _ECPIXExtensions, LUNAPlatform):
    name                   = "ECPIX-5 (85F)"

    clock_domain_generator = ECPIX5DomainGenerator
    default_usb_connection = "ulpi"

    # Create our semantic aliases.
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.add_resources(self.additional_resources)
