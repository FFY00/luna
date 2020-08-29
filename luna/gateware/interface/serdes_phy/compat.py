#
# This file is part of LUNA.
#
# Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
#
# Code based on ``litex`` and ``usb3_pipe``.
# SPDX-License-Identifier: BSD-3-Clause
    

from nmigen.compat import *
    
class WaitTimer(Module):
    def __init__(self, t):
        self.wait = Signal()
        self.done = Signal()

        # # #

        count = Signal(bits_for(t), reset=t)
        self.comb += self.done.eq(count == 0)
        self.sync += \
            If(self.wait,
                If(~self.done, count.eq(count - 1))
            ).Else(count.eq(count.reset))


def xdir(obj, return_values=False):
    for attr in dir(obj):
        if attr[:2] != "__" and attr[-2:] != "__":
            if return_values:
                yield attr, getattr(obj, attr)
            else:
                yield attr
