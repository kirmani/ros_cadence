#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2016 kirmani <sean@kirmani.io>
#
# Distributed under terms of the MIT license.

"""
Action.
"""

from multiprocessing import Process
import os
import signal

class Action:
    def __init__(self, name, entities, preconditions, postconditions):
        self.name = name
        self.entities = entities
        self.preconditions = preconditions
        self.postconditions = postconditions
        self.process_ = Process(target = self.TaskWrapper_)
        self.on_interrupt_ = None

    def Task(self):
        pass

    def Start(self):
        self.process_.start()

    def Interrupt(self):
        os.killpg(os.getpgid(self.process_.pid), signal.SIGTERM)
        if (self.on_interrupt_):
            self.on_interrupt_(self)

    def OnInterrupt(self, func):
        self.on_interrupt_ = func

    def IsFinished(self):
        return not self.process_.is_alive()

    def TaskWrapper_(self):
        os.setpgid(os.getpid(), os.getpid())
        self.Task()
