#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2016 kirmani <sean@kirmani.io>
#
# Distributed under terms of the MIT license.

"""
Wait for resource to be interrupted.
"""

from action import Action

class WaitForResourceInterrupted(Action):
    def __init__(self, resource):
        Action.__init__(self, 'wait_for_resource_interrupted', [resource],
                {resource: True},
                {resource: False})
        self.interrupted_ = False

    def Start(self):
        pass

    def Interrupt(self):
        self.interrupted_ = True

    def IsFinished(self):
        return self.interrupted_

