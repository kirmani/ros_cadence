#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2016 kirmani <sean@kirmani.io>
#
# Distributed under terms of the MIT license.

"""
Wait for resource to be free.
"""

from action import Action

class WaitForResourceFree(Action):
    def __init__(self, resource):
        Action.__init__(self, 'wait_for_resource_free', [resource],
                {resource: True},
                {resource: True})

    def Start(self):
        pass

    def Interrupt(self):
        pass

    def IsFinished(self):
        return True

