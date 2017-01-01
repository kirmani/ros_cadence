#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2016 kirmani <sean@kirmani.io>
#
# Distributed under terms of the MIT license.

"""
Resource listener.
"""
class ResourceListener:
    def __init__(self, name):
        self.name = name

    def StartListening(self, actions):
        pass

    def Poll(self):
        """
        Return True when object is 'free', False otherwise
        """
        return False

    def OnShutdown(self):
        pass
