#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2016 kirmani <sean@kirmani.io>
#
# Distributed under terms of the MIT license.

"""
The "hello world" example of interruptable actions.
"""

from action_process import ActionProcess
from speak import Speak

def on_interrupt(action):
    global g_interrupted
    g_interrupted = True

def main():
    global g_interrupted
    g_interrupted = True
    while g_interrupted:
        g_interrupted = False
        speak_action = Speak(150, 50, "please don't interrupt me i really want to finish" \
            "this sentence okay good it worked")
        speak_action.OnInterrupt(on_interrupt)
        ActionProcess('speak_action_process', speak_action).Run()

if __name__ == '__main__':
  main()

