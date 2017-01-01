#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2016 kirmani <sean@kirmani.io>
#
# Distributed under terms of the MIT license.

"""

"""

from sets import Set
from threading import Lock

class PetriNetNode(object):
  def __init__(self, name):
    self.name = name

class PetriNetTransition(PetriNetNode):
  def __init__(self, name):
    PetriNetNode.__init__(self, name)
    self.inputs_ = []
    self.outputs_ = []

  def fire(self):
    if self.activated():
      pass

  def activated(self):
    return True

class PetriNetPlace(PetriNetNode):
  def __init__(self, name):
    PetriNetNode.__init__(self, name)
    self.tokens_ = []

  def AddToken(self, token):
    self.tokens_.append(token)

  def HasToken(self, token):
    return token in self.tokens_

  def RemoveToken(self, token):
    if token not in self.tokens_:
      return False
    self.tokens_.remove(token)
    return True

  def GetTokens(self):
    return self.tokens_

class PetriNetToken(PetriNetNode):
  def __init__(self, name, location):
    PetriNetNode.__init__(self, name)
    self.location_ = location

  def GetLocation(self):
    return self.location_

  def SetLocation(self, location):
    self.location_ = location

class PetriNet(object):
  def __init__(self, name):
    self.name_ = name
    self.transitions_ = []
    self.static_ = False
    self.lock_ = Lock()

  def Run(self):
    # TODO(kirmani): Make all transition nodes fire concurrently.
    self.lock_.acquire()
    self.static_ = False
    while not self.EndCondition():
      self.static_ = True
      for transition in self.transitions_:
        if transition.activated():
          transition.fire()
          self.static_ = False
    self.lock_.release()

  def EndCondition(self):
    return self.static_
