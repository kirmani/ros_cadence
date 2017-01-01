#!/usr/bin/env python

from petri_net import PetriNet
from petri_net import PetriNetPlace
from petri_net import PetriNetTransition
from resource_controller import ResourceControllerApi
import rospy

class StartTransition(PetriNetTransition):
  def __init__(self, name, action, queue, started):
    PetriNetTransition.__init__(self, name)
    self.action_ = action
    self.queue_ = queue
    self.started_ = started

  def fire(self):
    print("Starting action: %s" % self.action_.name)
    self.action_.Start()
    ResourceControllerApi.AddActiveAction(self.action_.name)
    self.queue_.RemoveToken(self.action_.name)
    self.started_.AddToken(self.action_.name)

  def activated(self):
    if not self.queue_.HasToken(self.action_.name):
      return False
    for resource in self.action_.preconditions:
      if not ResourceControllerApi.CheckGuard('owned_robot', resource):
        return False
    return True

class InterruptTransition(PetriNetTransition):
  def __init__(self, name, action, started, interrupted):
    PetriNetTransition.__init__(self, name)
    self.action_ = action
    self.started_ = started
    self.interrupted_ = interrupted

  def fire(self):
    print("Interrupting action: %s" % self.action_.name)
    ResourceControllerApi.RemoveActiveAction(self.action_.name)
    self.action_.Interrupt()
    self.started_.RemoveToken(self.action_.name)
    self.interrupted_.AddToken(self.action_.name)

  def activated(self):
    if not self.started_.HasToken(self.action_.name):
      return False
    for resource in self.action_.preconditions:
      if not ResourceControllerApi.CheckGuard('owned_robot', resource):
        return True
    return False

class FinishTransition(PetriNetTransition):
  def __init__(self, name, action, started, interrupted, finished):
    PetriNetTransition.__init__(self, name)
    self.action_ = action
    self.started_ = started
    self.interrupted_ = interrupted
    self.finished_ = finished

  def fire(self):
    print("Finishing action: %s" % self.action_.name)
    if self.started_.HasToken(self.action_.name):
      self.started_.RemoveToken(self.action_.name)
    if self.interrupted_.HasToken(self.action_.name):
      self.interrupted_.RemoveToken(self.action_.name)
    for resource in self.action_.preconditions:
      ResourceControllerApi.RemoveResourceFromPlace('requested_robot', resource)
    self.finished_.AddToken(self.action_.name)
    ResourceControllerApi.RemoveActiveAction(self.action_.name)

  def activated(self):
    return self.action_.IsFinished() and \
        (self.started_.HasToken(self.action_.name) \
        or self.interrupted_.HasToken(self.action_.name))

class SeizeRobotTransition(PetriNetTransition):
  def __init__(self, name, action):
    PetriNetTransition.__init__(self, name)
    self.action_ = action

  def fire(self):
    # Remove resources from requested, and put resource tokens in requested
    # place.
    for resource in self.action_.preconditions:
      if (ResourceControllerApi.CheckGuard('requested_robot', resource)
              and ResourceControllerApi.CheckGuard('free', resource)):
        ResourceControllerApi.RemoveResourceFromPlace('free', resource)
        ResourceControllerApi.AddResourceToPlace('owned_robot', resource)

  def activated(self):
    for resource in self.action_.preconditions:
      if (ResourceControllerApi.CheckGuard('requested_robot', resource)
              and ResourceControllerApi.CheckGuard('free', resource)):
        return True
    return False

class RequestRobotTransition(PetriNetTransition):
  def __init__(self, action):
    PetriNetTransition.__init__(self, 'request_robot')
    self.action_ = action
    self.already_requested_ = False

  def fire(self):
    # Place resource tokens in requested place.
    print("Requesting resources for action: %s" % self.action_.name)
    for resource in self.action_.preconditions:
      ResourceControllerApi.AddResourceToPlace('requested_robot', resource)

  def activated(self):
    if not self.already_requested_:
      self.already_requested_ = True
      return True
    return False

class ActionProcess(PetriNet):
  def __init__(self, name, action):
    PetriNet.__init__(self, name)
    self.action_ = action

    # Places.
    queue = PetriNetPlace('queue')
    started = PetriNetPlace('started')
    interrupted = PetriNetPlace('interrupted')
    self.finished_ = PetriNetPlace('finished')

    # Transitions.
    self.transitions_.append(RequestRobotTransition(action))
    self.transitions_.append(StartTransition('start', action, queue, started))
    self.transitions_.append(SeizeRobotTransition('seize_robot', action))
    self.transitions_.append(
        InterruptTransition('interrupt', action, started, interrupted))
    self.transitions_.append(
        FinishTransition('finish', action, started,interrupted, self.finished_))

    # Put action token in queue.
    queue.AddToken(self.action_.name)

  def EndCondition(self):
    return rospy.is_shutdown() or self.finished_.HasToken(self.action_.name)
