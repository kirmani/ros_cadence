#!/usr/bin/env python

from petri_net import PetriNet
from petri_net import PetriNetPlace
from petri_net import PetriNetTransition
from resource_controller import ActionProcessApi
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
    self.queue_.RemoveToken(self.action_.name)
    self.started_.AddToken(self.action_.name)

    # Added intention to perform action.
    # Note: Technically, putting it here is when it is both intended and
    # being done.
    ActionProcessApi.AddIntendedAction(self.action_.name)

  def activated(self):
    if not self.queue_.HasToken(self.action_.name):
      return False
    for resource in self.action_.preconditions:
      if not ActionProcessApi.RobotOwnsResource(resource):
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
    self.action_.Interrupt()
    self.started_.RemoveToken(self.action_.name)
    self.interrupted_.AddToken(self.action_.name)

    # Remove intended resources.
    for resource in self.action_.preconditions:
      ActionProcessApi.RemoveIntendedResource(resource)

  def activated(self):
    if not self.started_.HasToken(self.action_.name):
      return False
    for resource in self.action_.preconditions:
      if not ActionProcessApi.RobotOwnsResource(resource):
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
      # Finishing from start.
      self.started_.RemoveToken(self.action_.name)

      # Remove intended resources.
      for resource in self.action_.preconditions:
        ActionProcessApi.RemoveIntendedResource(resource)

    elif self.interrupted_.HasToken(self.action_.name):
      # Finishing from interrupt.
      self.interrupted_.RemoveToken(self.action_.name)

    # Stop requested resources for this action.
    for resource in self.action_.preconditions:
      ActionProcessApi.RemoveRequestedResource(resource)

    self.finished_.AddToken(self.action_.name)

    # Removing the intended action.
    ActionProcessApi.RemoveIntendedAction(self.action_.name)


  def activated(self):
    return self.action_.IsFinished() and \
        (self.started_.HasToken(self.action_.name) \
        or self.interrupted_.HasToken(self.action_.name))

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
    self.transitions_.append(StartTransition('start', action, queue, started))
    self.transitions_.append(
        InterruptTransition('interrupt', action, started, interrupted))
    self.transitions_.append(
        FinishTransition('finish', action, started,interrupted, self.finished_))

    # Put action token in queue.
    queue.AddToken(self.action_.name)

    for resource in self.action_.preconditions:
      ActionProcessApi.AddIntendedResource(resource)
      ActionProcessApi.AddRequestedResource(resource)

  def EndCondition(self):
    return rospy.is_shutdown() or self.finished_.HasToken(self.action_.name)
