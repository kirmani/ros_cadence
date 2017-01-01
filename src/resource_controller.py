#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2016 kirmani <sean@kirmani.io>
#
# Distributed under terms of the MIT license.

"""
Timed Petri net for resource controller.
"""

from floor_listener import FloorListener

from ros_cadence.srv import *
from petri_net import *
from sets import Set
import rospy

kVerbose = True
kDebug = True

kPlaces = ['requested_robot', 'free', 'owned_user', 'owned_robot',
           'requested_user']

class ResourceControllerApi:
    @staticmethod
    def RemoveResourceFromPlace(place, token):
        rospy.wait_for_service('do_petri_net_arc')
        try:
            do_petri_net_arc = rospy.ServiceProxy(
                'do_petri_net_arc', DoPetriNetArc)
            return do_petri_net_arc('remove', place, token, None).response
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)
            return False

    @staticmethod
    def AddResourceToPlace(place, token):
        rospy.wait_for_service('do_petri_net_arc')
        try:
            do_petri_net_arc = rospy.ServiceProxy(
                'do_petri_net_arc', DoPetriNetArc)
            return do_petri_net_arc('add', place, token, None).response
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)
            return False

    @staticmethod
    def CheckGuard(place, token):
        rospy.wait_for_service('do_petri_net_arc')
        try:
            do_petri_net_arc = rospy.ServiceProxy(
                'do_petri_net_arc', DoPetriNetArc)
            return do_petri_net_arc('guard', place, token, None).response
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)
            return False

    @staticmethod
    def AddActiveAction(action):
        rospy.wait_for_service('do_petri_net_arc')
        try:
            do_petri_net_arc = rospy.ServiceProxy(
                'do_petri_net_arc', DoPetriNetArc)
            return do_petri_net_arc(
                'add_action', None, None, action).response
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)
            return False

    @staticmethod
    def RemoveActiveAction(action):
        rospy.wait_for_service('do_petri_net_arc')
        try:
            do_petri_net_arc = rospy.ServiceProxy(
                'do_petri_net_arc', DoPetriNetArc)
            return do_petri_net_arc(
                'remove_action', None, None, action).response
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)
            return False

class ReleaseRobotTransition(PetriNetTransition):
    def __init__(self, requested_robot, owned_robot, free, resource_listeners):
        PetriNetTransition.__init__(self, 'release_robot')
        self.requested_robot_ = requested_robot
        self.owned_robot_ = owned_robot
        self.free_ = free
        self.resource_listeners_ = resource_listeners

    def fire(self):
        for resource_listener in self.resource_listeners_:
            if (not self.requested_robot_.HasToken(resource_listener.name)
                and self.owned_robot_.HasToken(resource_listener.name)):
                if kVerbose:
                    print("Releasing resource (%s) from robot."
                          % resource_listener.name)
                self.owned_robot_.RemoveToken(resource_listener.name)
                self.free_.AddToken(resource_listener.name)

    def activated(self):
        for resource_listener in self.resource_listeners_:
            if (not self.requested_robot_.HasToken(resource_listener.name)
                and self.owned_robot_.HasToken(resource_listener.name)):
                return True
        return False

class YieldTransition(PetriNetTransition):
    def __init__(self, requested_user, owned_robot, owned_user,
                 resource_listeners):
        PetriNetTransition.__init__(self, 'yield')
        self.requested_user_ = requested_user
        self.owned_robot_ = owned_robot
        self.owned_user_ = owned_user
        self.resource_listeners_ = resource_listeners

    def fire(self):
        for resource_listener in self.resource_listeners_:
            if (self.requested_user_.HasToken(resource_listener.name)
                and self.owned_robot_.HasToken(resource_listener.name)):
                if kVerbose:
                    print("Yielding resource (%s) from robot to human."
                          % resource_listener.name)
                self.requested_user_.RemoveToken(resource_listener.name)
                self.owned_robot_.RemoveToken(resource_listener.name)
                self.owned_user_.AddToken(resource_listener.name)

    def activated(self):
        for resource_listener in self.resource_listeners_:
            if (self.requested_user_.HasToken(resource_listener.name)
                and self.owned_robot_.HasToken(resource_listener.name)):
                return True
        return False

class RequestUserTransition(PetriNetTransition):
    def __init__(self, owned_user, requested_user, resource_listeners, actions):
        PetriNetTransition.__init__(self, 'request_user')
        self.owned_user_ = owned_user
        self.requested_user_ = requested_user
        self.resource_listeners_ = resource_listeners
        self.actions_ = actions

    def fire(self):
        # Place resource tokens in requested place.
        for resource_listener in self.resource_listeners_:
            if (not resource_listener.Poll(self.actions_)
                and not self.requested_user_.HasToken(resource_listener.name)):
                if kVerbose:
                    print("Requesting resource for user: %s"
                          % resource_listener.name)
                self.requested_user_.AddToken(resource_listener.name)

    def activated(self):
        for resource_listener in self.resource_listeners_:
            if (not resource_listener.Poll(self.actions_)
                and not self.requested_user_.HasToken(resource_listener.name)):
                return True
        return False

class SeizeUserTransition(PetriNetTransition):
    def __init__(self, requested_user, free, owned_user, resource_listeners):
        PetriNetTransition.__init__(self, 'seize_user')
        self.requested_user_ = requested_user
        self.free_ = free
        self.owned_user_ = owned_user
        self.resource_listeners_ = resource_listeners

    def fire(self):
        for resource_listener in self.resource_listeners_:
            if (self.requested_user_.HasToken(resource_listener.name)
                and self.free_.HasToken(resource_listener.name)):
                if kVerbose:
                    print("Seizing resource for user: %s"
                          % resource_listener.name)
                self.free_.RemoveToken(resource_listener.name)
                self.owned_user_.AddToken(resource_listener.name)

    def activated(self):
        for resource_listener in self.resource_listeners_:
            if (self.requested_user_.HasToken(resource_listener.name)
                and self.free_.HasToken(resource_listener.name)):
                return True
        return False

class ReleaseUserTransition(PetriNetTransition):
    def __init__(self, requested_user, owned_user, free, resource_listeners,
                 actions):
        PetriNetTransition.__init__(self, 'release_user')
        self.requested_user_ = requested_user
        self.owned_user_ = owned_user
        self.free_ = free
        self.resource_listeners_ = resource_listeners
        self.actions_ = actions

    def fire(self):
        for resource_listener in self.resource_listeners_:
            if (resource_listener.Poll(self.actions_)
                and self.owned_user_.HasToken(resource_listener.name)):
                if kVerbose:
                    print("Releasing resource for user: %s"
                          % resource_listener.name)
                self.requested_user_.RemoveToken(resource_listener.name)
                self.owned_user_.RemoveToken(resource_listener.name)
                self.free_.AddToken(resource_listener.name)

    def activated(self):
        for resource_listener in self.resource_listeners_:
            if (resource_listener.Poll(self.actions_)
                and self.owned_user_.HasToken(resource_listener.name)):
                return True
        return False

class ResourceController(PetriNet):
    def __init__(self, resource_listeners):
        PetriNet.__init__(self, 'resource_controller')
        self.places_ = {}
        self.actions_ = Set()
        self.beliefs_ = {}

        self.resource_listeners_ = resource_listeners

        # Places.
        for place in kPlaces:
            self.places_[place] = PetriNetPlace(place)

        # Transitions.
        self.transitions_.append(
            ReleaseRobotTransition(self.places_['requested_robot'],
                                   self.places_['owned_robot'],
                                   self.places_['free'],
                                   self.resource_listeners_))
        self.transitions_.append(
            YieldTransition(self.places_['requested_user'],
                            self.places_['owned_robot'],
                            self.places_['owned_user'],
                            self.resource_listeners_))
        self.transitions_.append(
            RequestUserTransition(self.places_['owned_user'],
                                  self.places_['requested_user'],
                                  self.resource_listeners_,
                                  self.actions_))
        self.transitions_.append(
            SeizeUserTransition(self.places_['requested_user'],
                                self.places_['free'],
                                self.places_['owned_user'],
                                self.resource_listeners_))
        self.transitions_.append(
            ReleaseUserTransition(self.places_['requested_user'],
                                  self.places_['owned_user'],
                                  self.places_['free'],
                                  self.resource_listeners_,
                                  self.actions_))

        for resource_listener in self.resource_listeners_:
            self.places_['free'].AddToken(resource_listener.name)

    def AddTokenToPlace(self, place, token):
        if place not in self.places_:
            raise ValueError("Does not have place: %s" % place)
        self.places_[place].AddToken(token)
        self.Run()
        if kDebug:
            print("Marking after adding token (%s) to place (%s): %s"
                  % (token, place, str(resource_controller.GetMarking())))

    def HasTokenInPlace(self, place, token):
        self.Run()
        if place not in self.places_:
            raise ValueError("Does not have place: %s" % place)
        return self.places_[place].HasToken(token)

    def RemoveTokenFromPlace(self, place, token):
        if place not in self.places_:
            raise ValueError("Does not have place: %s" % place)
        remove_successful = self.places_[place].RemoveToken(token)
        self.Run()
        if kDebug:
            print("Marking after removing token (%s) to place (%s): %s"
                  % (token, place, str(resource_controller.GetMarking())))
        return remove_successful

    def GetMarking(self):
        marking = {}
        for place in self.places_:
            marking[place] = self.places_[place].GetTokens()
        return marking

    def AddActiveAction(self, action):
        if kDebug:
            print("Adding active action: %s" % action)
        self.actions_.add(action)

    def RemoveActiveAction(self, action):
        if kDebug:
            print("Removing active action: %s" % action)
        if action not in self.actions_:
            return False
        self.actions_.remove(action)
        return True

def handle_do_petri_net_arc(req):
    if req.function == 'add':
        resource_controller.AddTokenToPlace(req.place, req.token)
        return DoPetriNetArcResponse(True)
    if req.function == 'remove':
        return DoPetriNetArcResponse(
        resource_controller.RemoveTokenFromPlace(req.place, req.token))
    if req.function == 'guard':
        response = resource_controller.HasTokenInPlace(req.place, req.token)
        return DoPetriNetArcResponse(response)
    if req.function == 'add_action':
        resource_controller.AddActiveAction(req.action)
        return DoPetriNetArcResponse(True)
    if req.function == 'remove_action':
        response = resource_controller.RemoveActiveAction(req.action)
        return DoPetriNetArcResponse(response)
    raise rospy.ServiceException("Invalid function input: %s" % req.function)

def OnShutdown():
    print('shutting down!')

def main():
    global resource_controller

    # Add resource listeners.
    resource_listeners = Set()
    # floor_listener = FloorListener()
    # resource_listeners.add(floor_listener)

    resource_controller = ResourceController(resource_listeners)
    if kDebug:
        print("Initial marking: %s" % str(resource_controller.GetMarking()))

    rospy.init_node('do_petri_net_arc')
    s = rospy.Service('do_petri_net_arc', DoPetriNetArc, handle_do_petri_net_arc)
    print("Ready to do petri net arcs.")
    rospy.on_shutdown(OnShutdown)
    rospy.spin()

if __name__ == '__main__':
  main()
