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

class BeliefControllerApi:
    @staticmethod
    def SetBelief(belief, value):
        rospy.wait_for_service('set_belief')
        try:
            set_belief = rospy.ServiceProxy(
                'set_belief', DoPetriNetArc)
            set_belief(belief, value)
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)

    @staticmethod
    def GetBelief(belief):
        rospy.wait_for_service('get_belief')
        try:
            get_belief = rospy.ServiceProxy(
                'get_belief', DoPetriNetArc)
            return get_belief(belief).value
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)
            return False

class ActionProcessApi:
    @staticmethod
    def AddIntendedAction(action):
        rospy.wait_for_service('add_intended_action')
        try:
            add_intended_action = rospy.ServiceProxy(
                'add_intended_action', Resource)
            add_intended_action(action)
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)

    @staticmethod
    def RemoveIntendedAction(action):
        rospy.wait_for_service('remove_intended_action')
        try:
            remove_intended_action = rospy.ServiceProxy(
                'remove_intended_action', Resource)
            remove_intended_action(action)
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)

    @staticmethod
    def AddIntendedResource(resource):
        rospy.wait_for_service('add_intended_resource')
        try:
            add_intended_resource = rospy.ServiceProxy(
                'add_intended_resource', Resource)
            add_intended_resource(resource)
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)

    @staticmethod
    def RemoveIntendedResource(resource):
        rospy.wait_for_service('remove_intended_resource')
        try:
            remove_intended_resource = rospy.ServiceProxy(
                'remove_intended_resource', Resource)
            remove_intended_resource(resource)
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)

    @staticmethod
    def AddRequestedResource(resource):
        rospy.wait_for_service('add_requested_resource')
        try:
            add_requested_resource = rospy.ServiceProxy(
                'add_requested_resource', Resource)
            add_requested_resource(resource)
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)

    @staticmethod
    def RemoveRequestedResource(resource):
        rospy.wait_for_service('remove_requested_resource')
        try:
            remove_requested_resource = rospy.ServiceProxy(
                'remove_requested_resource', Resource)
            remove_requested_resource(resource)
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)

    @staticmethod
    def RobotOwnsResource(resource):
        rospy.wait_for_service('robot_owns_resource')
        try:
            robot_owns_resource = rospy.ServiceProxy(
                'robot_owns_resource', GetBelief)
            return robot_owns_resource(resource).value
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)

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
    def __init__(self, requested_robot, owned_robot, free, beliefs):
        PetriNetTransition.__init__(self, 'release_robot')
        self.requested_robot_ = requested_robot
        self.owned_robot_ = owned_robot
        self.free_ = free
        self.beliefs_ = beliefs

    def fire(self):
        for belief in self.beliefs_:
            if (not self.requested_robot_.HasToken(belief)
                and self.owned_robot_.HasToken(belief)):
                if kVerbose:
                    print("Releasing resource (%s) from robot."
                          % belief)
                self.owned_robot_.RemoveToken(belief)
                self.free_.AddToken(belief)

    def activated(self):
        for belief in self.beliefs_:
            if (not self.requested_robot_.HasToken(belief)
                and self.owned_robot_.HasToken(belief)):
                return True
        return False

class YieldTransition(PetriNetTransition):
    def __init__(self, requested_user, owned_robot, owned_user, beliefs):
        PetriNetTransition.__init__(self, 'yield')
        self.requested_user_ = requested_user
        self.owned_robot_ = owned_robot
        self.owned_user_ = owned_user
        self.beliefs_ = beliefs

    def fire(self):
        for belief in self.beliefs_:
            if (self.requested_user_.HasToken(belief)
                and self.owned_robot_.HasToken(belief)):
                if kVerbose:
                    print("Yielding resource (%s) from robot to human."
                          % belief)
                self.requested_user_.RemoveToken(belief)
                self.owned_robot_.RemoveToken(belief)
                self.owned_user_.AddToken(belief)

    def activated(self):
        for belief in self.beliefs_:
            if (self.requested_user_.HasToken(belief)
                and self.owned_robot_.HasToken(belief)):
                return True
        return False

class RequestUserTransition(PetriNetTransition):
    def __init__(self, owned_user, requested_user, beliefs):
        PetriNetTransition.__init__(self, 'request_user')
        self.owned_user_ = owned_user
        self.requested_user_ = requested_user
        self.beliefs_ = beliefs

    def fire(self):
        # Place resource tokens in requested place.
        for belief in self.beliefs_:
            if (not self.beliefs_[belief]
                and not self.requested_user_.HasToken(belief)):
                if kVerbose:
                    print("Requesting resource for user: %s"
                          % belief)
                self.requested_user_.AddToken(belief)

    def activated(self):
        for belief in self.beliefs_:
            if (not self.beliefs_[belief]
                and not self.requested_user_.HasToken(belief)):
                return True
        return False

class SeizeUserTransition(PetriNetTransition):
    def __init__(self, requested_user, free, owned_user, beliefs):
        PetriNetTransition.__init__(self, 'seize_user')
        self.requested_user_ = requested_user
        self.free_ = free
        self.owned_user_ = owned_user
        self.beliefs_ = beliefs

    def fire(self):
        for belief in self.beliefs_:
            if (self.requested_user_.HasToken(belief)
                and self.free_.HasToken(belief)):
                if kVerbose:
                    print("Seizing resource for user: %s"
                          % belief)
                self.free_.RemoveToken(belief)
                self.owned_user_.AddToken(belief)

    def activated(self):
        for belief in self.beliefs_:
            if (self.requested_user_.HasToken(belief)
                and self.free_.HasToken(belief)):
                return True
        return False

class ReleaseUserTransition(PetriNetTransition):
    def __init__(self, requested_user, owned_user, free, beliefs):
        PetriNetTransition.__init__(self, 'release_user')
        self.requested_user_ = requested_user
        self.owned_user_ = owned_user
        self.free_ = free
        self.beliefs_ = beliefs

    def fire(self):
        for belief in self.beliefs_:
            if (self.beliefs_[belief]
                and self.owned_user_.HasToken(belief)):
                if kVerbose:
                    print("Releasing resource for user: %s"
                          % belief)
                self.requested_user_.RemoveToken(belief)
                self.owned_user_.RemoveToken(belief)
                self.free_.AddToken(belief)

    def activated(self):
        for belief in self.beliefs_:
            if (self.beliefs_[belief]
                and self.owned_user_.HasToken(belief)):
                return True
        return False

class SeizeRobotTransition(PetriNetTransition):
    def __init__(self, requested_robot, free, owned_robot, intended_resources):
        PetriNetTransition.__init__(self, 'seize_robot')
        self.requested_robot_ = requested_robot
        self.free_ = free
        self.owned_robot_ = owned_robot
        self.intended_resources_ = intended_resources

    def fire(self):
        for resource in self.intended_resources_:
            if (self.requested_robot_.HasToken(resource)
                and self.free_.HasToken(resource)):
                self.free_.RemoveToken(resource)
                self.owned_robot_.AddToken(resource)

    def activated(self):
        for resource in self.intended_resources_:
            if (self.requested_robot_.HasToken(resource)
                and self.free_.HasToken(resource)):
                return True
        return False

class ResourceController(PetriNet):
    def __init__(self):
        PetriNet.__init__(self, 'resource_controller')
        self.places_ = {}
        self.intended_resources_ = []
        self.intended_actions_ = []
        self.actions_ = Set()
        self.beliefs_ = {}

        # Places.
        for place in kPlaces:
            self.places_[place] = PetriNetPlace(place)

        # Transitions.
        self.transitions_.append(
            SeizeRobotTransition(self.places_['requested_robot'],
                                 self.places_['free'],
                                 self.places_['owned_robot'],
                                 self.intended_resources_))
        self.transitions_.append(
            ReleaseRobotTransition(self.places_['requested_robot'],
                                   self.places_['owned_robot'],
                                   self.places_['free'],
                                   self.beliefs_))
        self.transitions_.append(
            YieldTransition(self.places_['requested_user'],
                            self.places_['owned_robot'],
                            self.places_['owned_user'],
                            self.beliefs_))
        self.transitions_.append(
            RequestUserTransition(self.places_['owned_user'],
                                  self.places_['requested_user'],
                                  self.beliefs_))
        self.transitions_.append(
            SeizeUserTransition(self.places_['requested_user'],
                                self.places_['free'],
                                self.places_['owned_user'],
                                self.beliefs_))
        self.transitions_.append(
            ReleaseUserTransition(self.places_['requested_user'],
                                  self.places_['owned_user'],
                                  self.places_['free'],
                                  self.beliefs_))

    # Perception Process API.
    def SetBelief(self, belief, value):
        if belief not in self.beliefs_:
            if value:
                self.places_['free'].AddToken(belief)
            else:
                self.places_['owned_user'].AddToken(belief)
            print(self.GetMarking())
        self.beliefs_[belief] = value
        print("SetBelief: %s -> %s" % (belief, value))
        print(self.beliefs_)

    def GetBelief(self, belief):
        if belief not in self.beliefs_:
            raise ValueError("GetBelief: Error: belief (%s) does not exist"
                             % belief)
        return self.beliefs_[belief]

    # Action Process API.
    def AddIntendedAction(self, action):
        self.intended_actions_.append(action)
        print("AddIntendedAction: added intended action (%s)"
              % action)
        print(self.intended_actions_)
        self.Run()
        print(self.GetMarking())

    def RemoveIntendedAction(self, action):
        self.intended_actions_.remove(action)
        print("RemoveIntendedAction: removed intended action (%s)"
              % action)
        print(self.intended_actions_)
        self.Run()
        print(self.GetMarking())

    def AddIntendedResource(self, resource):
        self.intended_resources_.append(resource)
        print("AddIntendedResource: added intended resource (%s)"
              % resource)
        print(self.intended_resources_)
        self.Run()
        print(self.GetMarking())

    def RemoveIntendedResource(self, resource):
        self.intended_resources_.remove(resource)
        print("RemoveIntendedResource: removed intended resource (%s)"
              % resource)
        print(self.intended_resources_)

    def AddRequestedResource(self, resource):
        self.places_['requested_robot'].AddToken(resource)
        print("AddRequestedResource: added requested resource (%s)"
              % resource)
        self.Run()
        print(self.GetMarking())

    def RemoveRequestedResource(self, resource):
        self.places_['requested_robot'].RemoveToken(resource)
        print("RemoveRequestedResource: removed requested resource (%s)"
              % resource)
        self.Run()
        print(self.GetMarking())

    def RobotOwnsResource(self, resource):
        self.Run()
        # print(self.GetMarking())
        return self.places_['owned_robot'].HasToken(resource)

    # TODO(kirmani): Deprecate all these.
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

def set_belief_handler(req):
    resource_controller.SetBelief(req.belief, req.value)
    return SetBeliefResponse()

def get_belief_handler(req):
    return GetBeliefResponse(resource_controller.GetBelief(req.belief))

def add_intended_action_handler(req):
    resource_controller.AddIntendedAction(req.resource)
    return ResourceResponse()

def remove_intended_action_handler(req):
    resource_controller.RemoveIntendedAction(req.resource)
    return ResourceResponse()

def add_intended_resource_handler(req):
    resource_controller.AddIntendedResource(req.resource)
    return ResourceResponse()

def remove_intended_resource_handler(req):
    resource_controller.RemoveIntendedResource(req.resource)
    return ResourceResponse()

def add_requested_resource_handler(req):
    resource_controller.AddRequestedResource(req.resource)
    return ResourceResponse()

def remove_requested_resource_handler(req):
    resource_controller.RemoveRequestedResource(req.resource)
    return ResourceResponse()

def robot_owns_resource_handler(req):
    return GetBeliefResponse(
        resource_controller.RobotOwnsResource(req.belief))

def OnShutdown():
    print('shutting down!')

def main():
    global resource_controller

    resource_controller = ResourceController()
    if kDebug:
        print("Initial marking: %s" % str(resource_controller.GetMarking()))

    rospy.init_node('do_petri_net_arc')
    s = rospy.Service('do_petri_net_arc', DoPetriNetArc, handle_do_petri_net_arc)

    # TODO(kirmani): Rename Resource.srv to String.srv
    # TODO(kirmani): Rename GetBelief.srv to StringBool.srv or something
    # better.
    set_belief_service = rospy.Service(
        'set_belief', SetBelief, set_belief_handler)
    get_belief_service = rospy.Service(
        'get_belief', GetBelief, get_belief_handler)
    add_intended_action_service = rospy.Service(
        'add_intended_action', Resource, add_intended_action_handler)
    remove_intended_action_service = rospy.Service(
        'remove_intended_action', Resource, remove_intended_action_handler)
    add_intended_resource_service = rospy.Service(
        'add_intended_resource', Resource, add_intended_resource_handler)
    remove_intended_resource_service = rospy.Service(
        'remove_intended_resource', Resource, remove_intended_resource_handler)
    add_requested_resource_service = rospy.Service(
        'add_requested_resource', Resource, add_requested_resource_handler)
    remove_requested_resource_service = rospy.Service(
        'remove_requested_resource', Resource, remove_requested_resource_handler)
    robot_owns_resource_service = rospy.Service(
        'robot_owns_resource', GetBelief,
        robot_owns_resource_handler)

    print("Ready to do petri net arcs.")
    rospy.on_shutdown(OnShutdown)
    rospy.spin()

if __name__ == '__main__':
  main()
