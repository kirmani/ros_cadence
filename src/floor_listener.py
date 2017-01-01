#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2016 kirmani <sean@kirmani.io>
#
# Distributed under terms of the MIT license.

"""
PyAudio Test
"""

# from petri_net import *
# from resource_controller import ResourceControllerApi
from collections import deque
from resource_listener import ResourceListener
from sampler import Sampler
import json
import math
import os
import rospy
import pyaudio
import struct
import time

kVerbose = True
kSensesFile = os.path.join(os.path.dirname(__file__),
            'data/senses.json')

class FloorListener(ResourceListener):
    def __init__(self):
        self.active_ = rospy.get_param('active', True)
        minimum_block_time = 0.05
        self.user_speaking_ = False
        self.robot_speaking_count_ = 0
        self.user_speaking_count_ = 0
        self.response_delay_ = 1.0
        self.lapse_tolerance_ = 0.5 if self.active_ else 4.0
        self.floor_factor_ = 2.0 if self.active_ else 0.5
        self.can_interrupt_user_ = self.active_
        self.can_interrupt_self_ = not self.active_
        self.lapse_deque_ = deque()
        self.conflict_tolerance_ = 1.0
        self.conflict_deque_ = deque()
        self.last_time_without_lapse_ = 0
        self.last_time_without_conflict_ = 0
        self.conflict_threshold_ = 0.9
        self.lapse_threshold_ = 0.5
        self.user_start_time_ = 0.0
        self.robot_start_time_ = 0.0
        self.robot_time_ = 0.0
        self.user_time_ = 0.0
        self.volume_factor_ = 0.5
        self.robot_speaking_ = False
        self.floor_is_free_ = True
        self.last_yield_time_ = 0.0

        # constants
        ResourceListener.__init__(self, 'floor')
        self.format_ = pyaudio.paInt16
        self.channels_ = 2
        self.rate_ = 44100
        self.input_frames_per_block_ = int(self.rate_ * minimum_block_time)
        self.short_normalize_ = (1.0 / 32768.0)

        self.pa_ = pyaudio.PyAudio()
        self.stream_ = self.open_mic_stream()
        self.floor_holding_threshold_ = 0.01
        self.error_count_ = 0
        self.last_update_time_ = 0
        self.samplers_ = self.LoadSenses_()

    def LoadSenses_(self):
        try:
            with open(kSensesFile, 'r+') as f:
                senses_file = json.load(fp=f)
            samplers = {}
            for actions_hash in senses_file:
                save_data = senses_file[actions_hash]
                samplers[actions_hash] = Sampler()
                samplers[actions_hash].LoadFromSaveData(save_data)
            return samplers
        except IOError:
            print("Error loading senses file.")
            return {}

    def find_input_device(self):
        device_index = None
        for i in range(self.pa_.get_device_count()):
            devinfo = self.pa_.get_device_info_by_index(i)
            for keyword in ['mic', 'input']:
                if keyword in devinfo['name'].lower():
                    print("Found an input: device %d - %s" % (i, devinfo['name']))
                    device_index = i
                    return device_index
        if device_index == None:
            print("No preferred input found. Using default input device.")
        return device_index

    def open_mic_stream(self):
        device_index = self.find_input_device()
        stream = self.pa_.open(format=self.format_,
                          channels=self.channels_,
                          rate=self.rate_,
                          input=True,
                          input_device_index=device_index,
                          frames_per_buffer=self.input_frames_per_block_)
        return stream

    def Poll(self, actions):
        "Returns True if the floor is free"
        try:
            if self.stream_.get_read_available() < self.input_frames_per_block_:
                return self.floor_is_free_
            block = self.stream_.read(self.input_frames_per_block_)
        except IOError, e:
            # Damnit.
            self.error_count_ += 1
            print("(%d) Error recording: %s" % (self.error_count_, e))
            return

        # actions_hash = ', '.join(actions)
        actions_hash = '' if 'speech' not in actions else 'speech'
        if actions_hash not in self.samplers_:
            self.samplers_[actions_hash] = Sampler()
        now = time.time()

        amplitude = self.GetRms_(block)
        self.samplers_[actions_hash].Sample(amplitude)

        self.conflict_deque_.append((amplitude, now))
        while now - self.conflict_deque_[0][1] > self.conflict_tolerance_:
            self.conflict_deque_.popleft()
        conflict_threshold_volume = self.samplers_[actions_hash].expectation_ \
                + self.volume_factor_ \
                * math.sqrt(self.samplers_[actions_hash].variance_)
        num_above_threshold = 0
        for sample in self.conflict_deque_:
            if sample[0] > conflict_threshold_volume:
                num_above_threshold += 1
        conflict_confidence = float(num_above_threshold) \
                / float(len(self.conflict_deque_))
        # print("conflict confidence: %s" % conflict_confidence)

        self.lapse_deque_.append((amplitude, now))
        while now - self.lapse_deque_[0][1] > self.lapse_tolerance_:
            self.lapse_deque_.popleft()
        lapse_threshold_volume = self.samplers_[actions_hash].expectation_ \
                + self.volume_factor_ \
                * math.sqrt(self.samplers_[actions_hash].variance_)
        num_below_threshold = 0
        for sample in self.lapse_deque_:
            if sample[0] < lapse_threshold_volume:
                num_below_threshold += 1
        lapse_confidence = float(num_below_threshold) \
                / float(len(self.lapse_deque_))
        # print("lapse confidence: %s" % lapse_confidence)

        # Determine if robot is speaking.
        if not self.robot_speaking_ and 'speech' in actions:
            self.robot_start_time_ = now
            self.robot_speaking_ = True

        if self.robot_speaking_ and 'speech' not in actions:
            self.robot_speaking_ = False
            self.robot_time_ += now - self.robot_start_time_

        # Determine if user is speaking.
        if not self.robot_speaking_ and not self.user_speaking_:
            if lapse_confidence < self.lapse_threshold_:
                self.user_speaking_ = True
                self.user_start_time_ = now

        if not self.robot_speaking_ and self.user_speaking_:
            if lapse_confidence > self.lapse_threshold_:
                self.user_speaking_ = False
                self.user_time_ += now - self.user_start_time_

        if self.robot_speaking_ and not self.user_speaking_:
            if conflict_confidence > self.conflict_threshold_:
                self.user_speaking_ = True
                self.user_start_time_ = now

        if self.robot_speaking_ and self.user_speaking_:
            if conflict_confidence < self.conflict_threshold_:
                self.user_speaking_ = False
                self.user_time_ += now - self.user_start_time_

        if self.user_speaking_ or self.robot_speaking_:
            self.last_time_without_lapse_ = now

        if not (self.user_speaking_ and self.robot_speaking_):
            self.last_time_without_conflict_ = now

        # print ("user speaking: %s, robot speaking: %s"
        #         % (self.user_speaking_, self.robot_speaking_))

        # print ("user time: %s, robot time: %s"
        #         % (self.user_time_, self.robot_time_))

        robot_time = self.robot_time_ if self.robot_speaking_ \
                else self.robot_time_ + 5.0
        user_time = self.user_time_ if self.user_time_ > 0.001 else 0.001
        floor_ratio = robot_time / user_time
        # print("floor ratio: %s" % floor_ratio)

        floor_should_be_free = True
        if floor_ratio > self.floor_factor_:
            # print("floor ratio not met")
            floor_should_be_free = False

        if self.can_interrupt_self_:
            if now - self.last_time_without_conflict_ > self.conflict_tolerance_:
                # Conflicted long enough. Robot can't use floor anymore.
                floor_should_be_free = False

        if now - self.last_time_without_lapse_ < self.lapse_tolerance_:
            floor_should_be_free = False

        if floor_should_be_free:
            if not self.floor_is_free_:
                self.last_yield_time_ = now
            self.floor_is_free_ = True
        else:
            if now - self.last_yield_time_ > self.response_delay_:
                self.floor_is_free_ = False
        return self.floor_is_free_

    def GetRms_(self, block):
        # We will get one short out for each two chars in the string.
        count = len(block) / 2
        format = "%dh" % count
        shorts = struct.unpack(format, block)

        # Iterate over the block.
        sum_squares = 0.0
        for sample in shorts:
            # Sample is a signed short in +/- 32768. Normalize it to 1.0.
            n = sample * self.short_normalize_
            sum_squares += n * n

        return math.sqrt(sum_squares / count)

    def OnShutdown(self):
        save_data = {}
        for actions_hash in self.samplers_:
            save_data[actions_hash] = self.samplers_[actions_hash].GetSaveData()
            self.WriteSenses_(save_data)
        if kVerbose:
            print("saving to long term memory")
            print(save_data)

    def WriteSenses_(self, data):
        j = json.dumps(data, indent=4)
        with open(kSensesFile, 'w') as f:
            f.write(j)
