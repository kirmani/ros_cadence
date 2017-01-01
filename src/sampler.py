#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2016 kirmani <sean@kirmani.io>
#
# Distributed under terms of the MIT license.

"""
Sampler.
"""

import math

kVerbose = False

class Sampler:
  def __init__(self):
    self.started_ = False
    self.expectation_ = 0
    self.m2_ = 0
    self.variance_ = 0
    self.population_variance_ = 0
    self.num_samples_ = 0
    self.error_count_ = 0

  def IsConfidentValue(self, value):
    if self.num_samples_ > 1:
      # determine confidence z-value
      z_score = (value - self.expectation_) / math.sqrt(self.variance_)
      z_cutoff = 1.96
      confidence_range = z_cutoff * math.sqrt(self.variance_)
      lower_bound = self.expectation_ - confidence_range
      upper_bound = self.expectation_ + confidence_range
      # if kVerbose:
      #   print("Confidence range: (%.9f, %.9f)"
      #         % (lower_bound, upper_bound))
      if value < lower_bound:
        if kVerbose:
          print("Value below confidence range")
        return -1
      if value > upper_bound:
        if kVerbose:
          print("Value above confidence range")
        return 1
      return 0

  def Expectation(self):
    return self.expectation_

  def SampleVariance(self):
    return self.variance_

  def GetSaveData(self):
    return {
        'expectation': self.expectation_,
        'm2': self.m2_,
        'num_samples': self.num_samples_}

  def LoadFromSaveData(self, data):
    self.started_ = True
    self.expectation_ = data['expectation']
    self.m2_ = data['m2']
    self.num_samples_ = data['num_samples']
    self.variance_ = self.m2_ / (self.num_samples_ - 1) \
        if self.num_samples_ > 1 else 0
    self.population_variance_ = self.m2_ / (self.num_samples_)

  def Sample(self, value):
    self.num_samples_ += 1
    error = value - self.expectation_
    self.expectation_ += error / self.num_samples_
    self.m2_ += error * (value - self.expectation_)

    self.variance_ = self.m2_ / (self.num_samples_ - 1) \
        if self.num_samples_ > 1 else 0
    self.population_variance_ = self.m2_ / (self.num_samples_)
    # print("Error ratio: %.9f" % (float(self.error_count_) / float(self.num_samples_)))

    # print("Expectation: %.9f" % self.expectation_)
    # print("Sample Variance: %.9f" % self.variance_)
    # print("Population Variance: %.9f" % self.population_variance_)
