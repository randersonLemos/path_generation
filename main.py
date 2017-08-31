import os
import sys
import time
import copy
import rospy
import numpy
import shutil
import rosbag
import warnings
import core.utils as utils
from sklearn import svm
from core.classes import Ransac, HandlePts, Kalman, RealTimePlot, AppendFile

numberIterations = 1400

hpdic = {
          'xmin': -1.0
         ,'xmax':  6.0
         ,'ymin': -2.5
         ,'ymax':  2.5
        }

kalmandic = {
              'A': numpy.matrix([[0.98, 0.0],[0.0, 0.98]])
             ,'Q': numpy.matrix([[1.0, 0.0],[0.0, 1.0]])
             ,'R': numpy.matrix([[150.0, 0.0],[0.0, 150.0]])
            }

rtp = RealTimePlot(
                    num_point_type = 5
                   ,num_line_type = 4
                   ,num_dash_type = 1
                   ,save_images = False
                   #,dir_name = dir_name
                  )

hp = HandlePts(
                xmin = hpdic['xmin']
               ,xmax = hpdic['xmax']
               ,ymin = hpdic['ymin']
               ,ymax = hpdic['ymax']
              )

kalman = Kalman(
                 A = kalmandic['A']
                ,Q = kalmandic['Q'] # model covariance matrix
                ,R = kalmandic['R'] # measure covariance matrix
               )

x = numpy.matrix([[0.0],[0.0]])
z = numpy.matrix([[0.0],[0.0]])
P = numpy.matrix([[1e4,0],[0,1e4]])

time_ = 1310046785
#time_ = 1310046820
bag = rosbag.Bag('bags/output.bag')
for count, (topic, msg, t) in enumerate(bag.read_messages(start_time=rospy.rostime.Time(time_))):

  angle = msg.angle_min
  data = {'ALL':[], 'L':[], 'R':[]}
  for radius in msg.ranges:
    if radius > msg.range_min and radius < msg.range_max:
      pt = utils.polar2cartesian(radius, angle)
      data['ALL'].append(pt)
      key = hp.selector(pt,utils.fromTwo2Three(x))
      if key:
        data[key].append(pt)
      angle += msg.angle_increment

  XX = data['L'] + data['R']
  yy = len(data['L'])*[0] + len(data['R'])*[1]

  clf = svm.LinearSVC(C=0.01, fit_intercept=True, dual=False)
  clf.fit(XX,yy)

  w = clf.coef_[0]
  a = -w[0]/w[1]
  b = -(clf.intercept_[0])/w[1]

  z = numpy.matrix([a,b]).T

  x, P = kalman.step(x,P,z)

  # Visualization
  rtp.plotPoint(*zip(*data['ALL']),index=0)
  rtp.plotPoint(*zip(*data['L']),index=1)
  rtp.plotPoint(*zip(*data['R']),index=2)
  rtp.plotLine(*zip(*utils.pointsFromModel(z)), index=2)
  rtp.plotLine(*zip(*utils.pointsFromModel(x)), index=3)
  rtp.update()

  if count%100 == 0: print 'count: {}'.format(count)

  if count > numberIterations: break
