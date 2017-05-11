import sys
import rospy
import numpy
import rosbag

import core.utils as utils
from core.classes import HandlePts, Kalman, RealTimePlot

from python.ransac import Ransac


if len(sys.argv) == 1:
  raise Exception('Value of parameter which not defined...')
else:
  which = sys.argv[1]

rtp = RealTimePlot(
                     num_point_type=2
                   , num_line_type=4
                   , num_dash_type=1
    )

hp = HandlePts()

kalman = Kalman(1.0,500.0)

x = numpy.matrix([[0.0],[0.0]])
P = numpy.matrix([[1e4,0],[0,1e4]])

bag = rosbag.Bag('../bags/output.bag')
for topic, msg, t in bag.read_messages(start_time=rospy.rostime.Time(1310046780)):
#  for topic, msg, t in bag.read_messages(start_time=rospy.rostime.Time(1310046770)):
  angle = msg.angle_min
  data = {'L':[], 'R':[]}
  for radius in msg.ranges:
    if radius > msg.range_min and radius < msg.range_max:
      pt = utils.polar2cartesian(radius, angle)
      key = hp.selector(pt,utils.fromTwo2Three(x))
      if key:
        data[key].append(pt)
      angle += msg.angle_increment


  ### RANSAC ###
  print 'RANSAC'
  outliers_ratio = 0.30
  prob_samples_free = 0.99
  iterations = 50
  threshold = 0.25
  ratio =  (1 - outliers_ratio)*0.95

  print 'Outliers ratio:', outliers_ratio
  print 'Total number of iterations:', iterations
  print 'Threshold:', threshold
  print 'Inliers ratio:', ratio

  xx,yy = zip(*data['L'])
  rcLeft = Ransac(iterations, threshold, ratio, xx, yy, which)
  rcLeft.search()
  moLeft = utils.fromTwo2Three((rcLeft.m_reg, rcLeft.c_reg))

  xx,yy = zip(*data['R'])
  rcRight = Ransac(iterations, threshold, ratio, xx, yy, which)
  rcRight.search()
  moRight = utils.fromTwo2Three((rcRight.m_reg, rcRight.c_reg))

  model = utils.computeBisectrix(moLeft,moRight)
  z = numpy.matrix(utils.fromThree2Two(model)).T
  xpre = x
  x, P = kalman.step(x,P,z)

  # Visualization
  rtp.plotPoint(*zip(*data['L']),index=0)
  rtp.plotPoint(*zip(*data['R']),index=1)

  rtp.plotLine(*zip(*utils.pointsFromModel(moLeft)), index=0)
  rtp.plotLine(*zip(*utils.pointsFromModel(moRight)), index=1)
  rtp.plotLine(*zip(*utils.pointsFromModel(z)), index=2)
  #rtp.plotLine(*zip(*utils.pointsFromModel(x)), index=3)

  #rtp.plotDash(*zip(*utils.pointsFromModel(xpre)), index=0)

#    raw_input('press enter')
