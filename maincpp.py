import sys
import time
import copy
import rospy
import numpy
import rosbag
import warnings
import core.utils as utils
from core.classes import Ransac, HandlePts, Kalman, RealTimePlot

if len(sys.argv) == 2:
  if not sys.argv[1] in ('True', 'False'):
    raise Exception("The parameter save is or True or False...")
  save = sys.argv[1] in ('True')
elif len(sys.argv) == 1:
  warnings.warn("Boolean parameter save not specified.\nUsing the default value, which is False...")
  save = False
else:
  raise Exception(" Too many parameters...")

rtp = RealTimePlot(
                     num_point_type=5
                   , num_line_type=4
                   , num_dash_type=1
                   , save_images=save
                   , dir_name= 'png/' + str(int(time.time()))
                  )

ransac = {'L':Ransac(), 'R':Ransac()}

hp = HandlePts()

kalman = Kalman(
                  A = numpy.matrix([[0.970, 0.0],[0.0, 0.980]])
                , Q = numpy.matrix([[1.0, 0.0],[0.0, 1.0]]) # model covariance matrix
                , R = numpy.matrix([[250.0, 0.0],[0.0, 250.0]]) # measure covariance matrix
               )

x = numpy.matrix([[0.0],[0.0]])
z = numpy.matrix([[0.0],[0.0]])
P = numpy.matrix([[1e4,0],[0,1e4]])

X = []
Time = []

time = 1310046785
#time = 1310046820
bag = rosbag.Bag('../bags/output.bag')
for count, (topic, msg, t) in enumerate(bag.read_messages(start_time=rospy.rostime.Time(time))):
#for topic, msg, t in bag.read_messages(start_time=rospy.rostime.Time(1310046785571381000)):
  Time.append(t.to_time())

  angle = msg.angle_min
  data = {'ALL':[], 'L':[], 'R':[]}
  for radius in msg.ranges:
    if radius > msg.range_min and radius < msg.range_max:
      pt = utils.polar2cartesian(radius, angle)
      #pt = (1.5+pt[0],pt[1]) # shifting points
      #if pt[0] >= 0: data['ALL'].append(pt)
      data['ALL'].append(pt)
      key = hp.selector(pt,utils.fromTwo2Three(x))
      if key:
        data[key].append(pt)
      angle += msg.angle_increment


  ransac['L'].run(data['L'], 0)
  ransac['R'].run(data['R'], 1)
  #ransac['L'].run(data['L'])
  #ransac['R'].run(data['R'])
  
  model = utils.computeBisectrix(ransac['L'].model,ransac['R'].model)
  z = numpy.matrix(utils.fromThree2Two(model)).T
  x, P = kalman.step(x,P,z)
  X.append(numpy.squeeze(numpy.asarray(x)))


  # Visualization
  rtp.plotPoint(*zip(*data['ALL']),index=0)

  rtp.plotPoint(*zip(*data['L']),index=1)
  rtp.plotPoint(*zip(*data['R']),index=2)

  rtp.plotPoint(*zip(*ransac['L'].getInliers()),index=3)
  rtp.plotPoint(*zip(*ransac['R'].getInliers()),index=4)

  rtp.plotLine(*zip(*utils.pointsFromModel(ransac['L'].model)), index=0)
  rtp.plotLine(*zip(*utils.pointsFromModel(ransac['R'].model)), index=1)
  rtp.plotLine(*zip(*utils.pointsFromModel(z)), index=2)
  #rtp.plotLine(*zip(*utils.pointsFromModel(x)), index=3)

  #rtp.plotDash(*zip(*utils.pointsFromModel(xpre)), index=0)

  rtp.update()


  if count > 4500: break
  else: print 'count: {}'.format(count)
  #raw_input('press enter...')


  #s = raw_input('Type print to print otherwise press any keyboard...\nTo exit type exit...')
  #if s == 'print':
  #if count%50 == 0:

  #  utils.printLatex(
  #                    'cloud_'
  #                   ,str(count)
  #                   ,numpy.array(utils.fromThree2Two(ransac['L'].model))
  #                   ,numpy.array(utils.fromThree2Two(ransac['R'].model))
  #                   ,numpy.squeeze(numpy.asarray(x))
  #                   ,numpy.squeeze(numpy.asarray(z))
  #                  )

  #  utils.csv2(
  #              filename='cloud_'+str(count)
  #             ,fieldnames=['x','y']
  #             ,dataframe=data['L']+data['R']
  #            )

  #  utils.csv2(
  #              filename='cloudall_'+str(count)
  #             ,fieldnames=['x','y']
  #             ,dataframe=data['ALL']
  #            )

  #  utils.csv2(
  #              filename='cloudinliers_'+str(count)
  #             ,fieldnames=['x','y']
  #             ,dataframe=ransac['L'].getInliers()+ransac['R'].getInliers()
  #            )

  #if s == 'exit':
  #  raise Exception('Bye bye!!!')


rtp.close()

#utils.csv2( # To use in Vero kinematic simulator
#              filename='bisectrix'
#            , fieldnames=['t','ang_coeff','lin_coeff']
#            , dataframe=[(a,)+tuple(b) for  a,b in zip(Time,X)]
#           )