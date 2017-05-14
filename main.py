import sys
import time
import copy
import rospy
import numpy
import rosbag
import warnings
import core.utils as utils
from core.classes import Ransac, HandlePts, Kalman, RealTimePlot, Ols

if len(sys.argv) == 2:
  if not sys.argv[1] in ('True', 'False'):
    raise Exception("The parameter save is or True or False...")
  save = sys.argv[1] in ('True')
elif len(sys.argv) == 1:
  warnings.warn("Boolean parameter save not specified.\nUsing the default value, which is False...")
  save = False
else:
  raise Exception(" Too many parameters...")

NumberIterations = 50

rtp = RealTimePlot(
                    num_point_type=5
                   ,num_line_type=4
                   ,num_dash_type=1
                   ,save_images=save
                   ,nIterations=NumberIterations
                   ,dir_name= 'png/' + time.strftime("%Y_%m_%d_%H_%M_%S")
                  )

#method = { 'name':'ols'
#          ,'L':Ols()
#          ,'R':Ols()
#         }

method = { 'name':'ransac'
          ,'L':Ransac()
          ,'R':Ransac()
         }

hp = HandlePts()

kalman = Kalman(
                 A = numpy.matrix([[0.975, 0.0],[0.0, 0.985]])
                ,Q = numpy.matrix([[1.0, 0.0],[0.0, 1.0]]) # model covariance matrix
                ,R = numpy.matrix([[200.0, 0.0],[0.0, 200.0]]) # measure covariance matrix
               )

x = numpy.matrix([[0.0],[0.0]])
z = numpy.matrix([[0.0],[0.0]])
P = numpy.matrix([[1e4,0],[0,1e4]])

X = []
Time = []
EstimationTotalTime = 0.0


time_ = 1310046785
#time_ = 1310046820
bag = rosbag.Bag('../bags/output.bag')
for count, (topic, msg, t) in enumerate(bag.read_messages(start_time=rospy.rostime.Time(time_))):
  Time.append(t.to_time())


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


  current_time = time.time()
  method['L'].run(data['L'], 0)
  method['R'].run(data['R'], 1)
  EstimationTotalTime += time.time() - current_time

  model = utils.computeBisectrix(method['L'].model,method['R'].model)
  z = numpy.matrix(utils.fromThree2Two(model)).T
  x, P = kalman.step(x,P,z)
  X.append(numpy.squeeze(numpy.asarray(x)))


  # Visualization
  #rtp.plotPoint(*zip(*data['ALL']),index=0)
  rtp.plotPoint(*zip(*data['L']),index=1)
  rtp.plotPoint(*zip(*data['R']),index=2)
  rtp.plotPoint(*zip(*method['L'].getInliers()),index=3)
  rtp.plotPoint(*zip(*method['R'].getInliers()),index=4)

  rtp.plotLine(*zip(*utils.pointsFromModel(method['L'].model)), index=0)
  rtp.plotLine(*zip(*utils.pointsFromModel(method['R'].model)), index=1)
  rtp.plotLine(*zip(*utils.pointsFromModel(z)), index=2)
  rtp.plotLine(*zip(*utils.pointsFromModel(x)), index=3)

  #rtp.plotDash(*zip(*utils.pointsFromModel(xpre)), index=0)

  rtp.update()


  if count > NumberIterations:
    #utils.csv2( # To use in Vero kinematic simulator
    #              filename='bisectrix'
    #            , fieldnames=['t','ang_coeff','lin_coeff']
    #            , dataframe=[(a,)+tuple(b) for  a,b in zip(Time,X)]
    #           )
    print "Simulation elapsed time: ", EstimationTotalTime/(NumberIterations)
    rtp.close()
    break

  else: print 'count: {}'.format(count)
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
