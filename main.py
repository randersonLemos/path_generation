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

if len(sys.argv) == 1:
  warnings.warn("First, second and third parameters not specified." \
                " Using the default values \"False,  0.25 and 100\"...")
  save_param = False
  threshold_param = 0.25
  nTries_param = 100

elif len(sys.argv) == 4:
  if not sys.argv[1] in ('True', 'False'):
    raise Exception("First parameter must be True or False...")
  save_param = sys.argv[1] in ('True')

  try:
    threshold_param = float(sys.argv[2])
  except:
    raise Exception("Second parameter must be a number...")

  try:
    nTries_param = int(sys.argv[3])
  except:
    raise Exception("Third parameter must be a integer...")

else:
  raise Exception("Too many or too less paramters." \
                  " Four parameters must be specified in the following" \
                  " sequence \"save_param, threshold_param, ntries_param\"...")

numberIterations = 1400
directory = os.path.abspath(
                              'tmp'
                            + '/ransac_biased'
                            + '_'
                            + str(int(100*threshold_param))
                            + '_'+str(nTries_param)
                           )

rtpdic = {
           'save_images': save_param
          ,'nIterations': numberIterations
          ,'dir_name': directory
         }

methoddic = {
              'nTries': nTries_param
             ,'threshold': threshold_param
            }

hpdic = {
          'xmin': -1.0
         ,'xmax':  4.0
         ,'ymin': -2.5
         ,'ymax':  2.5
        }

kalmandic = {
              'A': numpy.matrix([[1.0, 0.0],[0.0, 1.0]])
             ,'Q': numpy.matrix([[1.0, 0.0],[0.0, 1.0]])
             ,'R': numpy.matrix([[150.0, 0.0],[0.0, 150.0]])
            }


utils.saveSetParameters(save_param, directory, rtpdic, methoddic, hpdic, kalmandic)

rtp = RealTimePlot(
                    num_point_type = 5
                   ,num_line_type = 4
                   ,num_dash_type = 1
                   ,save_images = rtpdic['save_images']
                   ,nIterations = rtpdic['nIterations']
                   ,dir_name = rtpdic['dir_name']
                  )

method = { 'name':'ransac'
          ,'L':Ransac(
                       nTries = methoddic['nTries']
                      ,threshold = methoddic['threshold']
                     )
          ,'R':Ransac(
                       nTries = methoddic['nTries']
                      ,threshold = methoddic['threshold']
                     )
         }

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

Z = []
X = []
Time = []

time_ = 1310046785
#time_ = 1310046820
bag = rosbag.Bag('bags/output.bag')
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

  XX = data['L'] + data['R']
  yy = len(data['L'])*[0] + len(data['R'])*[1]

  clf = svm.SVC(C=0.1,kernel='linear')
  clf.fit(XX,yy)

  w = clf.coef_[0]
  a = -w[0] / w[1]
  b = (clf.intercept_[0]) / w[1]

  method['L'].run(data['L'], 0)
  method['R'].run(data['R'], 1)

  model = utils.computeBisectrix(method['L'].model,method['R'].model)
  #z = numpy.matrix(utils.fromThree2Two(model)).T
  z = numpy.matrix([a,b]).T

  x, P = kalman.step(x,P,z)
  Z.append(numpy.squeeze(numpy.asarray(z)))
  X.append(numpy.squeeze(numpy.asarray(x)))

  # Visualization
  rtp.plotPoint(*zip(*data['ALL']),index=0)
  rtp.plotPoint(*zip(*data['L']),index=1)
  rtp.plotPoint(*zip(*data['R']),index=2)
  #rtp.plotPoint(*zip(*method['L'].getInliers()),index=3)
  #rtp.plotPoint(*zip(*method['R'].getInliers()),index=4)
  #rtp.plotLine(*zip(*utils.pointsFromModel(method['L'].model)), index=0)
  #rtp.plotLine(*zip(*utils.pointsFromModel(method['R'].model)), index=1)
  rtp.plotLine(*zip(*utils.pointsFromModel(z)), index=2)
  rtp.plotLine(*zip(*utils.pointsFromModel(x)), index=3)
  #rtp.plotDash(*zip(*utils.pointsFromModel(xpre)), index=0)
  rtp.update()

  if count > numberIterations:
    print "Simulation elapsed time: ", (method['L'].spenttime + method['R'].spenttime)/(2*numberIterations)
    if save_param:
      utils.csv2( # To use in Vero kinematic simulator
                   dir_name = directory
                  ,filename = 'bisectrix'
                  ,fieldnames = [ 'count','t'
                                 ,'ang_coeff','lin_coeff'
                                 ,'ang_coeff_est','lin_coeff_est'
                                ]
                  ,dataframe = [(a,)+(b,)+tuple(c)+tuple(d)
                     for a,b,c,d in zip(range(numberIterations),Time,Z,X)]
                 )

      appendfile.write('Average time:')
      appendfile.write((method['L'].spenttime + method['R'].spenttime)/(2*numberIterations))
      appendfile.write('Mean and Standard Deviation')
      appendfile.write(numpy.mean(zip(*X)[0]))
      appendfile.write(numpy.std(zip(*X)[0]))
      appendfile.close()

      rtp.close()

      tmp = os.path.abspath(directory)
      shutil.move(tmp, tmp + '_complete')
      time.sleep(1) # delays for 5 seconds
    break

  else:
    if count%100 == 0:
      print 'count: {}'.format(count)
    if save_param:
      if count%100 == 0:
        utils.printLatex(
                          dir_name = directory
                         ,filename = 'cloud_'
                         ,idx = str(count)
                         ,left_model = numpy.array(utils.fromThree2Two(method['L'].model))
                         ,right_model = numpy.array(utils.fromThree2Two(method['R'].model))
                         ,bissectrix = numpy.squeeze(numpy.asarray(z))
                         ,filtered = numpy.squeeze(numpy.asarray(x))
                        )

        utils.csv2(
                    dir_name = directory
                   ,filename = 'cloud_'+str(count)
                   ,fieldnames = ['x','y']
                   ,dataframe = data['L']+data['R']
                  )

        utils.csv2(
                    dir_name = directory
                   ,filename = 'cloudall_'+str(count)
                   ,fieldnames = ['x','y']
                   ,dataframe = data['ALL']
                  )

        utils.csv2(
                    dir_name = directory
                   ,filename = 'cloudinliers_'+str(count)
                   ,fieldnames = ['x','y']
                   ,dataframe = method['L'].getInliers()+method['R'].getInliers()
                  )
