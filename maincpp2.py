import copy
import rospy
import numpy
import rosbag
import core.utils as utils
from core.classes import Ransac, HandlePts, Kalman, RealTimePlot


if __name__ == '__main__':

  rtp = RealTimePlot(
                       num_point_type=3
                     , num_line_type=4
                     , num_dash_type=1
      )

  ransac = {'L':Ransac(), 'R':Ransac()}

  hp = HandlePts()

  kalman = {'L': Kalman(1.0,50.0),'R': Kalman(1.0,50.0) ,'B': Kalman(1.0,50.0)}

  x    = {'L': numpy.matrix([[0.0],[0.0]]), 'R': numpy.matrix([[0.0],[0.0]]), 'B': numpy.matrix([[0.0],[0.0]])}
  z    = {'L': numpy.matrix([[0.0],[0.0]]), 'R': numpy.matrix([[0.0],[0.0]]), 'B': numpy.matrix([[0.0],[0.0]])}

  P = {'L': numpy.matrix([[1e4,0],[0,1e4]]), 'R': numpy.matrix([[1e4,0],[0,1e4]]), 'B': numpy.matrix([[1e4,0],[0,1e4]])}

  countPrints = 0
  fhandle = open('out.txt', 'w')


  bag = rosbag.Bag('../bags/output.bag')
  for topic, msg, t in bag.read_messages(start_time=rospy.rostime.Time(1310046785)):
#  for topic, msg, t in bag.read_messages(start_time=rospy.rostime.Time(1310046785571381000)):
    angle = msg.angle_min
    data = {'ALL':[], 'L':[], 'R':[]}
    for radius in msg.ranges:
      if radius > msg.range_min and radius < msg.range_max:
        pt = utils.polar2cartesian(radius, angle)
        pt = (1.5+pt[0],pt[1]) # shifting points

        if pt[0] >= 0: data['ALL'].append(pt) # not part of the implementation

        key = hp.selector(pt,utils.fromTwo2Three(x['B']))
        if key:
          data[key].append(pt)

        angle += msg.angle_increment

    ransac['L'].run(data['L'])
    ransac['R'].run(data['R'])

    z['L'] = numpy.matrix(utils.fromThree2Two(ransac['L'].model)).T
    x['L'], P['L'] = kalman['L'].step(x['L'],P['L'],z['L'])

    z['R'] = numpy.matrix(utils.fromThree2Two(ransac['R'].model)).T
    x['R'], P['R'] = kalman['R'].step(x['R'],P['R'],z['R'])

    model = utils.computeBisectrix(utils.fromTwo2Three(x['L']),utils.fromTwo2Three(x['R']))

    z['B'] = numpy.matrix(utils.fromThree2Two(model)).T
    x['B'], P['B'] = kalman['B'].step(x['B'],P['B'],z['B'])

    # Visualization
    rtp.plotPoint(*zip(*data['ALL']),index=0)

    rtp.plotPoint(*zip(*data['L']),index=1)
    rtp.plotPoint(*zip(*data['R']),index=2)

    rtp.plotLine(*zip(*utils.pointsFromModel(x['L'])), index=0)
    rtp.plotLine(*zip(*utils.pointsFromModel(x['R'])), index=1)
#    rtp.plotLine(*zip(*utils.pointsFromModel(z)), index=2)
    rtp.plotLine(*zip(*utils.pointsFromModel(x['B'])), index=3)

#    rtp.plotDash(*zip(*utils.pointsFromModel(xpre)), index=0)
    rtp.plotDash(*zip(*utils.pointsFromModel(z['B'])), index=0)


#    s = raw_input('type print to print otherwise press any keyboard...')
#    if s == 'print':
#      countPrints += 1
#      print countPrints
#
#      utils.printLatex('cloud_',str(countPrints), LLmodel, RRmodel, x, z)
#      utils.csvLatex('cloud_'+str(countPrints),t,data['L']+data['R'])
#      utils.csvLatex('cloudall_'+str(countPrints),t,data_all)
#
#    if s == 'close':
#      fhandle.close()
#      raise Exception('Ja chega!!!')
