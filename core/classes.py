import os
import math
import numpy
import shutil
import ctypes
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl

class Ols(object):
  def __init__(self):
    self._INT   = ctypes.c_int
    self._PINT  = ctypes.POINTER(self._INT)
    self._PPINT = ctypes.POINTER(self._PINT)
    self._FLOAT   = ctypes.c_float
    self._PFLOAT  = ctypes.POINTER(self._FLOAT)
    self._PPFLOAT = ctypes.POINTER(self._PFLOAT)
    self._DOUBLE = ctypes.c_double
    self._PDOUBLE  = ctypes.POINTER(self._DOUBLE)
    self._PPDOUBLE = ctypes.POINTER(self._PDOUBLE)

    self.data = None
    self.n = None
    self.model = None

    self._libols = ctypes.CDLL('./cpp/ols/libols.so')
    self._ols_2Dline = getattr(self._libols, 'ols_2Dline')

  def _pythonlist2C2dArray(self, data):
    parr = (self._PDOUBLE * len(data))() # pointer array type
    for i in range(len(data)):
      parr[i] = (self._DOUBLE*2)()
      for j in range(2):
        parr[i][j] = data[i][j]
    return parr

  def _setVariables(self, data,):
    self.data = self._pythonlist2C2dArray(data)
    self.n = self._INT(len(data))
    self.model = (self._DOUBLE * 3)(0.0,0.0,0.0)

  def run(self, data, _):
    self._setVariables(data)
    self._ols_2Dline( self.data
                     ,self.n
                     ,self.model
                    )

  def getInliers(self):
    tmp = []
    for i in range(self.n.value):
     tmp.append((self.data[i][0],self.data[i][1]))
    return tmp


class Ransac(object):
  def __init__(self):
    self._INT   = ctypes.c_int
    self._PINT  = ctypes.POINTER(self._INT)
    self._PPINT = ctypes.POINTER(self._PINT)
    self._FLOAT   = ctypes.c_float
    self._PFLOAT  = ctypes.POINTER(self._FLOAT)
    self._PPFLOAT = ctypes.POINTER(self._PFLOAT)

    self.data = None
    self.n = None
    self.maxT = None
    self.threshold = None
    self.model = None
    self.inliers = None
    self.size = None
    self.side = None
    self.verbose = None

    self._libransac = ctypes.CDLL('./cpp/ransac/libransac.so')
    self._ransac_2Dline = getattr(self._libransac, 'ransac_2Dline')

  def _pythonlist2C2dArray(self, data):
    FLOATARR  = self._FLOAT * 2
    PFLOATARR = self._PFLOAT * len(data)
    ptr = PFLOATARR()
    for i in range(len(data)):
      ptr[i] = FLOATARR()
      for j in range(2):
        ptr[i][j] = data[i][j]
    return ptr

  def _setVariables(self, data, side):
    self.data = self._pythonlist2C2dArray(data)
    self.n = self._INT(len(data))
    self.maxT = self._INT(50)
    self.threshold = self._FLOAT(0.350)
    self.model = (self._FLOAT * 3)(0.0,0.0,0.0)
    self.inliers = self._INT(0)
    self.side = self._INT(side)
    self.verbose = self._INT(0)

  def run(self, data, side):
    self._setVariables(data, side)
    self._ransac_2Dline(  ctypes.byref(self.data)
                        , self.n
                        , self.maxT
                        , self.threshold
                        , ctypes.byref(self.model)
                        , ctypes.byref(self.inliers)
                        , self.side
                        , self.verbose
                       )

  def getInliers(self):
    tmp = []
    for i in range(self.inliers.value):
     tmp.append((self.data[i][0],self.data[i][1]))
    return tmp


class HandlePts(object):
  def bisectrixFrame(self, pt, model):
    th = math.atan(-model[0]/(model[1] + 1e-6))
    vecrot = self.systemRotation((0,model[2]/model[1]),th)
    ptrot = self.systemRotation(pt, th)
    return (vecrot[0] + ptrot[0], vecrot[1] + ptrot[1])

  def systemRotation(self, pt, angle):
    return (pt[0]*math.cos(angle)+pt[1]*math.sin(angle), -pt[0]*math.sin(angle) + pt[1]*math.cos(angle))

  def ditsPoint2Line(self, pt, model):
    return (model[0]*pt[0] + model[1]*pt[1] + model[2])/ \
               math.sqrt(model[0]**2 + model[1]**2)

  def selector(self, pt, model):
    key = False
    #ptb = self.bisectrixFrame(pt, model)
    ptb = pt
    if ptb[0] <= 5.0 and ptb[0] >= -1.0:
    #if ptb[0] <= 8.5 and ptb[0] >= 0.0:
      if ptb[1] >= 0.0 and ptb[1] <= 2.1:
      #if ptb[1] >= 0.0:
        key = 'L'
      elif ptb[1] <= 0.0 and ptb[1] >= -2.1:
      #elif ptb[1] <= 0.0:
        key = 'R'
    return key


class Kalman:
  def __init__(self, A, Q, R):
    self.A = A
    self.C = numpy.matrix([[1.0, 0.0],[0.0, 1.0]])
    self.Gamma = numpy.matrix([[1.0, 0.0],[0.0, 1.0]])
    self.Q = Q # model covariance matrix

    self.R = R # measure covariance matrix

  def step(self, x, P, z):
    # TIME UPDATE
    x = self.A*x
    P = self.A*P*self.A.T + self.Gamma*self.Q*self.Gamma.T

    # MEASUREMENT UPDATE
    S = self.C*P*self.C.T + self.R
    M = P*self.C.T*numpy.linalg.inv(S)
    x = x + M*(z - self.C*x)
    P = (numpy.identity(2)-M*self.C)*P
    return x, P


class RealTimePlot(object):
  def __init__(self, num_point_type, num_line_type, num_dash_type, save_images, nIterations, dir_name):
    self._num_point_type = num_point_type
    self._num_line_type = num_line_type
    self._num_dash_type = num_dash_type
    self._save_images = save_images

    self._nIterations = nIterations
    self._directory = dir_name 
    self._fig = plt.figure()
    self._count_fig = 0

    plt.ion()
    ax = plt.axes(xlim=(-4.0,8.0), ylim=(-6.0, 6.0))
    #ax.set_aspect('equal','datalim')

    point_plot = []
    for i in range(self._num_point_type):
      tmp, = ax.plot([-100],[0],'o',markersize=3)
      point_plot.append(tmp)

    line_plot = []
    for i in range(self._num_line_type):
      tmp, = ax.plot([-100],[0])
      line_plot.append(tmp)

    dash_plot = []
    for i in range(self._num_dash_type):
      tmp, = ax.plot([-100],[0],'--')
      dash_plot.append(tmp)

    self._point_enum = list(enumerate(point_plot))
    self._line_enum = list(enumerate(line_plot))
    self._dash_enum = list(enumerate(dash_plot))

  def plotPoint(self, xs, ys, index):
    for idx, plot in self._point_enum:
      if index == idx: plot.set_xdata(xs), plot.set_ydata(ys)

  def plotLine(self, xs, ys, index):
    for idx, plot in self._line_enum:
      if index == idx: plot.set_xdata(xs), plot.set_ydata(ys)

  def plotDash(self, xs, ys, index):
    for idx, plot in self._dash_enum:
      if index == idx: plot.set_xdata(xs), plot.set_ydata(ys)

  def update(self):
    if self._save_images:
      if not os.path.exists(self._directory):
        os.makedirs(self._directory)
      self._fig.savefig(self._directory+'/{:05}.png'.format(self._count_fig), bbox_inches='tight')
      self._count_fig += 1
      if self._count_fig > self._nIterations:
        tmp = os.path.abspath(self._directory)
        shutil.move(tmp, tmp + '_complete')
    else:
      plt.pause(1e-6)

  def close(self):
    plt.close()
