import sys
import math
import time
import scipy
from scipy import optimize
import random
import numpy as np
import matplotlib.pyplot as plt


def find_line_model(points):
    """ find a line model for the given points
    :param points selected points for model fitting
    :return line model
    """
 
    # [WARNING] vertical and horizontal lines should be treated differently
    #           here we just add some noise to avoid division by zero
 
    # find a line model for these points
    m = (points[1,1] - points[0,1]) / (points[1,0] - points[0,0] + sys.float_info.epsilon)  # slope (gradient) of the line
    c = points[1,1] - m * points[1,0]                                                       # y-intercept of the line
  
    return m, c 


def find_intercept_point(m, c, x0, y0):
    """ find an intercept point of the line model with
        a normal from point (x0,y0) to it
    :param m slope of the line model
    :param c y-intercept of the line model
    :param x0 point's x coordinate
    :param y0 point's y coordinate
    :return intercept point
    """
 
    # intersection point with the model
    x = (x0 + m*y0 - m*c)/(1 + m**2)
    y = (m*x0 + (m**2)*y0 - (m**2)*c)/(1 + m**2) + c
 
    return x, y


class LS:
  def __init__(self, xs, ys):
    self.xs = xs
    self.ys = ys
    self.m = None
    self.c = None
    self.time = None

  def search(self):
    start_time = time.time()
    A = np.vstack([self.xs, np.ones(len(self.xs))]).T
    self.m, self.c =  np.linalg.lstsq(A,self.ys)[0]
    self.time = time.time() - start_time


class TLS:
  def __init__(self, x, y):
    self.x = x
    self.y = y
    self.m = None
    self.c = None

  def search(self):
    length = len(self.x)
    xbar = np.mean(self.x)
    ybar = np.mean(self.y)
    c0 = 0.0; c1 = 0.0
    for i in range(length):
      c0 += (self.x[i] - xbar)*(self.y[i] - ybar)
      c1 += -(self.x[i] - xbar)**2+(self.y[i] - ybar)**2

    # line in normal vector representation
    phi = 0.5*np.arctan2(-2*c0,c1)
    r   = xbar*np.cos(phi) + ybar*np.sin(phi)

    # line in traditicional representation
    self.m = -np.cos(phi)/(np.sin(phi)+1e-6)
    self.c = r/(np.sin(phi)+1e-6)


class MLS:
  def __init__(self, xs, ys, sigma):
    self.samples = zip(xs, ys)
    self.sigma = sigma
    self.theta = np.zeros([0,0,0])
    self.m = None
    self.c = None
    self.time = None 

  def error(self, theta, x):
    return (theta[0]*x[0] + theta[1]*x[1] + theta[2])/(np.sqrt(theta[0]**2+theta[1]**2))
    #return -theta[0]/theta[1]*x[0] - theta[2]/theta[1] - x[1]

  def p(self, theta, x):
    #return 0.5*(self.error(theta,x)/self.sigma)**2
    return  np.log(1+ 0.5*(self.error(theta,x)/self.sigma)**2)

  def ML(self, theta):
    tmp = 0
    for sample in self.samples:
      tmp += self.p(theta, sample)
    return tmp

  def search(self, number_searches=1):
    start_time = time.time()
    coeffs = []
    costs   = []
    for i in range(number_searches):
      coeffs.append(scipy.optimize.fmin_bfgs(self.ML,np.random.rand(1,3), gtol=1e-3,disp=False))
      costs.append(self.ML(coeffs[-1]))
    index = [i for i, x in enumerate(costs) if costs[i]==min(costs)]
    self.theta = coeffs[index[0]]
    self.m = -self.theta[0]/self.theta[1]
    self.c = -self.theta[2]/self.theta[1]
    self.time = time.time() - start_time


class Ransac:
  def __init__(self, iterations, threshold, ratio, xs, ys, which):
    self.iterations = iterations
    self.threshold = threshold
    self.ratio = ratio
    self.samples = np.array(zip(xs,ys))
    self.which = which

    self.m = []
    self.c = []
    self.m_reg = None
    self.c_reg = None
    self.m_best = None
    self.c_best = None
    self.x_inliers = None
    self.y_inliers = None
    self.n_inliers = None
    self.r_inliers = None
    self.time = None
    self.n_its = None

  def search(self):
    start_time = time.time()
    n_samples = len(self.samples)

    ratio = 0.0
    model_m = 0.0
    model_c = 0.0
    for it in range(self.iterations):

      n = 2

      all_indices = np.arange(n_samples)
      np.random.shuffle(all_indices)

      indices_1 = all_indices[:n]
      indices_2 = all_indices[n:]

      maybe_points = self.samples[indices_1,:]
      test_points = self.samples[indices_2,:]


      # find a line model for these points
      m, c = find_line_model(maybe_points)

      x_list = []
      y_list = []
      for xx,yy in maybe_points:
        x_list.append(xx)
        y_list.append(yy)

      num = 0

      # find orthogonal lines to the model for all testing points
      for ind in range(test_points.shape[0]):

          x0 = test_points[ind,0]
          y0 = test_points[ind,1]

          # find an intercept point of the model with a normal from point (x0,y0)
          x1, y1 = find_intercept_point(m, c, x0, y0)

          # distance from point to the model
          dist = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)

          # check whether it's an inlier or not
          if dist < self.threshold:
              x_list.append(x0)
              y_list.append(y0)
              num += 1


      self.m.append(m)
      self.c.append(c)


      num += 2 # Let's not forget the two points from minimum set

      # in case a new model is better - cache it
      if num/float(n_samples) > ratio:
          ratio = num/float(n_samples)
          model_m = m
          model_c = c
          x_inliers = np.array(x_list)
          y_inliers = np.array(y_list)

      # we are done in case we have enough inliers
      if num > n_samples*self.ratio:
          break


    # Computing final value of m and c
    if self.which == 'LS':
      regressor = LS(x_inliers,y_inliers)
    elif self.which == 'TLS':
      regressor = TLS(x_inliers,y_inliers)
    elif self.which == 'MLS':
      regressor = MLS(x_inliers,y_inliers, 1.0)
    else:
      raise Exception('which {} does not match any option'.format(self.which))
    regressor.search()
    

    print '\nIterations: ',it + 1
    print 'Ration of inliers: ', ratio
    print 'Number of inliers: ', ratio*n_samples
    print ''

    self.m_reg = regressor.m
    self.c_reg = regressor.c
    self.m_best = model_m
    self.c_best = model_c
    self.x_inliers = x_inliers
    self.y_inliers = y_inliers
    self.n_inliers = len(x_inliers)
    self.r_inliers = ratio
    self.n_its = it + 1
    self.time = time.time() - start_time
