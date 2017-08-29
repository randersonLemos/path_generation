import os
import csv
import math
import numpy
import shutil
import pickle
import scipy.io
from classes import AppendFile

def polar2cartesian(radius, angle):
  x = radius*math.cos(angle)
  y = radius*math.sin(angle)
  return (x,y)


def pointsFromModel(model):
  pts = []
  if len(model) == 2:
    for x in range(-2,7):
      y = model[0].item()*x + model[1].item()
      pts.append((x,y))
  elif len(model) == 3:
    for x in range(-2,7):
      y = -(model[0]*x + model[2])/(model[1] + 1e-6)
      pts.append((x,y))
  return pts


def computeBisectrix(model1, model2):
  D1 = math.sqrt(model1[0]**2 + model1[1]**2)
  D2 = math.sqrt(model2[0]**2 + model2[1]**2)

  Ab1 = (D2*model1[0] - D1*model2[0])
  Bb1 = (D2*model1[1] - D1*model2[1]) + 1e-6
  Cb1 = (D2*model1[2] - D1*model2[2])

  Ab2 = (D2*model1[0] + D1*model2[0])
  Bb2 = (D2*model1[1] + D1*model2[1]) + 1e-6
  Cb2 = (D2*model1[2] + D1*model2[2])

  # Let's get the bisectrix that is closer to the vehicle
  dist1 = abs(Ab1*0+Bb1*0+Cb1)/numpy.sqrt(Ab1**2+Bb1**2)
  dist2 = abs(Ab2*0+Bb2*0+Cb2)/numpy.sqrt(Ab2**2+Bb2**2)

  return (Ab1,Bb1,Cb1) if dist1 <= dist2 else (Ab2,Bb2,Cb2)


def fromThree2Two(model):
  angCoeff = -float(model[0]/(model[1] + 1e-6))
  linCoeff = -float(model[2]/(model[1] + 1e-6))
  return (angCoeff,linCoeff)


def fromTwo2Three(model):
  a = -float(model[0])
  b = 1.0
  c = -float(model[1])
  return (a,b,c)


def savePoints(pts):
  with open('data.pickle','wb') as f:
    pickle.dump(pts,f)


def saveNpArr2Mat(arr):
  scipy.io.savemat('out.mat',{'arr': arr})


def csv2(dir_name, filename, fieldnames, dataframe):
  with open(dir_name+'/csv'+'/'+filename+'.csv', 'w') as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()
    for data in dataframe:
      writer.writerow(dict(zip(fieldnames,data)))


def saveSetParameters(save_param, directory, rtpdic, methoddic, hpdic, kalmandic):
  if save_param:
    shutil.rmtree(directory, ignore_errors=True)
    shutil.rmtree(directory+'_complete', ignore_errors=True) # deleting old folders
    os.makedirs(directory)
    os.makedirs(directory+"/csv")
    os.makedirs(directory+"/tikz") # making folders
    appendfile = AppendFile(directory)
    appendfile.write('Simulation:')
    appendfile.write(rtpdic)
    appendfile.write('Method:')
    appendfile.write(methoddic)
    appendfile.write('Handle points:')
    appendfile.write(hpdic)
    appendfile.write('Kalman filter:')
    appendfile.write(kalmandic)




def printLatex(dir_name, filename, idx, left_model, right_model, bissectrix, filtered):

  fhandle = open(dir_name+'/tikz'+'/'+filename+idx+'.tex', 'w')

  print >> fhandle, """%
%\\begin{{figure}}[!htb]
%\\centering
\\begin{{tikzpicture}}[trim axis left, trim axis right, scale=1.0]
\\begin{{axis}}[
                xmin=-4,xmax=6,
                ymin=-4,ymax=6,
                width=7.0cm, height=7.0cm,
                grid,% <-- changed
                grid style={{line width=.1pt, draw=white}},
                clip mode=individual,
                %xtick=\empty,
                %ytick=\empty,
                legend style={{at={{(0.99,0.98)}},anchor=north east}},
                %hide axis,
                %hide y axis
                every tick label/.append style={{font=\\scriptsize}},
                %reverse legend,
               ]
\\addplot[
          only marks,
          mark options={{scale=1.5, fill=white}},
          mark size=2pt,
          color=black,
          %opacity=0.8,
          fill opacity = 0.2,
         ]
         table [x={{x}}, y={{y}}, col sep=comma] {{{0}}};

\\addplot[
          only marks,
          mark options={{scale=1.5, fill=white}},
          mark size=2pt,
          color=blue,
          %opacity=0.8,
          fill opacity = 0.2,
         ]
         table [x={{x}}, y={{y}}, col sep=comma] {{{1}}};

\\addplot[
          only marks,
          mark options={{scale=1.5, fill=white}},
          mark size=2pt,
          color=green,
          %opacity=0.8,
          fill opacity = 0.2,
         ]
         table [x={{x}}, y={{y}}, col sep=comma] {{{2}}};

\\addplot[
          color=red,
          domain=-3:6,
          line width=0.5mm,
         ]
         {{
          {3}*x {4:+}
         }};

\\addplot[
          color=red,
          domain=-3:6,
          line width=0.5mm,
         ]
         {{
          {5}*x {6:+}
         }};

\\addplot[
          color=blue,
          domain=-3:6,
          line width=0.5mm,
         ]
         {{
          {7}*x {8:+}
         }};

%\\addplot[
%          dashed,
%          color=black,
%          domain=-3:6,
%          line width=0.25mm,
%         ]
%         {{
%          {9}*x {10:+}
%         }};

%\\legend{{
%         \\scriptsize{{outros pontos}}
%        ,\\scriptsize{{pontos de interesse}}
%        ,\\scriptsize{{pontos de ajustagem}}
%        ,
%        ,\\scriptsize{{retas suporte}}
%        ,\\scriptsize{{caminho de refer\\^{{e}}ncia}}
%       }}
\\end{{axis}}

\\coordinate (car) at (1.75,2.15);
\\begin{{scope}} [rotate around={{0:(car)}}]
\\draw[fill=yellow] (car) +  (-0.250, 0.100) rectangle + ( 0.250,-0.100);
\\draw[fill=black]  (car) ++ (-0.250, 0.100) rectangle + ( 0.175, 0.075); % rear left wheel
\\draw[fill=black]  (car) ++ ( 0.250, 0.100) rectangle + (-0.175, 0.075); % front left wheel
\\draw[fill=black]  (car) ++ ( 0.250,-0.100) rectangle + (-0.175,-0.075); % front right wheel
\\draw[fill=black]  (car) ++ (-0.250,-0.100) rectangle + ( 0.175,-0.075); % rear right wheel
\\end{{scope}}

\\end{{tikzpicture}}
%\\caption{{Foo}}
%\\label{{Foo}}
%\\end{{figure}}
%""".format(
             'chapters/cap5/csv/more/cloudall_'+idx+'.csv'     # 0
            ,'chapters/cap5/csv/more/cloud_'+idx+'.csv'
            ,'chapters/cap5/csv/more/cloudinliers_'+idx+'.csv' # 2
            ,left_model[0]
            ,left_model[1]                       # 4
            ,right_model[0]
            ,right_model[1]                      # 6
            ,filtered[0]
            ,filtered[1]                         # 8
            ,bissectrix[0]
            ,bissectrix[1]                        # 10
           )

  fhandle.close()
