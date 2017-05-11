clear all
close all
clc

load('out.mat')

xx = arr(:,1);
yy = arr(:,2);

%A = [xx,xx./xx];
%b = yy;
A = [1,1;2,1;3,1;4,1;5,1];
b = [1,2,3,4,5]';
[U,S,V] = svd(A,0);

%polyfit(xx,yy,1)
par= V*inv(S)*U'*b
