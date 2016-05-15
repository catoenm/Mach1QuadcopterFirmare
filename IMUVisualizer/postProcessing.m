close all
clear all
clc

data = load('test5.txt');

figure
hold on
plot(data(:,1),'b')
plot(data(:,2),'r')
plot(data(:,3),'m')
