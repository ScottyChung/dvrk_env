clear all 
close all

M = csvread('joint_space.csv'); 
length(M)/1
data = reshape(M, [706, 14]);