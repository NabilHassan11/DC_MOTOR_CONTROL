clc;
clear;

%DC Motor Model Parameters 
Kb = 39.3e-3;
R  = 7.94;
B  = 15e-6;
J  = 40e-7;
Kt = 0.0393;
L  = 0.154;

A = [-R/L -Kb/L 0; Kt/J -B/J 0; 0 1 0];
b = [1/L; 0 ; 0];
C = [1 0 0; 0 1 0; 0 0 1];
D = [0 ; 0; 0];
