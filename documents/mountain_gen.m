clear;

A = csvread("md-121016-vb1-m1-6bunches-10acc-1341-1343.csv");

xlabel("samples");
ylabel("amplitude");
zlabel("time");

mesh(A);

pause;
