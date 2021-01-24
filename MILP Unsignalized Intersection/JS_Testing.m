clear all;clc
a = 5;
b = randi([1 4], 1, a);
c = ones(1, a);

for i = 2:a
    c(i) = c(i-1) + 10*rand();
end

tic
[x, delay] = solvemymilp(a, b, c);
toc