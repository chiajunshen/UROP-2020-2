function [x, delay] = solvemymilp(vnum, vlane, vtime)
tic
f = zeros(vnum+1, 1);
f(1) = 1;
intcon = zeros(1, vnum+1);
lb = zeros(vnum+1, 1);
ub = zeros(vnum+1, 1);
ub(:) = Inf;

Amat = zeros(vnum*2, vnum+1);
bmat = zeros(vnum*2, 1);
Amat(1:vnum, 1) = -1;
varj = 2;
var2 = vnum*2 + 1;

varloop = 2;

for loopi = 1:vnum
   Amat(loopi, varj) = 1;
   Amat(loopi+vnum, varj) = -1;
   bmat(loopi+vnum) = vtime(loopi)*-1;
   varj = varj+1;
end

for loopi = 1:vnum-1
    for loopj = varloop:vnum
        if vlane(loopj) == vlane(loopi)
           Amat(var2, loopi+1) = 1;
           Amat(var2, loopj+1) = -1;
           bmat(var2) = -1;
           var2 = var2+1;
           
        elseif mod(vlane(loopj),2) ~= mod(vlane(loopi),2)
            Amat(var2, loopi+1) = 1;
            Amat(var2, loopj+1) = -1;
            bmat(var2) = -8;
            var2 = var2+1;
        end
    end
    varloop = varloop + 1;
end

for loop = 1:vnum+1
    intcon(loop) = loop;
end
x = intlinprog(f, intcon, Amat, bmat, [], [], lb, ub);
%sol = x(2:end);
delay = toc;
end