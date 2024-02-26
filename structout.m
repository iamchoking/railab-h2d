function struct = structout(symbols,values)
%STRUCTOUT Summary of this function goes here
%   Detailed explanation goes here
if nargin < 1
    symbols = {};
    values  = {};
end
syms a b
aplusb = a+b;
vecab = [a;b];
cosa  = cos(a);
struct.a = a;
struct.b = b;
disp("symbols: ")
disp(symbols)
disp("values: ")
disp(values)
struct.aplusb = subs(aplusb,symbols,values);
struct.cosa = cosa;
struct.vecab = vecab;
end

