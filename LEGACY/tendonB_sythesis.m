gamma = cos(deg2rad(45));
max_range = 0.040;

sympref('FloatingPointOutput',true)

syms r_1m r_1p r_1d r_2m

sol = solve(r_1m == 0.005,(r_2m+2*r_1m)/110 == gamma*(r_1p*2)/60, (r_1p*2)/60 == gamma*(r_1d*2)/25, r_1m*deg2rad(90)+r_1p*deg2rad(110)+r_1d*deg2rad(90) == max_range)