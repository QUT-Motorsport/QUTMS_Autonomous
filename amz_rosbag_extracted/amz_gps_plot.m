Array=readtable('amz_gps.csv');
lat = table2array(Array(:, 7));
long = table2array(Array(:, 8));

figure()
plot(lat, long)