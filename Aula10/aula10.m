%%
clear
close all
clc

openExample('nav/OptimalTrajectoryGenerationForUrbanDrivingExample')

%% Exercise 1

clear, close all
clc

WW = 150;
HH = 100;

P=[ 40 80 %point1
    85 45 %point2
    60 20 %point3
   ];

r = 4; %radius of obstacles

x = 1:WW;
y = 1:HH;

[X, Y] = meshgrid(x, y);

map = zeros(HH, WW);

for n = 1:size(P, 1)
    x0 = P(n, 1);
    y0 = P(n, 2);
    
    obstacle = ((X - x0).^2 + (Y - y0).^2 <= r^2);
    map = map + obstacle;
end

imshow(map)

krep = 100;
Dmax = 5;
maxPot = 20;

Urep = zeros(size(map));

for xv = x
    for yv = y
        % Check if is in a obstacle
        if map(xv, yv) == 1
            Urep(xv, yv) = maxPot;
            continue
        end
        
        % Search obstacles in the neighborhood and apply the equation
        mask = (Y - xv).^2 + (X - yv).^2 <= Dmax^2;
        obst = mask & map;

        [rows, cols] = find(obst == 1);

        for i = 1:size(rows)
            yobst = rows(i);
            xobst = cols(i);

            Urep(xv,yv) = Urep(xv,yv) + (1/2)*krep.*(1./norm([(xv-xobst) (yv-yobst)]) - 1/Dmax).^2;
        end

    end
end

surf(Urep)

Tx = 120;
Ty = 50;
katt = 0.005;

Uatt = 1/2 * katt * ((X-Tx).^2 + (Y-Ty).^2);
figure
surf(Uatt)
axis equal

U = Urep + Uatt;
figure
surf(U)
axis equal


[Gx, Gy] = gradient(U);
Gx = -Gx;
Gy = -Gy;

figure
quiver(x, y, Gx, Gy, 1)
axis equal

figure
contour(x, y, U, 60)
axis equal

startPoints = [15 10];

ll = stream2(X, Y, Gx, Gy, startPoints(:, 1), startPoints(:, 2));
h = streamline(ll);
h.Color = 'red';
h.LineWidth = 2;

B = ordfilter2(U, 1, ones(3,3));
mm = (B == U);
[rr, cc] = find(mm);
hold on
plot(cc, rr, 'ob')

%% Exercise 2

clear, close all
clc

