function [coeff, A, b] = getCoeff(waypoints)

n = size(waypoints, 1)-1; %n = size(waypoints, 2)-1; % number of segments P1..n
A = zeros(8*n, 8*n);
b = zeros(1, 8*n); %b = zeros(3, 8*n);

for i = 1:n
    b(1, i) = waypoints(i, 1); 
    b(1, i + n) = waypoints(i+1, 1);
end

row = 1;

% Constraint 1) Pi(0) = Wi for all i = 1..n

for i = 1:n
   A(row, (i-1)*8+1: i*8) = polyT(8, 0, 0);
   row = row + 1;
end

% Constraint 2) Pi(1) = Wi + 1 for all i = 1..n

for i = 1:n
   A(row, (i-1)*8+1: i*8) = polyT(8, 0, 1);
   row = row + 1;
end

% Constraint 3) P1(k)(0) = 0 for all 1 <= k <= 3

for k = 1:3
    A(row, (1-1)*8+1: 1*8) = polyT(8, k, 0);
    row = row + 1;
end

% Constraint 4) Pn(k)(1) = 0 for all 1 <= k <= 3

for k = 1:3
   A(row, (n-1)*8+1: n*8) = polyT(8, k, 1);
   row = row + 1;
end

% Constraint 5) Pi-1(k)(1) = Pi(k)(0) for all i = 2..n and for all k =1..6

for i = 2:n
   for k = 1:6
       A(row, (i-2)*8+1: i*8) = [polyT(8, k, 1) -polyT(8, k, 0)];
       row = row + 1;
   end
end

coeff = A\b'; 

end