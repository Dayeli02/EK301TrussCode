Working Truss Code
load('TrussDesign5_RavenSonjaDayeli_A3.mat')
[J,M] = size(C);
R = zeros(J,J);
LTotalCount = 0;
% fill in x force of members
for i = 1:J
for j= 1:J
    R(i,j) = sqrt(power(X(i)-X(j),2)+power(Y(i)-Y(j),2));
end
end
A = zeros(2*J,M+3);
for i = 1:J
for j = 1:M
    if C(i,j) == 1
% find other place that C has a 1 in that column
        member_j = find (C(:,j) == 1 );
        for x = 1:2
            if member_j(x) ~= i
                otherrow = member_j(x);
            end
        end
% input data
        A(i,j)= (X(otherrow)-X(i))/R(otherrow,i);
        LTotalCount = LTotalCount + R(otherrow,i);
    else
        A(i,j) = 0;
    end
 end
end
% fill in y force of members
for i = 1:J
for j = 1:M
    if C(i,j) == 1
% find other place that C has a 1 in that column
        member_j = find (C(:,j) == 1 );
        for x = 1:2
            if member_j(x) ~= i
                otherrow = member_j(x);
            end
        end
% input data
        A(J+i,j)= (Y(otherrow)-Y(i))/R(otherrow,i);
    else
        A(J+i,j) = 0;
    end
 end
end
% fill in Sx force of members
for i = 1:J
for j = 1:3
    A(i,M+j) = Sx(i,j);
end
end
% fill in Sy force of members
for i = 1:J
for j = 1:3
    A(J+i,M+j) = Sy(i,j);
end
end
Ainv = pinv(A);
T = Ainv*L;
LTotal = LTotalCount/2;
failList = T;
failList(failList > 0 ) = 0;
failList = abs(failList);
Pcrit = zeros(1,M);
for j = 1:M
 member_j = find (C(:,j) == 1 );
 joint1 = member_j(1);
 joint2 = member_j(2);
 Pcrit(j) = 3654.533 * power(R(joint1,joint2),-2.119);
 failList(j) = failList(j)/(Pcrit(j));
end
fm = find(failList == max(failList));
TotalFailForce = sum(L)./failList;
failForce = abs(sum(L)/failList(fm));
% Group information
group_name = 'EK301, Section A3, Group 11';
members = {'Raven', 'Sonja','Dayeli'};
date = '11/12/2023';
% Reaction forces (example values)
reaction_labels = {'Sx1', 'Sy1', 'Sy2'};
cost = 10 * J + LTotal; % in dollars
% Display output
fprintf('EK301, Section A3, Group 11: RavenC, SonjaB, DayeliC, 11/17/23\n');
fprintf('Load: %d oz\n', sum(L));
% Display member forces
fprintf('Member forces in oz\n');
for i = 1:M
 fprintf('m%d: %.3f ', i, abs(T(i)));
if T(i) > 0
	fprintf('(T)\n');
elseif T(i) < 0
	fprintf('(C)\n');
elseif T(i) == 0
	fprintf('\n');
end
end
% Display reaction forces
fprintf('Reaction forces in oz\n');
for i = 1:3
fprintf('%s: %.2f\n', reaction_labels{i}, T(i+M));
end
fprintf('Cost of truss using formula C = 10J + Ltot: $%.2f\n', cost);
fprintf('Buckling Member: m%d\n', fm)
fprintf('Critical Load of Truss: %.2f oz\n', abs(failForce))
fprintf('Theoretical max load/cost ratio (in ounces/$): %.4f\n', abs(failForce)/cost);


Input file for PRACTICE TRUSS
%This is the input file for the PRACTICE TRUSS
%C(j,m) // j = number of joints // m = number of members
C = [1 1 0 0 0   0 0 0 0 0   0 0 0;%joint1
    1 0 1 1 0   0 0 0 0 0   0 0 0;%joint2
    0 0 1 0 1   0 0 0 1 0   0 0 0;%joint3
    0 1 0 1 1   1 1 0 0 0   0 0 0;%joint4
    0 0 0 0 0   1 0 1 1 1   0 0 0;%joint5
    0 0 0 0 0   0 1 1 0 0   1 1 0;%joint6
    0 0 0 0 0   0 0 0 0 1   1 0 1;%joint7
    0 0 0 0 0   0 0 0 0 0   0 1 1;];%joint8
%Sx(j,3) // cols = Sx1, Sy1, Sy2
Sx = [1 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0];
%Sy(j,3) // cols = Sx1, Sy1, Sy2
Sy = [0 1 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 1];
%length = j // gives x coord of each joint
X = [0 0 4 4 8 8 12 12];
%length = j // gives y coord of each joint
Y = [0 4 8 4 8 4 4 0];
%length = 2j // Load on each joint
%// first j elements == x-load on the joint
%// second j elements == y-load on each element
L = [0 0 0 0 0 0 0 0    0 0 0 25 0 0 0 0]';
save('TrussDesign4_RavenSonjaDayeli_A3.mat','C','Sx','Sy','X','Y','L')


Input file for design 1
%This is the input file for DESIGN NUMBER 1
%C(j,m) // j = number of joints // m = number of members
C = [1 0 0 0 1   0 0 0 0 0   0 0 0 0 0;%joint1
    1 1 0 0 0   1 0 1 0 0   0 0 0 0 0;%joint2
    0 1 1 0 0   0 0 0 1 0   1 0 0 0 0;%joint3
    0 0 1 1 0   0 0 0 0 0   0 1 0 1 0;%joint4
    0 0 0 1 0   0 0 0 0 0   0 0 0 0 1;%joint5
    0 0 0 0 1   1 1 0 0 0   0 0 0 0 0;%joint6
    0 0 0 0 0   0 1 1 1 1   0 0 0 0 0;%joint7
    0 0 0 0 0   0 0 0 0 1   1 1 1 0 0;%joint8
    0 0 0 0 0   0 0 0 0 0   0 0 1 1 1;];%joint9
%Sx(j,3) // cols = Sx1, Sy1, Sy2
Sx = [1 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0];
%Sy(j,3) // cols = Sx1, Sy1, Sy2
Sy = [0 1 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 1;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0];
%length = j // gives x coord of each joint
X = [0 8 16 24 33 4 12 20 28];
%length = j // gives y coord of each joint
h = 8*sind(60);
Y = [0 0 0 0 0 h h h h];
%length = 2j // Load on each joint
%// first j elements == x-load on the joint
%// second j elements == y-load on each element
L = [0 0 0 0 0 0 0 0 0     0 0 0 32 0 0 0 0 0]';
save('TrussDesign5_RavenSonjaDayeli_A3.mat','C','Sx','Sy','X','Y','L')


Input file for design 2

%This is the input file for DESIGN NUMBER 2
%C(j,m) // j = number of joints // m = number of members
%    1 2 3 4 5   6 7 8 9 10  1 2 3 4 5   6 7 8 9 10
C = [1 1 0 0 0   0 0 0 0 0   0 0 0 0 0   0 0 0 0 0;%joint1
    0 1 1 1 1   0 0 0 0 0   0 0 0 0 0   0 0 0 0 0;%joint2
    0 0 0 0 1   1 1 1 0 0   0 0 0 0 0   0 0 0 0 0;%joint3
    0 0 0 0 0   0 0 1 1 1   1 0 0 0 0   0 0 0 0 0;%joint4
    0 0 0 0 0   0 0 0 0 0   1 1 0 0 0   0 0 0 0 0;%joint5
    1 0 1 0 0   0 0 0 0 0   0 0 0 0 0   0 0 0 0 1;%joint6
    0 0 0 1 0   1 0 0 0 0   0 0 0 0 0   1 0 0 1 1;%joint7
    0 0 0 0 0   0 1 0 1 0   0 0 1 0 1   1 1 0 0 0;%joint8
    0 0 0 0 0   0 0 0 0 1   0 1 1 1 0   0 0 0 0 0;%joint9
    0 0 0 0 0   0 0 0 0 0   0 0 0 0 0   0 1 1 1 0;%joint10
    0 0 0 0 0   0 0 0 0 0   0 0 0 1 1   0 0 1 0 0;];%joint11
%Sx(j,3) // cols = Sx1, Sy1, Sy2
Sx = [1 0 0;%1
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;%6
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
   
    0 0 0;];
%Sy(j,3) // cols = Sx1, Sy1, Sy2
Sy = [0 1 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 1;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;];
%length = j // gives x coord of each joint
X = [0 8 16 24 33 4 12 20 28 17 26];
%length = j // gives y coord of each joint
Y = [0 2 2 0 0 10 12 11 8 20 17];
%length = 2j // Load on each joint
%// first j elements == x-load on the joint
%// second j elements == y-load on each element
L = [0 0 0 0 0 0 0 0 0 0 0     0 0 0 32 0 0 0 0 0 0 0]';
save('TrussDesign7_RavenSonjaDayeli_A3.mat','C','Sx','Sy','X','Y','L')





