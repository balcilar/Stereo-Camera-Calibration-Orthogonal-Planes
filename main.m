%% Written by Muhammet Balcilar, France, muhammetbalcilar@gmail.com
%  All rights reserved

clear all
clc
close all

IinL=imread('Inputs/left.jpg');

sigma=2;
peakThreshold=0.15;


[Ipoints Wpoints]=findpoints768686(IinL,sigma,peakThreshold);

figure;imshow(IinL);hold on;

for i=1:size(Ipoints)
    plot(Ipoints(i,1),Ipoints(i,2),'bo');
    text(Ipoints(i,1),Ipoints(i,2),[num2str(Wpoints(i,1)) ',' num2str(Wpoints(i,2)) ',' num2str(Wpoints(i,3))],'Color','r','FontSize',6)
end


Wpoints=Wpoints*28;

[K1 R1 T1]=ransac_calibration(Wpoints,Ipoints);
rot = rotationMatrixToVector(R1);
wp=Wpoints;
wp(:,4)=1;
rr=((K1*[R1 T1])*wp')';
RR=rr(:,1:2)./rr(:,3);
hold on;
plot(RR(:,1),RR(:,2),'r*')


IinR=imread('Inputs/right.jpg');

sigma=2;
peakThreshold=0.15;


[IpointsR Wpoints]=findpoints768686(IinR,sigma,peakThreshold);


figure;imshow(IinR);hold on;

for i=1:size(IpointsR)
    plot(IpointsR(i,1),IpointsR(i,2),'bo');
    text(IpointsR(i,1),IpointsR(i,2),[num2str(Wpoints(i,1)) ',' num2str(Wpoints(i,2)) ',' num2str(Wpoints(i,3))],'Color','r','FontSize',6)
end

Wpoints=Wpoints*28;

[K2 R2 T2]=ransac_calibration(Wpoints,IpointsR);
rot2 = rotationMatrixToVector(R2);
wp=Wpoints;
wp(:,4)=1;
rr=((K2*[R2 T2])*wp')';
RR=rr(:,1:2)./rr(:,3);
hold on;
plot(RR(:,1),RR(:,2),'r*')

figure;hold on;
axis vis3d; axis image;
grid on;
plot3(Wpoints(:,1),Wpoints(:,2),Wpoints(:,3),'b.','MarkerSize',20)

Lcam=K1*[R1 T1];
Rcam=K2*[R2 T2];
[points3d] = mytriangulate(Ipoints, IpointsR, Lcam,Rcam );
plot3(points3d(:,1),points3d(:,2),points3d(:,3),'r.')

% referencePoint(0,0,0)= R*Camera+T, So Camera=-inv(R)*T;
CL=-R1'*T1;
CR=-R2'*T2;

plot3(CR(1),CR(2),CR(3),'gs','MarkerFaceColor','g');
plot3(CL(1),CL(2),CL(3),'cs','MarkerFaceColor','c');
legend({'ground truth point locations','Calculated point locations','Right Camera position','Left Camera Position'});

% relative transformation from left one to right camera 
R=R2*R1';
T=T2-R*T1;

%Essential and Fundamental Matrix
tx = [0 -T(3) T(2); T(3) 0 -T(1); -T(2) T(1) 0];
E = tx * R;
F = K2' \ E /  K1;


% find epipolar line of (218,398) coorinate in left image
lf=F*[218;398;1];


% show that line on to right image
spx=[1 size(IinR,2)];
spy(1,1)=(-lf(3)-lf(1)*spx(1))/lf(2);
spy(1,2)=(-lf(3)-lf(1)*spx(2))/lf(2);
figure;imagesc(IinR);hold on;
plot(spx,spy,'b-','linewidth',4);


% find the world coordinate of (218,398) pixel on left image which is on
% y=0 plane
% ip=K1*[R1 T1]*w 
H=K1*[R1 T1];
w=inv([H(:,1) H(:,3) -[218;398;1]])*-H(:,4);

W(1,1)=w(1);
W(2,1)=0;
W(3,1)=w(2);


% find that world point in right image coordinate
q=K2*[R2 T2]*[W;1]
ip=q/q(3);
plot(ip(1),ip(2),'r*','MarkerSize',10);  

