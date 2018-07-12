%% written by Muhammet Balcilar,France
%% muhammetbalcilar@gmail.com

clear all
clc
close all

Iin=imread('Inputs/left.jpg');

sigma=2;
peakThreshold=0.15;


[Ipoints Wpoints]=findpoints768686(Iin,sigma,peakThreshold);
figure;imshow(Iin);hold on;

for i=1:size(Ipoints)
    plot(Ipoints(i,1),Ipoints(i,2),'bo');
    text(Ipoints(i,1),Ipoints(i,2),[num2str(Wpoints(i,1)) ',' num2str(Wpoints(i,2)) ',' num2str(Wpoints(i,3))],'Color','r','FontSize',6)
end


Wpoints=Wpoints*2.8;

[K R T]=ransac_calibration(Wpoints,Ipoints);
rot = rotationMatrixToVector(R);
wp=Wpoints;
wp(:,4)=1;
rr=((K*[R T])*wp')';
RR=rr(:,1:2)./rr(:,3);
hold on;
plot(RR(:,1),RR(:,2),'r*')


Iin=imread('Inputs/right.jpg');

sigma=2;
peakThreshold=0.15;


[IpointsR Wpoints]=findpoints768686(Iin,sigma,peakThreshold);


figure;imshow(Iin);hold on;

for i=1:size(IpointsR)
    plot(IpointsR(i,1),IpointsR(i,2),'bo');
    text(IpointsR(i,1),IpointsR(i,2),[num2str(Wpoints(i,1)) ',' num2str(Wpoints(i,2)) ',' num2str(Wpoints(i,3))],'Color','r','FontSize',6)
end

Wpoints=Wpoints*2.8;

[K2 R2 T2]=ransac_calibration(Wpoints,IpointsR);
rot2 = rotationMatrixToVector(R2);
wp=Wpoints;
wp(:,4)=1;
rr=((K2*[R2 T2])*wp')';
RR=rr(:,1:2)./rr(:,3);
hold on;
plot(RR(:,1),RR(:,2),'r*')

figure;hold on;
plot3(Wpoints(:,1),Wpoints(:,2),Wpoints(:,3),'b.')

Rcam=K*[R T];
Lcam=K2*[R2 T2];
[points3d] = mytriangulate(Ipoints, IpointsR, Rcam,Lcam );
plot3(points3d(:,1),points3d(:,2),points3d(:,3),'r.')

CR=-inv(R)*T;
CL=-inv(R2)*T2;
plot3(CR(1),CR(2),CR(3),'gs');
plot3(CL(1),CL(2),CL(3),'cs');
axis equal
legend({'ground truth point locations','Calculated point locations','Right Camera position','Left Camera Position'});










