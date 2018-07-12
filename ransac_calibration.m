function [K R T]=ransac_calibration(Wpoints,Ipoints)

%% written by Muhammet Balcilar,France
%% muhammetbalcilar@gmail.com


% these are ransac parameters
% it refers the threshold value to determine which instance in inlier or outlier.
% it means if the dstance is bigger than this threshold we get it into account
% as outlier unless inlier
threshDist=2;
% it defines how many iteration we need to keep continue. normally given code gives
% us infinitive loop as define while 1, but in pratically we should keep
% loop with in certain number of loop
maxIter=5000;
% this define how many instance is inlier, we define as 0 for initial value
bestInNum=0;





% we set 2d and 3d points locations
Points3D= Wpoints;%nccworldPt(2:end,:);
Points2D=Ipoints; %nccimagePt(2:end,:);

nbr=size(Wpoints,1);

% initial iteration is 0
iter=0;

% the main loop of ransac
while iter<maxIter
    % increase iteation number step by step
    iter=iter+1;
    % select 6 random sample in the points set
    ns=20;
    order=randperm(nbr,ns);
    % set all zero for Q and B matrix
    Q=zeros(ns*2,11);
    B=zeros(ns*2,1);
    % loop for construct Q and B matrix
    for k=1:length(order)
        % get selected instance order
        kk=order(k);
        % set consecutive 2 rows of Q matrix as defined 
        Q(2*k-1,:)=[Points3D(kk,1),Points3D(kk,2),Points3D(kk,3),1,0,0,0,0,Points2D(kk,1)*(-Points3D(kk,1)),Points2D(kk,1)*(-Points3D(kk,2)),Points2D(kk,1)*(-Points3D(kk,3))];
        Q(2*k,:)=[0,0,0,0,Points3D(kk,1),Points3D(kk,2),Points3D(kk,3),1,Points2D(kk,2)*(-Points3D(kk,1)),Points2D(kk,2)*(-Points3D(kk,2)),Points2D(kk,2)*(-Points3D(kk,3))];
        % set consecutive 2 rows of B matrix as defined         
        B(2*k-1,:)=[Points2D(kk,1)];        
        B(2*k,:)=[Points2D(kk,2)];        
    end
    % Estimate the camera matrix from given a 6 number of selected 3D-2D correspondences
    
    % right now we need to find 11 element A vector which should supply 
    % Q*A=B equation. To solve this equation we should use pseudo inverse
    % of Q let we say pseudo inverse of Q as pQ,(pQ=(((Q'*Q)^(-1))*Q')) then the solution is A=pQ*B
    A=(((Q'*Q)^(-1))*Q')*B;
    aa=A;
    % we reshape 11 element A vector as 3*4 matrix which last element is 1
    A=[A(1),A(2),A(3),A(4);A(5),A(6),A(7),A(8);A(9),A(10),A(11),1];
    % construct all 4 column martix with 3d points which last column is 1 
    trt=[Points3D ones(nbr,1)];
    % calculate 2d location of given 3d points 
    trt2=A * trt';    
    trt2 = trt2(1:2,:)./trt2(3,:);
    trt2=trt2';
    % calculate differences of calculated 2d points and given 2d points
    differ=abs(trt2-Points2D);
    % caluclate euclidien distance between calculated points location and
    % given point location
    distance=sqrt(differ(:,1).^2+differ(:,2).^2);
    
    % Compute how many points is inside the threshold with calculated
    % model A        
    inlierNum = length(find(abs(distance)<=threshDist));
    
    % Update the number of inliers and fitting model if better model is found     
     if inlierNum>bestInNum
         bestInNum = inlierNum;
         % keep found A as the best model 
         bestA=A;   
         besta=aa;
     end
end

% after itertions we load the best camera matrix  A, which we found above
A=bestA;
% Decompose the camera matrix, extract intrinsic and extrinsic camera parameters 
T1=A(1,1:3)./A(3,4);
T2=A(3,1:3)./A(3,4);
T3=A(2,1:3)./A(3,4);
C1=A(1,4)./A(3,4);
C2=A(2,4)./A(3,4);

U0=dot(T1,(T2'))./(norm(T2)^2);
V0=dot(T2,T3')./(norm(T2)^2);
AlphaU=norm(cross((T1)',(T2)'))./(norm(T2)^2);
AlphaV=norm(cross((T2)',(T3)'))./(norm(T2)^2);


tz=1./norm(T2);

% the calibration matrix can be defined as follows.
% Intrinsic parameter K matrix should be define as follows
%K=[AlphaU 0 U0;0 AlphaV V0;0 0 1];
K=[AlphaU 0 800;0 AlphaV 600;0 0 1];

% Extrinsic parameters are T and R
RT=inv(K)*bestA*tz;
T=RT(:,4);
R=RT(:,1:3);








