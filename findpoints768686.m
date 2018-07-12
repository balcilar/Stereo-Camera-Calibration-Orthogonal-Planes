
function [Ipoints Wpoints]=findpoints768686(Iin,sg,pk)

%% written by Muhammet Balcilar,France
%% muhammetbalcilar@gmail.com

bn=1;
Igray = rgb2gray(Iin);
I = im2single(Igray);

sigma=sg;%2;
peakThreshold=pk;%0.15;

[cxy, c45, Ix, Iy, Ixy, I_45_45] = vision.internal.calibration.checkerboard.secondDerivCornerMetric(I, sigma);
[Ix2, Iy2, IxIy] = computeJacobianEntries(Ix, Iy);

points0 = vision.internal.calibration.checkerboard.find_peaks(cxy, peakThreshold);
scores0 = cxy(sub2ind(size(cxy), points0(:, 2), points0(:, 1)));
board0 = growCheckerboard(points0, scores0, Ix2, Iy2, IxIy, 0);
board0.type=0;
while board0.isValid==1
    [m n]=size(board0.BoardIdx);    
    
    if (m==8 & n==6)| (m==6 & n==8) | (m==7 & n==6)| (m==6 & n==7)
        board{bn}=board0;
        bn=bn+1;
    end
    ind=board0.BoardIdx(:);
    points0(ind,:)=[];
    scores0(ind,:)=[];
    board0 = growCheckerboard(points0, scores0, Ix2, Iy2, IxIy, 0);
    board0.type=0;
end

points45 = vision.internal.calibration.checkerboard.find_peaks(c45, peakThreshold);
scores45 = c45(sub2ind(size(c45), points45(:, 2), points45(:, 1)));
board45 = growCheckerboard(points45, scores45, Ix2, Iy2, IxIy, pi/4);
board45.type=45;
while board45.isValid==1
    [m n]=size(board45.BoardIdx);    
    if (m==8 & n==6)| (m==6 & n==8) | (m==7 & n==6)| (m==6 & n==7)
        board{bn}=board45;
        bn=bn+1;
    end
    
    ind=board45.BoardIdx(:);
    points45(ind,:)=[];
    scores45(ind,:)=[];
    board45 = growCheckerboard(points45, scores45, Ix2, Iy2, IxIy, 0);
    board45.type=45;
end
E=[];
for i=1:length(board)
    brd=board{i};
    cx=brd.BoardCoords(:,:,1);
    cy=brd.BoardCoords(:,:,2);
    E(i,1)=board{i}.Energy;
    E(i,2)=i;
    E(i,3:6)=[mean(cx(:)) mean(cy(:)) size(brd.BoardIdx)];
end
if size(E,1)<3
    return
end

E=sortrows(E,1);
II=ones(size(E,1),1);
for i=1:size(E,1)-1
    for j=i+1:size(E,1)
        if norm(E(i,3:4)-E(j,3:4))<10
            II(j)=0;
        end
    end
end
ind=fix(E(II==1,2));
E=E(ind,:);

if size(E,1)~=3
    return
end

II=find(E(:,5)~=7 & E(:,6)~=7);

if length(II)~=2
    return
end

if E(II(1),3)<E(II(2),3)
    B=E(II(1),2);
    C=E(II(2),2);
else
    B=E(II(2),2);
    C=E(II(1),2);
end
II=find(E(:,5)==7 | E(:,6)==7);
if length(II)~=1
    return
end

A=E(II(1),2);

a=board{A}.type;
A=board{A}.BoardCoords;
b=board{B}.type;
B=board{B}.BoardCoords;
c=board{C}.type;
C=board{C}.BoardCoords;

ac=[A(1,1,1) A(1,1,2);A(end,1,1) A(end,1,2);A(1,end,1) A(1,end,2);A(end,end,1) A(end,end,2)];
bc=[B(1,1,1) B(1,1,2);B(end,1,1) B(end,1,2);B(1,end,1) B(1,end,2);B(end,end,1) B(end,end,2)];
cc=[C(1,1,1) C(1,1,2);C(end,1,1) C(end,1,2);C(1,end,1) C(1,end,2);C(end,end,1) C(end,end,2)];
D=[];
for i=1:4
    for j=1:4
        for k=1:4
            D=[D;norm(ac(i,:)-bc(j,:))+norm(ac(i,:)-cc(k,:))+norm(bc(j,:)-cc(k,:)) i j k];
        end
    end
end
[mn im]=min(D(:,1));
trn=D(im,2:end);
A=turn(A,trn(1));
B=turn(B,trn(2));
C=turn(C,trn(3));

A=superres(A,a,Ixy,I_45_45);
B=superres(B,b,Ixy,I_45_45);
C=superres(C,c,Ixy,I_45_45);

if size(A,1)>size(A,2)
    [aY aX]=meshgrid(1:size(A,2),1:size(A,1));
else
    [aX aY]=meshgrid(1:size(A,2),1:size(A,1));
end
aZ=zeros(size(aX));

if size(B,1)>size(B,2)
    [bZ bX]=meshgrid(1:size(B,2),1:size(B,1));
else
    [bX bZ]=meshgrid(1:size(B,2),1:size(B,1));
end
bY=zeros(size(bX));

if size(C,1)>size(C,2)
    [cY cZ]=meshgrid(1:size(C,2),1:size(C,1));
else
    [cZ cY]=meshgrid(1:size(C,2),1:size(C,1));
end
cX=zeros(size(cY));

Ipoints=[];
x = A(:, :, 1);
y = A(:, :, 2);
Ipoints=[Ipoints;x(:) y(:)];  
x = B(:, :, 1);
y = B(:, :, 2);
Ipoints=[Ipoints;x(:) y(:)];
x = C(:, :, 1);
y = C(:, :, 2);
Ipoints=[Ipoints;x(:) y(:)];

Wpoints=[];
x = aX;
y = aY;
z=aZ;
Wpoints=[Wpoints;x(:) y(:) z(:)];  
x = bX;
y = bY;
z=bZ;
Wpoints=[Wpoints;x(:) y(:) z(:)];
x = cX;
y = cY;
z=cZ;
Wpoints=[Wpoints;x(:) y(:) z(:)];

% 
% imagesc(I);
% hold on;
% clr={'r*','g*','k*','c*','m*','y*'};
% for i=1:length(board)
%     
%     tmpx=board{i}.BoardCoords(:,:,1);
%     tmpy=board{i}.BoardCoords(:,:,2);
%     hold on;plot(tmpx(:),tmpy(:),clr{i})
%     
%     [m n]=size(board{i}.BoardIdx);
%     
%     %     for j=1:size(tmpx,1)
%     %         for k=1:size(tmpx,2)
%     %
%     %             text(tmpx(j,k),tmpy(j,k),[num2str(j),',',num2str(k)]);
%     %         end
%     %     end
%     pause;
% end



end

function A=superres(A,a,Ixy,I_45_45)
[m n k]=size(A);
x = A(:, :, 1);
y = A(:, :, 2);
points=[x(:) y(:)];    
if a==0    
    Apoints = vision.internal.calibration.checkerboard.subPixelLocation(Ixy, points);
else    
    Apoints = vision.internal.calibration.checkerboard.subPixelLocation(I_45_45, points);
end
A(:,:,1)=reshape(Apoints(:,1),[m n]);
A(:,:,2)=reshape(Apoints(:,2),[m n]);

end


function B=turn(A,a)
if a==2
    B=flipud(A(:,:,1));
    B(:,:,2)=flipud(A(:,:,2));
elseif a==3
    B=fliplr(A(:,:,1));
    B(:,:,2)=fliplr(A(:,:,2));
elseif a==4
    B=flipud(fliplr(A(:,:,1)));
    B(:,:,2)=flipud(fliplr(A(:,:,2)));  
else 
    B=A;
end
end

%--------------------------------------------------------------------------
function [Ix2, Iy2, Ixy] = computeJacobianEntries(Ix, Iy)

Ix2 = Ix .^ 2;
Iy2 = Iy .^ 2;
Ixy = Ix .* Iy;

G = fspecial('gaussian', 7, 1.5);

Ix2 = imfilter(Ix2, G);
Iy2 = imfilter(Iy2, G);
Ixy = imfilter(Ixy, G);

end
%--------------------------------------------------------------------------
function board = growCheckerboard(points, scores, Ix2, Iy2, Ixy, theta)

% Exit immediately if no corner points were found
if isempty(scores)
    if isempty(coder.target)
        board = struct('BoardIdx', zeros(3), 'BoardCoords', zeros(3,3,3), ...
            'Energy', Inf, 'isValid', 0);
    else
        board = vision.internal.calibration.checkerboard.Checkerboard;
    end
    return;
end

% only use corners with high scores as seeds to reduce computation
seedIdx = 1:size(points, 1);
[~, sortedIdx] = sort(scores(seedIdx), 'descend');
seedIdx = seedIdx(sortedIdx);
seedIdx = seedIdx(1:round(numel(seedIdx / 2)));

angleThreshold = 3 * pi / 16;

if isempty(coder.target)
    v1_matrix = [];
    v2_matrix = [];
    seedIdx_matrix = [];
    
    for i = seedIdx
        [v1, v2] = cornerOrientations(Ix2, Iy2, Ixy, round(points(i, :)));
        alpha1 = abs(atan2(v1(2), v1(1)));
        alpha2 = abs(atan2(v2(2), v2(1)));
        if abs(abs(alpha1 - pi) - theta) > angleThreshold && ...
                abs(abs(alpha2 - pi) - theta) > angleThreshold
            continue;
        else
            v1_matrix = [v1_matrix;v1]; %#ok<AGROW>
            v2_matrix = [v2_matrix;v2]; %#ok<AGROW>
            seedIdx_matrix = [seedIdx_matrix;i]; %#ok<AGROW>
        end
    end
    
    board = visionInitializeAndExpandCheckerboard(seedIdx_matrix,single(points),v1_matrix,v2_matrix);
else
    previousBoard = vision.internal.calibration.checkerboard.Checkerboard;
    currentBoard = vision.internal.calibration.checkerboard.Checkerboard;
    for i = 1:numel(seedIdx)
        [v1, v2] = cornerOrientations(Ix2, Iy2, Ixy, round(points(seedIdx(i), :)));
        alpha1 = abs(atan2(v1(2), v1(1)));
        alpha2 = abs(atan2(v2(2), v2(1)));
        if abs(abs(alpha1 - pi) - theta) > angleThreshold && ...
                abs(abs(alpha2 - pi) - theta) > angleThreshold
            continue;
        end
        
        currentBoard.initialize(seedIdx(i), points, v1, v2);
        expandBoardFully(currentBoard);
        if currentBoard.Energy < previousBoard.Energy
            tmpBoard = previousBoard;
            previousBoard = currentBoard;
            currentBoard = tmpBoard;
        end
    end
    board = previousBoard;
end
end

%--------------------------------------------------------------------------
function [v1, v2] = cornerOrientations(Ix2, Iy2, Ixy, p)
% The orientation vectors are the eigen vectors of the
% structure tensor:
% [Ix^2  Ixy ]
% [Ixy   Iy^2]

a = Ix2(p(2), p(1));
b = Ixy(p(2), p(1));
c = Iy2(p(2), p(1));

% % Computing eigenvectors "by hand", because the eig() function behaves
% % differently in codegen.
% % Since the matrix is positive-semidefinite, its eigenvectors are
% % orthogonal. Compute the first eigenvector, then swap its elements and
% % negate the y-component to make the second one.
sm = a + c;
df = a - c;
adf = abs(df);
tb = b + b;
ab = abs(tb);

if adf > ab
    rt = adf * sqrt(1 + (ab/adf)^2);
elseif adf < ab
    rt = ab * sqrt(1 + (adf/ab)^2);
else
    rt = ab * sqrt(2);
end

if sm < 0
    sgn1 = -1;
else
    sgn1 = 1;
end

if df > 0
    cs = df + rt;
    sgn2 = 1;
else
    cs = df - rt;
    sgn2 = -1;
end

acs = abs(cs);
if acs > ab
    ct = -tb / cs;
    sn1 = 1 / sqrt(1 + ct * ct);
    cs1 = ct * sn1;
else
    if ab == single(0)
        cs1 = single(1);
        sn1 = single(0);
    else
        tn = -cs / tb;
        cs1 = 1 / sqrt(1 + tn * tn);
        sn1 = tn * cs1;
    end
end
if sgn1 == sgn2
    tn = cs1;
    cs1 = -sn1;
    sn1 = tn;
end

v1 = [-sn1, cs1];
v2 = [cs1, sn1];

% Rotate the vectors by 45 degrees to align with square edges.
R = [cos(pi/4) -sin(pi/4); sin(pi/4) cos(pi/4)];
v1 = v1 * R;
v2 = v2 * R;
end

%--------------------------------------------------------------------------

function [points, boardSize] = toPoints(this)
% returns the points as an Mx2 matrix of x,y coordinates, and
% the size of the board

if any(this.BoardIdx(:) == 0)
    points = [];
    boardSize = [0 0];
    return;
end

numPoints = size(this.BoardCoords, 1) * size(this.BoardCoords, 2);
points = zeros(numPoints, 2);
x = this.BoardCoords(:, :, 1)';
points(:, 1) = x(:);
y = this.BoardCoords(:, :, 2)';
points(:, 2) = y(:);
boardSize = [size(this.BoardCoords, 2)+1, size(this.BoardCoords, 1)+1];
end


%--------------------------------------------------------------------------
function board = orient(board, I)

if ~isinf(board.Energy)
    % orient the board so that the long side is the X-axis
    if size(board.BoardCoords, 1) < size(board.BoardCoords, 2)
        board = rot90_checkerboard(board, 1);
    end
    
    % try to orient the board so that (0,0) is on a black square
    if ~isUpperLeftBlack(board, I);
        board = rot90_checkerboard(board, 2);
    end
    
    % if both sides are odd or both sides are even, make sure
    % that (0,0) is at the upper-left corner.
    if ~xor(mod(size(board.BoardCoords, 1), 2) == 0,...
            mod(size(board.BoardCoords, 2), 2) == 0)
        if any(board.BoardCoords(1,1,:) > board.BoardCoords(end, end, :))
            board = rot90_checkerboard(board, 2);
        end
    end
end
end

%--------------------------------------------------------------------------
function board = rot90_checkerboard(board, k)
board.BoardIdx = rot90(board.BoardIdx, k);
newBoardCoords1 = rot90(board.BoardCoords(:,:,1), k);
newBoardCoords2 = rot90(board.BoardCoords(:,:,2), k);
board.BoardCoords = cat(3, newBoardCoords1, newBoardCoords2);
end

%--------------------------------------------------------------------------

function tf = isUpperLeftBlack(this, I)
% check if the upper-left square of the board is black

% create a mask for the upper-left square
upperLeftPolyX = [this.BoardCoords(1, 1, 1), ...
    this.BoardCoords(1, 2, 1), this.BoardCoords(2, 2, 1), ...
    this.BoardCoords(2, 1, 1)];
upperLeftPolyY = [this.BoardCoords(1, 1, 2), ...
    this.BoardCoords(1, 2, 2), this.BoardCoords(2, 2, 2), ...
    this.BoardCoords(2, 1, 2)];
upperLeftMask = poly2RectMask(upperLeftPolyX, upperLeftPolyY, ...
    size(I, 1), size(I, 2));

% create a mask for the square to the right of it
nextSquarePolyX = [this.BoardCoords(1, 2, 1), ...
    this.BoardCoords(1, 3, 1), this.BoardCoords(2, 3, 1), ...
    this.BoardCoords(2, 2, 1)];
nextSquarePolyY = [this.BoardCoords(1, 2, 2), ...
    this.BoardCoords(1, 3, 2), this.BoardCoords(2, 3, 2), ...
    this.BoardCoords(2, 2, 2)];
nextSquareMask = poly2RectMask(nextSquarePolyX, nextSquarePolyY,...
    size(I, 1), size(I, 2));

% check if the first square is darker than the second
tf = mean(mean(I(upperLeftMask))) < mean(mean(I(nextSquareMask)));
end

%--------------------------------------------------------------------------
function mask = poly2RectMask(X, Y, height, width)
X = sort(X);
Y = sort(Y);
x1 = X(2);
x2 = X(3);
y1 = Y(2);
y2 = Y(3);
mask = false(height, width);
mask(y1:y2, x1:x2) = true;
end

