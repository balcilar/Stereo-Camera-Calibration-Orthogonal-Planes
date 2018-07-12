# Mono and Stereo Camera Calibration with Orthogonal 3 checkerboard

A camera projects 3D world-points onto the 2D image plane. Traditional cameras have some internal parameter which affect imaging process such as, Image center, Focal length and Lens distortion parameters. In addition to this almost unknown (actually all manufacturers gives their parameters but note that focal lenght will be change when you zoom in or out, and also in order to pyhsical deformations, other parameters might be changed as well) instrinsic parameters, if you want to calculate depth information of scene or 3d scene reconstruction, you have to know positions of cameras according to any reference point. In case of stereo vision application, one camera can be selected as reference point too. That time we need to know rotation and translation of one camera according to another in addition to instrinsic parameters. Generally 1 checkerboard plane is widely used for this aim. Accoding to theoretical background, we need at least 4 different view of one checker board image for all camera. But the more and different image you take, the better accuracy you gain. But if we use 2 or 3 ortogonal checkerboard, we can obtain both instrinsic and extrinsic parameters using only one shot.

In this research, we developed fully automatic 3 orthogonal checkerboard calibration techniques either mono or stereo camera. Actually we just calculated the extrinsic parameters according to outher reference point which is intersection point of 3 orthogonal plane.

## Find Calibration Point From 3 Ortogonal Checkerboard Image

The main issue of this task is find all necessary point and their world plane positions automatically. To reach this aim, we make some assumtions, such the X,Y plane checkerboard  should be 7x8 points, Y,Z plane checkerboard should be 8x6 points, and X,Z plane checkerboard should be 6x8 points. In this way we can easily recognize which sensed plane refers which plane. And also in order to prevent some confussion of intersection line of 3 planes, we do not use that the first row of intersection lines. The row image of our left camera image is in following figure.

![Sample image](Inputs/left_calib.jpg?raw=true "Title")

We manually painted the intersection part of the plane to get the calibration size become as how we desing. Please note that does not mean we all need manual painiting. if you use appropriate calibration checkerboard, you wont need any pre-processing. After all we detect all planes, their image plane locations in superpixel resolution and their real word locations as you can see in following figure.

![Sample image](Outputs/lr.jpg?raw=true "Title")


if you select reference of 

instrinsic parameters, we actually do not know where the camera is according to and selected reference point.  



 
## Reference
