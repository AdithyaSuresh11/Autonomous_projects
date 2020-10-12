# `Image/Video Stitching with ROS`

Simple image stitching algorithm using SIFT, homography, KNN and Ransac in Python. For further details and explanations, you're welcome to mail your queries to adithya@clemson.edu. Also, I have uploaded the filtered rosbag of the data that I recorded, for full rosbag contact me through the same email provided.

The project is to implement a real-time featured based automatic image stitching algorithm. The real time video stream was collected with the help of [Pointgrey Firefly](https://studylib.net/doc/18473951/firefly-mv-usb2-datasheet/) camera. This camera data was collected in two custom ROS topics /left/image_raw and /right/image_raw (for left and right cameras respectively). Since the CV2 BFMatcher, WarpPerspective and SIFT are computationally expensive algorithms, the lag might occur in real time.

The live image stream is converted to image data by using CV bridge and then the data is fed as input. Regarding the output, when we input two images with overlapped fields, we expect to obtain a wide seamless panorama.

We use Scale Invariant Features Transform (SIFT) to extract local features of the input images, K nearest neighbors algorithms to match these features and Random Sample Consensus(Ransac) to calculate the homograph matrix, which will be used for image warping. Finally we apply a weighted matrix as a mask for image blending based on the height and width of the image frame. 

## Dependency

CV2 or 3 libraries with SIFT, SURF, ORB algorithms require OpenCV version 3.4.2.16. For installation follow: [OpenCV Installation](https://pylessons.com/OpenCV-image-stiching/)

* Python 2 or 3
* OpenCV-contrib-python 3.4.2.16
* OpenCV-python 3.4.2.16

## Usage

`roscore`
`rosbag play -l image_stitch.bag`
`rosrun [package_name] Stitch.py` or
`cd [directory]/ python Stitch.py` 

## Sample

One frame of matched and stitched image:

### Matching
![](https://github.com/AdithyaSuresh11/Autonomous_projects/blob/main/Stitch/matching.jpg/)

### Panorama View
![](https://github.com/AdithyaSuresh11/Autonomous_projects/blob/main/Stitch/panorama.jpg/)
