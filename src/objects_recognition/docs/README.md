# Documentation

## Introduction

This piece of software makes easy the stage of objects segmentation and recognition using point clouds for fixed scenes with objects places above planes. This package is capable of finding the number of planes the user wants (only rectangular planes), finding the limits and producing an intelligent segmentation based on this information. It is considered a valid point the one that is above one plane, inside its limits and is not part of any plane. The planes and their limits are computed every time the segmentation node is started. To avoid errors, an approximate orientation of the planes must be introduced (*tf_calibration* node provides an easy way to do it).

Once the objects are segmented the recognition step is executed and the identifiers of the objects are produced. The training of the model could be easily done using the 'training' node. It provides and easy way to take photos of the objects in different positions and an automatic way to train and store the model.

Finally, the tf_publisher and tf_calibration node allows to has multiple cameras using the same coordinate frame, and easing the work its integration with bigger systems.

### Original scene with four planes
![Image Alt](./img/scene_with_planes.png)

### Recognised scene
![Image Alt](./img/final_result.png)

## Software used
It was tested using opencv 3, pcl 1.8 and ROS indigo.

## Nodes provided
- **point_cloud**. It creates the point cloud given a depth and a colour image, the point cloud is published using a ROS message.

- **segmentation**. It finds the planes of the scene and their limits, it produces the extraction of the objects based on these planes. The results contain the point cloud of the aisled object together with its position and oriented bounding box.
	
- **objects_recognition**. Implements the recognition pipeline. It complements the segmentation output by adding the object identifier.

- **training**. It contains different tools for adding training and testing data, training machine learning methods using different parameters and descriptors and finally extracting results.
To capture new objects the only thing that it has to be done is to put the tag in a visible area, the node itself finds the tag position and removes everything from the scene that is not above the tag. This functionality eases the step of adding new training, testing and validation data because a new picture is taken every time the user press enter. To obtain data from another position, the only thing to be done is start the node again to find the new position of the tag.
It also provides a menu to change the parameters of the recognition step and a way to do on-line tests in the same way than the training. But the most useful possibility is to compute the metrics and confusion matrix for a specific testing set, most part of the results were extracted using this functionality of the tool.

- **objects_to_rviz**. *rviz* cannot understand the system output directly, so in this node it is translated to elements that *rviz* could understand. To do this the objects are joined to make a point cloud that contains them all, the oriented bounding boxes are translated to primitives and the identifier of an object it is sent (if it exists).

- **tf_calibration**. A transformation between different frames is computed with the intention of integrating more elements in the system.

- **tf_publisher**. It is in charge of sending the transform between frames during the execution of the system.

## Segmentation

As it was said in the introduction it is used a plane segmentation to extract the objects from the scene. During the initialization of the *segmentation* node the planes are searched using RANSAC, the planes searched will have a similar orientation (with a maximum difference of 10 degree) to the tag found in the *tf_calibration* step.

The planes are limited to obtain better performance and allow the possibility of having multiple planes. This stage consists of multiple steps:
- **Normal filtering**. To remove invalid points, points that it can be known that are not correct because its normal is very different from the normal of the plane.
- **Clustering**. It is assumed that the biggest cluster is the one representing the plane.
- **Convex hull computation**. The result of the convex hull is a set of points, the polygon formed with these points include all the points of the original one. The points returned are around 20.
- **Polygon simplification**. The system works with rectangular planes, this step is oriented to simplify the convex hull and obtaining the four points that describe the plane.
- **Object clustering**. The points are gathered in clusters based on the its distance.

The regions of interest of the scene are the ones that are above a rectangular plane, for these reason the returned points are the ones that are above at least one plane, inside its limits (it avoids spending time of calculus in parts of the scene that are not objects, for example furniture near in the scene) and that is not part of any of the planes used for the segmentation. It is said that a point is part of a plane if the distance between the two is at most 1.5 centimetres.

The objects must be separated at least for 3 centimetres between them to obtain a good segmentation.

## Recognition

The recognition makes use of the colour and shape information of the objects. A colour histogram on the HSV colour space using H and S channels to obtain a descriptor invariant to the illumination, for the shape the descriptor VFH is used.

The descriptor of an object is computed using a Bag of Features scheme. To do this each object is divided in different parts using a region growing clustering algorithm based on normals, and for each part the descriptor described before is calculated. A vocabulary is computed using all the parts of all the objects of the training set, each element of the vocabulary represents a part of an object. Finally, an object is defined by the parts of the vocabulary that it contains. This descriptor is fed to a SVM that returns the identifier of the object.

When an object to recognise arrives, its parts are computed, the descriptor of each part calculated and then a matching between each one of this parts and the vocabulary is done to know to which part of the vocabulary corresponds, the final descriptor is computed and the identifier obtained using a SVM.

### Pipeline
![Image Alt](./img/recognition_pipeline.png)

## Configuration steps
To run a roslaunch the next command must be executed. *roslaunch objects_recognition launch_file*.
- Compute the transformations between the different cameras and the plane orientation, it is made using the roslaunch *tf_calibration.launch* node for all the cameras of the scene. It makes use of a tag and the package *ar_track_alvar* to find a relation from the camera to the tag. It cannot be changed of place during the calibration of the different cameras. The tag must be placed in the same orientation than the planes to be recognised.

- Objects training. The objects must be trained using the roslaunch *objects_training.launch*. It makes use of a tag to extract the object from the scene. The first step is to take the photos from the objects using the option *1*, its is recommended to make photos with different angles and positions. Then the model must be trained and stored, this is done with the option *3*. Different descriptors and parameters could be configured using this node.

## Execution

To execute the system the following command must be executed. *roslaunch objects_recognition objects_recognition.launch*.