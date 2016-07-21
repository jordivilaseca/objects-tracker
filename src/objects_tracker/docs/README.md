# Documentation

## Introduction

This piece of software makes easy the stage of objects segmentation and recognition using point clouds for fixed scenes with objects places above planes. This package is capable of finding the number of planes the user wants (only rectangular planes), finding the limits and producing an intelligent segmentation based on this information. It is considered a valid point the one that is above one plane, inside its limits and is not part of any plane (distance superior to 1.5 centimetres between the plane and the point). The planes and their limits are computed every time the segmentation node is started. To avoid errors, an approximate orientation of the planes must be introduced (*tf_calibration* node provides an easy way to do it).

Once the objects are segmented the recognition step is executed and the identifiers of the objects are produced. The training of the model could be easily done using the 'training' node. It provides and easy way to take photos of the objects in different positions and an automatic way to train and store the model.

Finally, the tf_publisher and tf_calibration node allows to has multiple cameras using the same coordinate frame, and easing the work its integration with bigger systems.

### Original scene with four planes
![Image Alt](./img/scene_with_planes.png)

### Recognised scene
![Image Alt](./img/final_result.png)

## Nodes provided
- **point_cloud**. It creates the point cloud given a depth and a colour image, the point cloud is published using a ROS message.

- **segmentation**. It finds the planes of the scene and their limits, it produces the extraction of the objects based on these planes. The results contain the point cloud of the aisled object together with its position and oriented bounding box.
	
- **objects_recognition**. Implements the recognition pipeline. It complements the segmentation output by adding the object identifier.

- **training**. It contains different tools for adding training and testing data, training machine learning methods using different parameters and descriptors and finally extracting results.
To capture new objects the only thing that it has to be done is to put the tag in a visible area, the node itself finds the tag position and removes everything from the scene that is not above the tag. This functionality eases the step of adding new training, testing and validation data because a new picture is taken every time the user press enter. To obtain data from another position, the only thing to be done is start the node again to find the new position of the tag.
It also provides a menu to change the parameters of the recognition step and a way to do online tests in the same way than the training. But the most useful possibility is to compute the metrics and confusion matrix for a specific testing set, most part of the results were extracted using this functionality of the tool.

- **objects_to_rviz**. *rviz* cannot understand the system output directly, so in this node it is translated to elements that *rviz* could understand. To do this the objects are joined to make a point cloud that contains them all, the oriented bounding boxes are translated to primitives and the identifier of an object it is sent (if it exists).

- **tf_calibration**. A transformation between different frames is computed with the intention of integrating more elements in the system.

- **tf_publisher**. It is in charge of sending the transform between frames during the execution of the system.