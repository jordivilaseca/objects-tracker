#include <objects_tracker/utilities/pcl.hpp>

using namespace std;

// Cluster variables.
const int MAX_CLUSTER_POINTS = 500000;

/***********************
   Colour point cloud
***********************/

/**
 * @brief It colour a point.
 * 
 * @param point Point to colour.
 * @param r Quantity of red [0-255].
 * @param g Quantity of green [0-255].
 * @param b Quantity of blue [0-255].
 */
void colourPoint(pcl::PointXYZRGBA &point, int r, int g, int b) {
	point.r = r;
	point.g = g;
	point.b = b;
}

/**
 * @brief It colours a point cloud with a given colour.
 * 
 * @param cloud Point cloud to colour.
 * @param inliers Point of the point cloud that have to be coloured.
 * @param r Quantity of red [0-255].
 * @param g Quantity of green [0-255].
 * @param b Quantity of blue [0-255].
 */
void colourPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const boost::shared_ptr<std::vector<int> > &inliers, uint8_t r, uint8_t g, uint8_t b) {
	for(uint i = 0; i < inliers->size(); i++) {
		int p = (*inliers)[i];
		colourPoint(cloud->points[p], r, g, b);
	}
}

/**
 * @brief It colours a point cloud with a given colour.
 * 
 * @param cloud Point cloud to colour.
 * @param inliers Point of the point cloud that have to be coloured.
 * @param r Quantity of red [0-255].
 * @param g Quantity of green [0-255].
 * @param b Quantity of blue [0-255].
 */
void colourPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const std::vector<int> &inliers, uint8_t r, uint8_t g, uint8_t b) {
	for(uint i = 0; i < inliers.size(); i++) {
		int p = inliers[i];
		colourPoint(cloud->points[p], r, g, b);
	}
}

/**
 * @brief It colours a point cloud with a given colour.
 * 
 * @param cloud Point cloud to colour.
 * @param inliers Point of the point cloud that have to be coloured.
 * @param r Quantity of red [0-255].
 * @param g Quantity of green [0-255].
 * @param b Quantity of blue [0-255].
 */
void colourPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const pcl::PointIndices &inliers, uint8_t r, uint8_t g, uint8_t b) {
	for(uint i = 0; i < inliers.indices.size(); i++) {
		int p = inliers.indices[i];
		colourPoint(cloud->points[p], r, g, b);
	}
}

/**
 * @brief It colours the point cloud using different colours for the different clusters of points.
 * 
 * @param cloud Point cloud to colour.
 * @param inliers Vector of points to colour, each cluster of points will be painted using a different colour.
 */
void colourPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const std::vector<pcl::PointIndices> &inliers) {
	std::vector< std::vector<int> > colours;
	computeColors(inliers.size(), colours);
	for(int i = 0; i < colours.size(); i++) {
		colourPointCloud(cloud, inliers[i].indices, colours[i][0], colours[i][1], colours[i][2]);
	}
}

/**
 * @brief It colours the points that are at a distance inferior to the maximum specified.
 * 
 * @param cloud Point cloud to colour.
 * @param inputIndices Points that are candidates to be coloured.
 * @param coef Coefficients of the line.
 * @param maxDist Maximum distance for a point to be coloured.
 * @param r Quantity of red [0-255].
 * @param g Quantity of green [0-255].
 * @param b Quantity of blue [0-255].
 */
void colourLine(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const pcl::PointIndices::ConstPtr &inputIndices, const pcl::ModelCoefficients &coef, double maxDist, int r, int g, int b) {
	for (int i = 0; i < inputIndices->indices.size(); i++) {
		int pos = inputIndices->indices[i];
		pcl::PointXYZRGBA p = cloud->points[pos];

		Eigen::Vector4f pt = Eigen::Vector4f(p.x, p.y, p.z, 0);
		Eigen::Vector4f line_pt = Eigen::Vector4f(coef.values[0], coef.values[1], coef.values[2], 0);
		Eigen::Vector4f line_dir = Eigen::Vector4f(coef.values[3], coef.values[4], coef.values[5], 0);
		double dist = pcl::sqrPointToLineDistance(pt, line_pt, line_dir);

		if (dist <= maxDist) {
			colourPoint(cloud->points[pos], r, g, b);
		}
	}
}

/***********************
    Point functions
***********************/

/**
 * @brief Create point with specific coordinates.
 * 
 * @param x Coordinate X.
 * @param y Coordinate Y.
 * @param z Coordinate Z.
 * @return Point.
 */
pcl::PointXYZRGBA createXYZRGBA(double x, double y, double z) {
	pcl::PointXYZRGBA p = pcl::PointXYZRGBA();
	p.x = x;
	p.y = y;
	p.z = z;
	return p;
}

/**
 * @brief It computes the area between two vectors (pq and pr).
 * 
 * @param p Initial point of the two vectors.
 * @param q End point of the first vector.
 * @param r End point of the second vector.
 * @return Area.
 */
double triangleArea(const pcl::PointXYZRGBA &p, const pcl::PointXYZRGBA &q, const pcl::PointXYZRGBA &r) {
	double pq[] = {q.x - p.x, q.y - p.y, q.z - p.z};
	double pr[] = {r.x - p.x, r.y - p.y, r.z - p.z};

	double dpq = sqrt(pq[0]*pq[0] + pq[1]*pq[1] + pq[2]*pq[2]);
	double dpr = sqrt(pr[0]*pr[0] + pr[1]*pr[1] + pr[2]*pr[2]);

	return fabs(dpq*dpr/2.0);
}

/**
 * @brief Find the angle between two vectors build with 3 points (pq and pr) knowing that they are part of a plane.
 * 
 * @param p Initial point of the two vectors.
 * @param q End point of the first vector.
 * @param r End point of the second vector.
 * @param n Normal of the plane where the vectors are embedded.
 * @return Angle in degree between the two vectors.
 */
double vectAngle3dEmbeddedPlane(const pcl::PointXYZRGBA &p, const pcl::PointXYZRGBA &q, const pcl::PointXYZRGBA &r, const std::vector<float> &n) {
	double x1,y1,z1,x2,y2,z2, xn, yn, zn;

	x1 = q.x - p.x;
	y1 = q.y - p.y;
	z1 = q.z - p.z;

	x2 = r.x - p.x;
	y2 = r.y - p.y;
	z2 = r.z - p.z;

	xn = n[0];
	yn = n[1];
	zn = n[2];

	double dot = x1*x2 + y1*y2 + z1*z2;
	double det = x1*y2*zn + x2*yn*z1 + xn*y1*z2 - z1*y2*xn - z2*yn*x1 - zn*y1*x2;
	double angle = atan2(det, dot);

	return angle*57.2957795;
}

/**
 * @brief It returns the absolute angle between two vectors.
 * 
 * @param x1 Coordinate X of the first vector.
 * @param y1 Coordinate Y of the first vector.
 * @param z1 Coordinate Z of the first vector.
 * @param x2 Coordinate X of the second vector.
 * @param y2 Coordinate Y of the second vector.
 * @param z2 Coordinate Z of the second vector.
 * @return Absolute degree between them.
 */
float vectAngle3d(float x1, float y1, float z1, float x2, float y2, float z2) {
	float dot = x1*x2 + y1*y2 + z1*z2;
	float lenSq1 = x1*x1 + y1*y1 + z1*z1;
	float lenSq2 = x2*x2 + y2*y2 + z2*z2;
	float angle = acos(dot/sqrt(lenSq1 * lenSq2));
	return angle*57.2957795;
}

/**
 * @brief It returns the absolute angle between thwo vectors.
 * 
 * @param p Initial point of the two vectors.
 * @param q End point of the first vector.
 * @param r End point of the second vector.
 * @return Absolute degree between them.
 */
float vectAngle3d(const pcl::PointXYZRGBA &p, const pcl::PointXYZRGBA &q, const pcl::PointXYZRGBA &r) {
	float x1,y1,z1,x2,y2,z2;

	x1 = q.x - p.x;
	y1 = q.y - p.y;
	z1 = q.z - p.z;

	x2 = r.x - p.x;
	y2 = r.y - p.y;
	z2 = r.z - p.z;

	return vectAngle3d(x1, y1, z1, x2, y2, z2);
}

/**
 * @brief Squared distance between two points.
 * 
 * @param p First point
 * @param q Second point
 * 
 * @return Squared distance.
 */
double distPoints(const pcl::PointXYZRGBA &p, const pcl::PointXYZRGBA &q) {
	double dx = p.x - q.x;
	double dy = p.y - q.y;
	double dz = p.z - q.z;
	return dx*dx + dy*dy + dz*dz;
}

/*
 * It returns the orientation of the point pd, using as a reference the plane limited by pa, pb and pc.
 */

 /**
  * @brief 3D orientation test. It returns the position of a point in reference to a plane formed by three points.
  * 
  * @param pa First point of the plane.
  * @param pb Second point of the plane
  * @param pc Third point of the plane.
  * @param pd Query point.
  * @return Relative orientation. A 0 means the point lies on the plane, -1 and +1 that are at one side or another.
  */
int orient3d(const pcl::PointXYZRGBA &pa, const pcl::PointXYZRGBA &pb, const pcl::PointXYZRGBA &pc, const pcl::PointXYZRGBA &pd) {
	double adx, bdx, cdx;
	double ady, bdy, cdy;
	double adz, bdz, cdz;

	adx = pa.x - pd.x;
	bdx = pb.x - pd.x;
	cdx = pc.x - pd.x;
	ady = pa.y - pd.y;
	bdy = pb.y - pd.y;
	cdy = pc.y - pd.y;
	adz = pa.z - pd.z;
	bdz = pb.z - pd.z;
	cdz = pc.z - pd.z;

	double orientation = adx * (bdy * cdz - bdz * cdy)
					+ bdx * (cdy * adz - cdz * ady)
					+ cdx * (ady * bdz - adz * bdy);

	if (orientation < 0.0) return -1;
	if (orientation > 0.0) return +1;
	return 0;
}

/**
 * @brief It corrects the normal of a plane, making that its direction is in the direction of the side of the plane that is seen.
 * 
 * @param origin Camera position.
 * @param p A point of the plane.
 * @param coef Normal of the plane.
 */
void correctNormal(const float origin[], const pcl::PointXYZRGBA &p, pcl::ModelCoefficients &coef) {
	float originVect[] = {origin[0] - p.x, origin[1] - p.y, origin[2] - p.z};
	float angle = vectAngle3d(originVect[0], originVect[1], originVect[2], coef.values[0], coef.values[1], coef.values[2]);

	if (angle > 90.0) {
		coef.values[0] = -coef.values[0];
		coef.values[1] = -coef.values[1];
		coef.values[2] = -coef.values[2];
		coef.values[3] = -coef.values[3];
	}
}

/***********************
    Polygon functions
***********************/

/**
 * @brief It computes the centroid of a polygon.
 * 
 * @param cloud Point cloud in which the computations are done.
 * @param polygon Indices of the polygon.
 * @param center Center coordinates.
 */
void polygonCenter(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::PointIndices &polygon, std::vector<double> &center) {
	int ndim = 3;
	center = std::vector<double>(ndim, 0.0);
	for(int i = 0; i < polygon.indices.size(); i++) {
		int p = polygon.indices[i];
		center[0] += cloud->points[p].x;
		center[1] += cloud->points[p].y;
		center[2] += cloud->points[p].z;
	}

	for(int j = 0; j < ndim; j++) {
		center[j] /= (float) polygon.indices.size();
	}
}

/**
 * @brief Given polygon describing the limits of a flat surface it returns if the point is inside its limits or not.
 * 
 * @param cloud Point cloud in which the computations are done.
 * @param point Index of the point to test.
 * @param polygon Points describing the limits of the polygon.
 * @param n Normal of the flat surface.
 * 
 * @return [description]
 */
bool isInlier(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, int point, const std::vector<pcl::PointXYZRGBA> &polygon, const Eigen::Vector4f &n) {

	pcl::PointXYZRGBA p = cloud->points[point];
    int npoints = polygon.size();
	pcl::PointXYZRGBA p0 = polygon[0];
    pcl::PointXYZRGBA p1 = polygon[1];
    pcl::PointXYZRGBA p2 = createXYZRGBA(p0.x + n[0], p0.y + n[1], p0.z + n[2]);
	int relPos = orient3d(p0, p1, p2, p);

	for(int i = 1; i < npoints; i++) {
        pcl::PointXYZRGBA p0 = polygon[i];
        pcl::PointXYZRGBA p1 = polygon[(i+1) % npoints];
        pcl::PointXYZRGBA p2 = createXYZRGBA(p0.x + n[0], p0.y + n[1], p0.z + n[2]);

		// We get the relative position of the third vertex of the triangle relative the
		// first and the second one. If a point is inside the polygon, it's relative position
		// must be the same for all segments of the triangle.
		int currRelPos = orient3d(p0, p1, p2, p);

		if (relPos != currRelPos) {
			return false;
		}
	}

	// The point is inside the polygon or in the boundary.
    return true;
}

/**
 * @brief It orders angularly the vertices of a convex polygon.
 * 
 * @param cloud Point cloud in which the computations are done.
 * @param n Normal of the polygon
 * @param polygon Points to sort.
 */
void orderConvexPolygon(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const std::vector<float> &n, pcl::PointIndices &polygon) {

	int npoints = polygon.indices.size();
	std::priority_queue< pair<double,int> > q;

	std::vector<double> center;
	polygonCenter(cloud, polygon, center);

	// Point inside plane 1.
	pcl::PointXYZRGBA p1;
	p1.x = center[0];
	p1.y = center[1];
	p1.z = (p1.x*n[0] + p1.y*n[1] + n[3])/-n[2];

	// Point inside plane 2.
	pcl::PointXYZRGBA p2;
	p2.x = center[0];
	p2.y = center[1] + 1;
	p2.z = (p2.x*n[0] + p2.y*n[1] + n[3])/-n[2];

	// Insert pairs to a priority_queue.
	for(int i = 0; i < npoints; i++) {
		int pos = polygon.indices[i];
		q.push(make_pair(vectAngle3dEmbeddedPlane(p1, p2, cloud->points[pos], n), pos));
	}

	// Get the ordered points.
	for(int i = 0; i < npoints; i++) {
		pair<double,int> pa = q.top();
		q.pop();

		polygon.indices[i] = pa.second;
	}
}

/**
 * @brief Algorithm to simplify a polygon.
 * @details Algorithm inspired in https://bost.ocks.org/mike/simplify using lazy deletion in the queue to avoid
 *updating the priority queue key (that cannot be done using priority_queue nowadays). This can be done because when we update
 *a point for the removal of one of its neighbours, the angle will always decrease as a result of having a convex polygon. 
 *But we want to get first the biggest angles (the ones that are more similar to 180º), so when we get the top of the queue, 
 *this value could be an old angle value (because it was previously updated), so what we will do is to each time a duplicated 
 *item is pushed to the queue, we will increment counter for this item. When we get the top from the queue, if there is one ore
 *more duplications of this item in the queue, we will discard the angle value and decrement the counter of the item.
 *
 *WARNING: this can be done because we know for sure that the angles will always decrease (for the convexity of the polygon), and
 *the maximum angle will be always smaller than 180º.
 * 
 * @param cloud Point cloud in which the computations are done.
 * @param polygon Polygon to simplify.
 * @param n Normal of the polygon.
 * @param maxPoints Points of the returned polygon.
 * @param [out] simPolygon Simplified polygon.
 */
typedef pair<double, int> pdi;
void polygonSimplification(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::PointIndices::ConstPtr &polygon, const std::vector<float> &n, int maxPoints, pcl::PointIndices &simPolygon) {
	int npoints = polygon->indices.size();
	std::vector< pair<int,int> > neighbours = std::vector< pair<int,int> >(npoints, make_pair(-1,-1));
	std::vector<int> duplicationsCounter = std::vector<int>(npoints, 0);
	std::priority_queue< pdi > pqueue;
	std::stack<int> idsStack;

	// Initialization of queue and neighbours.
	for(int i = 0; i < npoints; i++) {
		
		// Calculus of the next and previous points.
		int prev = (i + npoints - 1)%npoints;
		int next = (i+1)%npoints;

		pcl::PointXYZRGBA p, q, r;
		p = cloud->points[polygon->indices[i]];
		q = cloud->points[polygon->indices[prev]];
		r = cloud->points[polygon->indices[next]];

		// Add angle and point identifier to the queue.
		double angles = fabs(vectAngle3dEmbeddedPlane(p, q, r, n));
		pqueue.push(make_pair(angles, i));

		// Initialize neighbours of the point.
		neighbours[i] = make_pair(prev, next);
	}
	// Polygon simplification.
	while(pqueue.size() > 1) {
		pair<double, int> p = pqueue.top();
		pqueue.pop();

		int id = p.second;
		double angle = p.first;

		// Deal with the case with duplications.
		if (duplicationsCounter[id] > 0) {
			duplicationsCounter[id]--;
			continue;
		}

		// At this point we are sure that the angle is correct.
		idsStack.push(polygon->indices[id]);

		int prev = neighbours[id].first;
		int next = neighbours[id].second;
		pcl::PointXYZRGBA prevPoint = cloud->points[polygon->indices[prev]];
		pcl::PointXYZRGBA nextPoint = cloud->points[polygon->indices[next]];

		// Update previous.
		int prev2 = neighbours[prev].first;
		pcl::PointXYZRGBA prev2Point = cloud->points[polygon->indices[prev2]];
		double anglePrev = fabs(vectAngle3dEmbeddedPlane(prevPoint, prev2Point, nextPoint, n));
		pqueue.push(make_pair(anglePrev, prev));
		neighbours[prev].second = next;
		duplicationsCounter[prev]++;

		// Update next.
		int next2 = neighbours[next].second;
		pcl::PointXYZRGBA next2Point = cloud->points[polygon->indices[next2]];
		double angleNext = fabs(vectAngle3dEmbeddedPlane(nextPoint, prevPoint, next2Point, n));
		pqueue.push(make_pair(angleNext, next));
		neighbours[next].first = prev;
		duplicationsCounter[next]++;

		// Update removed point.
		neighbours[id] = make_pair(-1,-1);
		duplicationsCounter[id]--;
	}
	int id = pqueue.top().second;
	pqueue.pop();
	idsStack.push(polygon->indices[id]);

	// Copy results to simPolygon.
	simPolygon = pcl::PointIndices();
	simPolygon.header = polygon->header;
	simPolygon.indices.resize(maxPoints);
	for(int i = 0; i < maxPoints; i++) {
		simPolygon.indices[i] = idsStack.top();
		idsStack.pop();
	}

	// Order simPolygon indices.
	orderConvexPolygon(cloud, n, simPolygon);
	idsStack = stack<int>();
}

/***********************
  Point cloud functions
***********************/

/**
 * @brief It computes the convex hull of a point cloud.
 * 
 * @param cloud Point cloud in which the computations are done.
 * @param inputIndices Points over which the convex hull is computed.
 * @param [out] hullIndices Points that are part of the convex hull.
 */
void findConvexHull(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::PointIndices::ConstPtr &inputIndices, pcl::PointIndices &hullIndices) {
	// Create a Concave Hull representation of the projected inliers
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGBA>);
	hullIndices = pcl::PointIndices();
	pcl::ConvexHull<pcl::PointXYZRGBA> chull;
	chull.setInputCloud(cloud);
	chull.setIndices(inputIndices);
	chull.setDimension(2);
	chull.reconstruct(*cloud_hull);
	chull.getHullPointIndices(hullIndices); 
}

/**
 * @brief It computes a region growing segmentation based on normals.
 * 
 * @param cloud Point cloud in which the computations are done.
 * @param indices Point for which the region growing must be computed.
 * @param normals Normal of each point of the point cloud.
 * @param tree Search tree that must be used.
 * @param angleThresh Maximum angles accepted for a valid point.
 * @param curvatureThresh Points with an angle bigger than that will start a new cluster.
 * @param nNeigh Neighbours to consult.
 * @param minClusSize Minimum size of a cluster.
 * @param [out] clusters Returned segmentation.
 */
void regionGrowing(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::IndicesPtr &indices, const pcl::PointCloud<pcl::Normal>::Ptr &normals, const pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr &tree, double angleThresh, double curvatureThresh, int nNeigh, int minClusSize, std::vector<pcl::PointIndices> &clusters) {
	pcl::RegionGrowing<pcl::PointXYZRGBA, pcl::Normal> reg;
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(nNeigh);
	reg.setMinClusterSize(minClusSize);
	reg.setInputCloud(cloud);
	reg.setIndices(indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(angleThresh);
	reg.setCurvatureThreshold(curvatureThresh);

	clusters = std::vector<pcl::PointIndices>();
	reg.extract (clusters);
}

/**
 * @brief It computes an euclidean clustering.
 * 
 * @param cloud Point cloud in which the computations are done.
 * @param tolerance Points with a distance bigger than that will start a new cluster.
 * @param minSize Minimum size of a cluster.
 * @param [out] clusterIndices Returned segmentation. 
 */
void clustering(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, float tolerance, int minSize, std::vector<pcl::PointIndices> &clusterIndices) {
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
	tree->setInputCloud(cloud);

	clusterIndices = std::vector<pcl::PointIndices>();

	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
	ec.setClusterTolerance(tolerance);	// 1cm
	ec.setMinClusterSize(minSize);
	ec.setMaxClusterSize(MAX_CLUSTER_POINTS);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(clusterIndices);
}

/**
 * @brief It computes an euclidean clustering.
 * 
 * @param cloud Point cloud in which the computations are done.
 * @param inputIndices Indices for which the clustering will be computed.
 * @param tolerance Points with a distance bigger than that will start a new cluster.
 * @param minSize Minimum size of a cluster.
 * @param [out] clusterIndices Returned segmentation. 
 */
void clustering(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::PointIndices::ConstPtr &inputIndices, float tolerance, int minSize, std::vector<pcl::PointIndices> &clusterIndices) {
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
	tree->setInputCloud(cloud, boost::make_shared<std::vector<int>>(inputIndices->indices));

	clusterIndices = std::vector<pcl::PointIndices>();

	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
	ec.setClusterTolerance(tolerance);	// 1cm
	ec.setIndices(inputIndices);
	ec.setMinClusterSize(minSize);
	ec.setMaxClusterSize(MAX_CLUSTER_POINTS);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(clusterIndices);
}

/**
 * @brief Estimate the normal for each one of the points of the point cloud.
 * 
 * @param cloud Point cloud in which the computations are done.
 * @param [out] normals Computed normals. 
 * @param thresh Radius used for the computation of a normal.
 */
void estimateNormals(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, double thresh) {
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	//tree->setInputCloud(cloud);
	ne.setSearchMethod(tree);

	normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
	ne.setRadiusSearch(thresh);
	ne.compute(*normals);
}

/**
 * @brief Estimate the normal for each one of the points of the point cloud.
 * 
 * @param cloud Point cloud in which the computations are done.
 * @param [out] normals Computed normals. 
 * @param thresh Radius used for the computation of a normal.
 */
void estimateNormals(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, double thresh) {
	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
	tree->setInputCloud(cloud);
	ne.setSearchMethod(tree);

	normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
	ne.setRadiusSearch(thresh);
	ne.compute(*normals);
}

/**
 * @brief Extract indices from a point cloud keeping it organized.
 * 
 * @param input Input point cloud.
 * @param inliers Points to extract.
 * @param [out] New cloud with the selected points of the original one.	
 */
void extractIndices(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &input, const pcl::PointIndices::ConstPtr &inliers, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) {
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	extract.setInputCloud(input);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.setKeepOrganized(true);
	extract.filter(*cloud);
}

/**
 * @brief It reduces and organized point cloud height and width to the minimum window available based on the proportioned indices.
 * 
 * @param cloud Point cloud to update
 * @param indices Indices of the point cloud. They are updated in the function to match the new point cloud
 */
void subPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, pcl::PointIndices &indices) {
	pair<int,int> mi, ma;

	int width = cloud->width;
	int height = cloud->height;

	// Get minimum and maximum point. (in 2D)
	mi.first = ma.first = indices.indices[0] % width;
	mi.second = ma.second = (int) indices.indices[0] / width;
	for (int i : indices.indices) {
		int x = i % width;
		int y = (int) i/width;

		mi.first = min(mi.first, x);
		ma.first = max(ma.first, x);
		mi.second = min(mi.second, y);
		ma.second = max(ma.second, y);
	}

	// Move the object to the new position.
	int newWidth = ma.first - mi.first + 1;
	int newHeight = ma.second - mi.second + 1;
	for(int i = 0; i < newHeight; i++) {
		for(int j = 0; j < newWidth; j++) {
			cloud->points[j + i*newWidth] = cloud->points[mi.first + j + (mi.second + i)*width];
		}
	}

	// Update indices.
	for(int &i : indices.indices) {
		int x = i % width;
		int y = (int) i/width;
		i = (x - mi.first) + (y - mi.second)*newWidth;
	}

	// Update cloud parameters.
	cloud->width = newWidth;
	cloud->height = newHeight;
	cloud->points.resize(newWidth*newHeight);
}

/**
 * @brief It removes the points that the angle between its normal and the normal of a plane is bigger than a threshold.
 * 
 * @param cloud Input cloud
 * @param input Used indices.
 * @param planeCoefficients Reference plane.
 * @param angleThresh Maximum angle.
 * @param [out] output Valid indices after the filtering. 
 */
void filterByNormal(const pcl::PointCloud<pcl::Normal>::ConstPtr cloud, const pcl::PointIndices::ConstPtr &input, const pcl::ModelCoefficients &planeCoefficients, float angleThresh, pcl::PointIndices::Ptr & output) {
	output = pcl::PointIndices::Ptr(new pcl::PointIndices());
	output->indices.resize(input->indices.size());

	std::vector<float> n1 = planeCoefficients.values;

	int nr_p = 0;
	for(int i : input->indices) {
		pcl::Normal n2 = cloud->points[i];

		float angle = vectAngle3d(n1[0], n1[1], n1[2], n2.normal_x, n2.normal_y, n2.normal_z);
		if(angleThresh >= angle or 180.0 - angle <= angleThresh) output->indices[nr_p++] = i;
	}

	output->indices.resize(nr_p);
}

/**
 * @brief It removes the nans of a point cloud.
 * 
 * @param cloud Point cloud to update.
 * @param input Valid indices.
 */
void removeNans(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud,  pcl::PointIndices::Ptr &input) {
	std::vector<int> aux(input->indices);

	int nr_p = 0;
	for(int i : aux) {
		if (!isnan(cloud->points[i].x)) input->indices[nr_p++] = i;
	}
	input->indices.resize(nr_p);
}

/***********************
    Plane functions
***********************/

/**
 * @brief Find the points that are inliers to plane with a defined maximum distance.
 * 
 * @param cloud Input point cloud
 * @param modelCoef Plane coefficients.
 * @param threshold Maximum distance to consider a point inlier. In meters.
 * @param [out] inliers Points inliers to the plane.
 */
void findPlaneInliers(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::ModelCoefficients &modelCoef, float threshold, pcl::IndicesPtr &inliers) {

	Eigen::Vector4f coef = Eigen::Vector4f(modelCoef.values.data());
	int totalPoints = cloud->points.size();
	int nr_p = 0;
	inliers = pcl::IndicesPtr(new vector<int>());
	inliers->resize(totalPoints);
	// Iterate through the 3d points and calculate the distances from them to the plane

	for(size_t i = 0; i < totalPoints; i++) {

		if (isnan(cloud->points[i].x)) continue;
		// Calculate the distance from the point to the plane normal as the dot product
		// D =(P-A).N/|N|
		Eigen::Vector4f pt(cloud->points[i].x,
		                    cloud->points[i].y,
		                    cloud->points[i].z,
		                    1);

		float distance = fabsf(coef.dot(pt));
		if(distance < threshold) {
			// Returns the indices of the points whose distances are smaller than the threshold
			(*inliers)[nr_p] = i;
			nr_p++;
		}
	}
	inliers->resize(nr_p);
}

/**
 * @brief Project points to a plane
 * 
 * @param cloud Input point cloud
 * @param modelCoef Plane coefficients.
 * @param projCloud Projected cloud.
 */
void projectToPlane(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, pcl::ModelCoefficients::ConstPtr modelCoef, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &projCloud) {

	projCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(modelCoef);
	proj.filter(*projCloud);
}

/***********************
       Descriptors
***********************/

/**
 * @brief Compute the VFH descriptor for a set of indices of a point cloud.
 * 
 * @param object Input cloud.
 * @param normals Normals of the point cloud.
 * @param indices Valid indices.
 * @param tree Used serach tree.
 * @param descriptor Obtained descriptor.
 */
void vfh(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &object, const pcl::PointCloud<pcl::Normal>::ConstPtr &normals, const pcl::PointIndices::Ptr &indices, const pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr &tree, pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptor) {
	pcl::VFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud(object);
	vfh.setInputNormals(normals);
	vfh.setIndices(indices);
	vfh.setSearchMethod(tree);
	// Optionally, we can normalize the bins of the resulting histogram,
	// using the total number of points.
	vfh.setNormalizeBins(true);
	vfh.setFillSizeComponent(true);
	// Also, we can normalize the SDC with the maximum size found between
	// the centroid and any of the cluster's points.
	vfh.setNormalizeDistance(true);
 
	vfh.compute(*descriptor);
}