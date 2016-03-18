#include <objects_tracker/utilities/pcl.hpp>

using namespace std;

// Cluster variables.
const int MIN_CLUSTER_POINTS = 10000;
const int MAX_CLUSTER_POINTS = 500000;

long long getTime(){
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long long mslong = (long long) tp.tv_sec * 1000L + tp.tv_usec / 1000;
    return mslong;
}

/***********************
   Colour point cloud
***********************/

void colourPoint(pcl::PointXYZRGBA &point, int r, int g, int b) {
	point.r = r;
	point.g = g;
	point.b = b;
}

void colourPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const boost::shared_ptr<std::vector<int> > &inliers, uint8_t r, uint8_t g, uint8_t b) {
	for(uint i = 0; i < inliers->size(); i++) {
		int p = (*inliers)[i];
		colourPoint(cloud->points[p], r, g, b);
	}
}

void colourPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const std::vector<int> &inliers, uint8_t r, uint8_t g, uint8_t b) {
	for(uint i = 0; i < inliers.size(); i++) {
		int p = inliers[i];
		colourPoint(cloud->points[p], r, g, b);
	}
}

void colourPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const pcl::PointIndices &inliers, uint8_t r, uint8_t g, uint8_t b) {
	for(uint i = 0; i < inliers.indices.size(); i++) {
		int p = inliers.indices[i];
		colourPoint(cloud->points[p], r, g, b);
	}
}

void colourPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const std::vector<pcl::PointIndices> &inliers) {
	std::vector< std::vector<int> > colours;
	computeColors(inliers.size(), colours);
	for(int i = 0; i < colours.size(); i++) {
		colourPointCloud(cloud, inliers[i].indices, colours[i][0], colours[i][1], colours[i][2]);
	}
}

void colourLine(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const pcl::PointIndices::ConstPtr &inputIndices, const pcl::ModelCoefficients &coef, double minDist, int r, int g, int b) {
	for (int i = 0; i < inputIndices->indices.size(); i++) {
		int pos = inputIndices->indices[i];
		pcl::PointXYZRGBA p = cloud->points[pos];

		Eigen::Vector4f pt = Eigen::Vector4f(p.x, p.y, p.z, 0);
		Eigen::Vector4f line_pt = Eigen::Vector4f(coef.values[0], coef.values[1], coef.values[2], 0);
		Eigen::Vector4f line_dir = Eigen::Vector4f(coef.values[3], coef.values[4], coef.values[5], 0);
		double dist = pcl::sqrPointToLineDistance(pt, line_pt, line_dir);

		if (dist <= minDist) {
			colourPoint(cloud->points[pos], r, g, b);
		}
	}
}

/***********************
    Point functions
***********************/

/**
 * Compute the area between two vectors (pq and pr).
 */
double triangleArea(const pcl::PointXYZRGBA &p, const pcl::PointXYZRGBA &q, const pcl::PointXYZRGBA &r) {
	double pq[] = {q.x - p.x, q.y - p.y, q.z - p.z};
	double pr[] = {r.x - p.x, r.y - p.y, r.z - p.z};

	double dpq = sqrt(pq[0]*pq[0] + pq[1]*pq[1] + pq[2]*pq[2]);
	double dpr = sqrt(pr[0]*pr[0] + pr[1]*pr[1] + pr[2]*pr[2]);

	return dpq*dpr/2.0;
}

/**
 * Find the angle between two vectors build with 3 points (pq and pr), knowing that they are part of a plane.
 *
 * p: Initial point of the vectors.
 * q: End point of the first vector.
 * r: End point of the second vector.
 * n: Normal of the plane containing the two vectors.
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

double vectAngle3d(const pcl::PointXYZRGBA &p, const pcl::PointXYZRGBA &q, const pcl::PointXYZRGBA &r) {
	double x1,y1,z1,x2,y2,z2;

	x1 = q.x - p.x;
	y1 = q.y - p.y;
	z1 = q.z - p.z;

	x2 = r.x - p.x;
	y2 = r.y - p.y;
	z2 = r.z - p.z;

	double dot = x1*x2 + y1*y2 + z1*z2;
	double lenSq1 = x1*x1 + y1*y1 + z1*z1;
	double lenSq2 = x2*x2 + y2*y2 + z2*z2;
	double angle = acos(dot/sqrt(lenSq1 * lenSq2));
	return angle*57.2957795;
}


double distPoints(const pcl::PointXYZRGBA &p, const pcl::PointXYZRGBA &q) {
	double dx = p.x - q.x;
	double dy = p.y - q.y;
	double dz = p.z - q.z;
	return dx*dx + dy*dy + dz*dz;
}

double orient3d(double pa, double pb, double pc, double pd) {
  /*double adx, bdx, cdx;
  double ady, bdy, cdy;
  double adz, bdz, cdz;

  adx = pa[0] - pd[0];
  bdx = pb[0] - pd[0];
  cdx = pc[0] - pd[0];
  ady = pa[1] - pd[1];
  bdy = pb[1] - pd[1];
  cdy = pc[1] - pd[1];
  adz = pa[2] - pd[2];
  bdz = pb[2] - pd[2];
  cdz = pc[2] - pd[2];

  return adx * (bdy * cdz - bdz * cdy)
       + bdx * (cdy * adz - cdz * ady)
       + cdx * (ady * bdz - adz * bdy);*/
}



/***********************
    Polygon functions
***********************/

void orderConvexPolygon(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const std::vector<float> &n, pcl::PointIndices &polygon) {

	int npoints = polygon.indices.size();
	std::priority_queue< pair<double,int> > q;

	// Point inside plane 1.
	pcl::PointXYZRGBA p1;
	p1.x = 0;
	p1.y = 0;
	p1.z = n[3]/-n[2];

	// Point inside plane 2.
	pcl::PointXYZRGBA p2;
	p2.x = 1;
	p2.y = 1;
	p2.z = (n[0] + n[1] +n[3])/-n[2];

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
 *Algorithm inspired in https://bost.ocks.org/mike/simplify using lazy deletion in the queue to avoid
 *updating the priority queue key (that cannot be done using priority_queue nowadays). This can be done because when we update
 *a point for the removal of one of its neighbours, the angle will always decrease as a result of having a convex polygon. 
 *But we want to get first the biggest angles (the ones that are more similar to 180º), so when we get the top of the queue, 
 *this value could be an old angle value (because it was previously updated), so what we will do is to each time a duplicated 
 *item is pushed to the queue, we will increment counter for this item. When we get the top from the queue, if there is one ore
 *more duplications of this item in the queue, we will discard the angle value and decrement the counter of the item.
 *
 *WARNING: this can be done because we know for sure that the angles will always decrease (for the convexity of the polygon), and
 *the maximum angle will be always smaller than 180º.
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
		double angles = abs(vectAngle3dEmbeddedPlane(p, q, r, n));
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
		double anglePrev = abs(vectAngle3dEmbeddedPlane(prevPoint, prev2Point, nextPoint, n));
		pqueue.push(make_pair(anglePrev, prev));
		neighbours[prev].second = next;
		duplicationsCounter[prev]++;

		// Update next.
		int next2 = neighbours[next].second;
		pcl::PointXYZRGBA next2Point = cloud->points[polygon->indices[next2]];
		double angleNext = abs(vectAngle3dEmbeddedPlane(nextPoint, prevPoint, next2Point, n));
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

void findLines(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const pcl::PointIndices::ConstPtr &inputIndices, std::vector<pcl::ModelCoefficients> &coef) {
	// Cloud containing the points without the planes.
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr remainingCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud));
	coef = std::vector<pcl::ModelCoefficients>(4);

	// Create the segmentation object.
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;

	// Set segmentation parameters.
	seg.setModelType(pcl::SACMODEL_LINE);
	seg.setIndices(inputIndices);
	seg.setOptimizeCoefficients(true);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(5000);
	seg.setDistanceThreshold(0.005);

	// Create the filtering object.
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

	// At each step, one plane is removed from remainingCloud.
	for(int i = 0; i < 4; i++){

		pcl::PointIndices::Ptr inliers = pcl::PointIndices::Ptr(new pcl::PointIndices());

		// Segment the largest planar component from the remaining cloud.
		seg.setInputCloud(remainingCloud);
		seg.segment(*inliers, coef[i]);

		if (inliers->indices.size() == 0) break;

		// Extract the plane inliers from the remainingCloud.
		extract.setInputCloud(remainingCloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*remainingCloud);
	}
}

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

void clustering(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::PointIndices::ConstPtr &inputIndices, std::vector<pcl::PointIndices> &clusterIndices) {
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
	tree->setInputCloud(cloud);

	clusterIndices = std::vector<pcl::PointIndices>();

	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
	ec.setClusterTolerance(0.005);	// 1cm
	ec.setIndices(inputIndices);
	ec.setMinClusterSize(MIN_CLUSTER_POINTS);
	ec.setMaxClusterSize(MAX_CLUSTER_POINTS);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(clusterIndices);
}

/***********************
    Plane functions
***********************/

void findPlaneInliers(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, pcl::ModelCoefficients modelCoef, float threshold, pcl::IndicesPtr &inliers) {
	Eigen::Vector4f coef = Eigen::Vector4f(modelCoef.values.data());
	int totalPoints = cloud->points.size();
	int nr_p = 0;
	inliers = pcl::IndicesPtr(new vector<int>());
	inliers->resize(totalPoints);
	// Iterate through the 3d points and calculate the distances from them to the plane

	//#pragma omp parallel for firstprivate(threshold, coef) shared(cloud, inliers, nr_p) num_threads(3)
	for(size_t i = 0; i < totalPoints; i++) {
		// Calculate the distance from the point to the plane normal as the dot product
		// D =(P-A).N/|N|
		Eigen::Vector4f pt(cloud->points[i].x,
		                    cloud->points[i].y,
		                    cloud->points[i].z,
		                    1);

		float distance = fabsf(coef.dot(pt));
		if(distance < threshold) {
			// Returns the indices of the points whose distances are smaller than the threshold
			//#pragma omp critical
			//{
				(*inliers)[nr_p] = i;
				nr_p++;
			//}
		}
	}
	inliers->resize(nr_p);

	/*Eigen::Vector4f coef = Eigen::Vector4f(modelCoef.values.data());
	pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA>::Ptr dit(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA> (cloud));
	inliers = pcl::IndicesPtr(new vector<int>());

	// Get plane inliers using 'coef' as plane coefficients.
	dit->selectWithinDistance(coef, maxDist, *inliers);*/
}

void projectToPlane(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, pcl::ModelCoefficients::ConstPtr modelCoef, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &projCloud) {

	projCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(modelCoef);
	proj.filter(*projCloud);
}