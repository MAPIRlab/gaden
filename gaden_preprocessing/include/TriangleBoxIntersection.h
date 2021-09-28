#pragma once

/*------------------------------------------------

	Adapted from https://gist.github.com/jflipts/fc68d4eeacfcc04fbdb2bf38e0911850#file-triangleboxintersection-h

--------------------------------------------------*/

#include <cmath>
#include <eigen3/Eigen/Dense>

inline void findMinMax(float x0, float x1, float x2, float &min, float &max) {
	min = max = x0;
	if (x1 < min)
		min = x1;
	if (x1 > max)
		max = x1;
	if (x2 < min)
		min = x2;
	if (x2 > max)
		max = x2;
}

inline bool planeBoxOverlap(Eigen::Vector3d normal, Eigen::Vector3d vert, Eigen::Vector3d maxbox) {
	Eigen::Vector3d vmin, vmax;
	float v;
	for (size_t q = 0; q < 3; q++) {
		v = vert[q];
		if (normal[q] > 0.0f) {
			vmin[q] = -maxbox[q] - v;
			vmax[q] = maxbox[q] - v;
		} else {
			vmin[q] = maxbox[q] - v;
			vmax[q] = -maxbox[q] - v;
		}
	}
	if (normal.dot(vmin) > 0.0f)
		return false;
	if (normal.dot(vmax) >= 0.0f)
		return true;

	return false;
}

/*======================== X-tests ========================*/

inline bool axisTestX01(float a, float b, float fa, float fb, const Eigen::Vector3d &v0,
	const Eigen::Vector3d &v2, const Eigen::Vector3d &boxhalfsize, float &rad, float &min,
	float &max, float &p0, float &p2) {
	p0 = a * v0.y() - b * v0.z();
	p2 = a * v2.y() - b * v2.z();
	if (p0 < p2) {
		min = p0;
		max = p2;
	} else {
		min = p2;
		max = p0;
	}
	rad = fa * boxhalfsize.y() + fb * boxhalfsize.z();
	if (min > rad || max < -rad)
		return false;
	return true;
}
inline bool axisTestX2(float a, float b, float fa, float fb, const Eigen::Vector3d &v0,
	const Eigen::Vector3d &v1, const Eigen::Vector3d &boxhalfsize, float &rad, float &min,
	float &max, float &p0, float &p1) {
	p0 = a * v0.y() - b * v0.z();
	p1 = a * v1.y() - b * v1.z();
	if (p0 < p1) {
		min = p0;
		max = p1;
	} else {
		min = p1;
		max = p0;
	}
	rad = fa * boxhalfsize.y() + fb * boxhalfsize.z();
	if (min > rad || max < -rad)
		return false;
	return true;
}

/*======================== Y-tests ========================*/

inline bool axisTestY02(float a, float b, float fa, float fb, const Eigen::Vector3d &v0,
	const Eigen::Vector3d &v2, const Eigen::Vector3d &boxhalfsize, float &rad, float &min,
	float &max, float &p0, float &p2) {
	p0 = -a * v0.x() + b * v0.z();
	p2 = -a * v2.x() + b * v2.z();
	if (p0 < p2) {
		min = p0;
		max = p2;
	} else {
		min = p2;
		max = p0;
	}
	rad = fa * boxhalfsize.x() + fb * boxhalfsize.z();
	if (min > rad || max < -rad)
		return false;
	return true;
}

inline bool axisTestY1(float a, float b, float fa, float fb, const Eigen::Vector3d &v0,
	const Eigen::Vector3d &v1, const Eigen::Vector3d &boxhalfsize, float &rad, float &min,
	float &max, float &p0, float &p1) {
	p0 = -a * v0.x() + b * v0.z();
	p1 = -a * v1.x() + b * v1.z();
	if (p0 < p1) {
		min = p0;
		max = p1;
	} else {
		min = p1;
		max = p0;
	}
	rad = fa * boxhalfsize.x() + fb * boxhalfsize.z();
	if (min > rad || max < -rad)
		return false;
	return true;
}

/*======================== Z-tests ========================*/
inline bool axisTestZ12(float a, float b, float fa, float fb, const Eigen::Vector3d &v1,
	const Eigen::Vector3d &v2, const Eigen::Vector3d &boxhalfsize, float &rad, float &min,
	float &max, float &p1, float &p2) {
	p1 = a * v1.x() - b * v1.y();
	p2 = a * v2.x() - b * v2.y();
	if (p1 < p2) {
		min = p1;
		max = p2;
	} else {
		min = p2;
		max = p1;
	}
	rad = fa * boxhalfsize.x() + fb * boxhalfsize.y();
	if (min > rad || max < -rad)
		return false;
	return true;
}

inline bool axisTestZ0(float a, float b, float fa, float fb, const Eigen::Vector3d &v0,
	const Eigen::Vector3d &v1, const Eigen::Vector3d &boxhalfsize, float &rad, float &min,
	float &max, float &p0, float &p1) {
	p0 = a * v0.x() - b * v0.y();
	p1 = a * v1.x() - b * v1.y();
	if (p0 < p1) {
		min = p0;
		max = p1;
	} else {
		min = p1;
		max = p0;
	}
	rad = fa * boxhalfsize.x() + fb * boxhalfsize.y();
	if (min > rad || max < -rad)
		return false;
	return true;
}

bool triBoxOverlap(Eigen::Vector3d boxcenter, Eigen::Vector3d boxhalfsize, Eigen::Vector3d tv0, Eigen::Vector3d tv1,
	Eigen::Vector3d tv2) {
	/*    use separating axis theorem to test overlap between triangle and box */
	/*    need to test for overlap in these directions: */
	/*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
	/*       we do not even need to test these) */
	/*    2) normal of the triangle */
	/*    3) crossproduct(edge from tri, {x,y,z}-directin) */
	/*       this gives 3x3=9 more tests */
	Eigen::Vector3d v0, v1, v2;
	float min, max, p0, p1, p2, rad, fex, fey, fez;
	Eigen::Vector3d normal, e0, e1, e2;

	/* This is the fastest branch on Sun */
	/* move everything so that the boxcenter is in (0,0,0) */
	v0 = tv0 - boxcenter;
	v1 = tv1 - boxcenter;
	v2 = tv2 - boxcenter;

	/* compute triangle edges */
	e0 = v1 - v0;
	e1 = v2 - v1;
	e2 = v0 - v2;

	/* Bullet 3:  */
	/*  test the 9 tests first (this was faster) */
	fex = fabsf(e0.x());
	fey = fabsf(e0.y());
	fez = fabsf(e0.z());

	if (!axisTestX01(e0.z(), e0.y(), fez, fey, v0, v2, boxhalfsize, rad, min, max, p0, p2))
		return false;
	if (!axisTestY02(e0.z(), e0.x(), fez, fex, v0, v2, boxhalfsize, rad, min, max, p0, p2))
		return false;
	if (!axisTestZ12(e0.y(), e0.x(), fey, fex, v1, v2, boxhalfsize, rad, min, max, p1, p2))
		return false;

	fex = fabsf(e1.x());
	fey = fabsf(e1.y());
	fez = fabsf(e1.z());

	if (!axisTestX01(e1.z(), e1.y(), fez, fey, v0, v2, boxhalfsize, rad, min, max, p0, p2))
		return false;
	if (!axisTestY02(e1.z(), e1.x(), fez, fex, v0, v2, boxhalfsize, rad, min, max, p0, p2))
		return false;
	if (!axisTestZ0(e1.y(), e1.x(), fey, fex, v0, v1, boxhalfsize, rad, min, max, p0, p1))
		return false;

	fex = fabsf(e2.x());
	fey = fabsf(e2.y());
	fez = fabsf(e2.z());
	if (!axisTestX2(e2.z(), e2.y(), fez, fey, v0, v1, boxhalfsize, rad, min, max, p0, p1))
		return false;
	if (!axisTestY1(e2.z(), e2.x(), fez, fex, v0, v1, boxhalfsize, rad, min, max, p0, p1))
		return false;
	if (!axisTestZ12(e2.y(), e2.x(), fey, fex, v1, v2, boxhalfsize, rad, min, max, p1, p2))
		return false;

	/* Bullet 1: */
	/*  first test overlap in the {x,y,z}-directions */
	/*  find min, max of the triangle each direction, and test for overlap in */
	/*  that direction -- this is equivalent to testing a minimal AABB around */
	/*  the triangle against the AABB */

	/* test in X-direction */
	findMinMax(v0.x(), v1.x(), v2.x(), min, max);
	if (min > boxhalfsize.x() || max < -boxhalfsize.x())
		return false;

	/* test in Y-direction */
	findMinMax(v0.y(), v1.y(), v2.y(), min, max);
	if (min > boxhalfsize.y() || max < -boxhalfsize.y())
		return false;

	/* test in Z-direction */
	findMinMax(v0.z(), v1.z(), v2.z(), min, max);
	if (min > boxhalfsize.z() || max < -boxhalfsize.z())
		return false;

	/* Bullet 2: */
	/*  test if the box intersects the plane of the triangle */
	/*  compute plane equation of triangle: normal*x+d=0 */
	normal = e0.cross(e1);
	if (!planeBoxOverlap(normal, v0, boxhalfsize))
		return false;

	return true; /* box and triangle overlaps */
}