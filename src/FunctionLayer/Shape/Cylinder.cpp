#include "Cylinder.h"
#include "ResourceLayer/Factory.h"
bool Cylinder::rayIntersectShape(Ray& ray, int* primID, float* u, float* v) const {
	//* todo 完成光线与圆柱的相交 填充primId,u,v.如果相交，更新光线的tFar
	//* 1.光线变换到局部空间
	//* 2.联立方程求解
	//* 3.检验交点是否在圆柱范围内
	//* 4.更新ray的tFar,减少光线和其他物体的相交计算次数
	//* Write your code here.
	auto inv_ray = transform.inverseRay(ray);
	Point3f origin = inv_ray.origin;
	Vector3f direction = inv_ray.direction;
	float t0, t1;
	float A = direction[0] * direction[0] + direction[1] * direction[1];
	float B = 2 * (direction[0] * origin[0] + direction[1] * origin[1]);
	float C = origin[0] * origin[0] + origin[1] * origin[1] - radius * radius;
	bool succ = Quadratic(A, B, C, &t0, &t1);
	if (!succ) {
		return false;
	}
	if (t0<ray.tNear || t1 > ray.tFar) {
		return false;
	}
	auto check_hitpoint = [&](const Point3f& hit_point, int* primID, float* u, float* v)->bool {
		if (hit_point[2] < 0 || hit_point[2] > height) {
			return false;
		}
		float cos = (Vector2f(hit_point[0], hit_point[1]) / radius).dot(Vector2f(1, 0));
		float phi = fm::acos(cos);
		if (hit_point[1] < 0) {
			phi = 2 * PI - phi;
		}
		if (phi < phiMax) {
			*primID = 0;
			*u = phi / phiMax;
			*v = hit_point[2] / height;
			return true;
		}
		return false;
		};
	if (check_hitpoint(origin + t0 * direction, primID, u, v)) {
		return true;
	}
	if (check_hitpoint(origin + t1 * direction, primID, u, v)) {
		return true;
	}
	return false;
}

void Cylinder::fillIntersection(float distance, int primID, float u, float v, Intersection* intersection) const {
	/// ----------------------------------------------------
	//* todo 填充圆柱相交信息中的法线以及相交位置信息
	//* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
	//* 2.位置信息可以根据uv计算出，同样需要变换
	//* Write your code here.
	/// ----------------------------------------------------

	float cos = fm::cos(phiMax * u);
	float sin = fm::sin(phiMax * u);
	intersection->normal = normalize(transform.toWorld(Vector3f(cos, sin, 0)));
	intersection->position = transform.toWorld(Point3f(cos * radius, sin * radius, v * height));

	intersection->shape = this;
	intersection->distance = distance;
	intersection->texCoord = Vector2f{ u, v };
	Vector3f tangent{ 1.f, 0.f, .0f };
	Vector3f bitangent;
	if (std::abs(dot(tangent, intersection->normal)) > .9f) {
		tangent = Vector3f(.0f, 1.f, .0f);
	}
	bitangent = normalize(cross(tangent, intersection->normal));
	tangent = normalize(cross(intersection->normal, bitangent));
	intersection->tangent = tangent;
	intersection->bitangent = bitangent;
}

void Cylinder::uniformSampleOnSurface(Vector2f sample, Intersection* result, float* pdf) const {

}

Cylinder::Cylinder(const Json& json) : Shape(json) {
	radius = fetchOptional(json, "radius", 1.f);
	height = fetchOptional(json, "height", 1.f);
	phiMax = fetchOptional(json, "phi_max", 2 * PI);
	AABB localAABB = AABB(Point3f(-radius, -radius, 0), Point3f(radius, radius, height));
	boundingBox = transform.toWorld(localAABB);
}

REGISTER_CLASS(Cylinder, "cylinder")
