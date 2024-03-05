#include "Cone.h"
#include "ResourceLayer/Factory.h"

bool Cone::rayIntersectShape(Ray& ray, int* primID, float* u, float* v) const {
	//* todo 完成光线与圆柱的相交 填充primId,u,v.如果相交，更新光线的tFar
	//* 1.光线变换到局部空间
	//* 2.联立方程求解
	//* 3.检验交点是否在圆锥范围内
	//* 4.更新ray的tFar,减少光线和其他物体的相交计算次数
	//* Write your code here.
	auto inv_ray = transform.inverseRay(ray);
	Point3f origin = inv_ray.origin;
	Vector3f direction = inv_ray.direction;
	Vector3f vec_CO = origin - Point3f(0, 0, height);
	Vector3f vec_V = Vector3f(0, 0, -1);

	float t0, t1;
	float A = dot(direction, vec_V);
	A = A * A - cosTheta * cosTheta;
	float B = 2 * (dot(direction, vec_V) * dot(vec_CO, vec_V) - dot(direction, vec_CO) * cosTheta * cosTheta);
	float C = dot(vec_CO, vec_V);
	C = C * C - dot(vec_CO, vec_CO) * cosTheta * cosTheta;

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
		float cos = dot(normalize(Vector3f(hit_point[0], hit_point[1], 0)), Vector3f(1, 0, 0));
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

void Cone::fillIntersection(float distance, int primID, float u, float v, Intersection* intersection) const {
	/// ----------------------------------------------------
	//* todo 填充圆锥相交信息中的法线以及相交位置信息
	//* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
	//* 2.位置信息可以根据uv计算出，同样需要变换
	//* Write your code here.
	/// ----------------------------------------------------

	float hit_radius = radius * (1 - v);
	float cos = fm::cos(u * phiMax);
	float sin = fm::sin(u * phiMax);
	intersection->position = transform.toWorld(Point3f(hit_radius * cos, hit_radius * sin, v * height));
	intersection->normal = normalize(transform.toWorld(Vector3f(Point3f(hit_radius * cos, hit_radius * sin, v * height) - Point3f(0, 0, v * height - (hit_radius * hit_radius) / (height - v * height)))));

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

void Cone::uniformSampleOnSurface(Vector2f sample, Intersection* result, float* pdf) const {

}

Cone::Cone(const Json& json) : Shape(json) {
	radius = fetchOptional(json, "radius", 1.f);
	height = fetchOptional(json, "height", 1.f);
	phiMax = fetchOptional(json, "phi_max", 2 * PI);
	float tanTheta = radius / height;
	cosTheta = sqrt(1 / (1 + tanTheta * tanTheta));
	//theta = fetchOptional(json,)
	AABB localAABB = AABB(Point3f(-radius, -radius, 0), Point3f(radius, radius, height));
	boundingBox = transform.toWorld(localAABB);
	boundingBox = AABB(Point3f(-100, -100, -100), Point3f(100, 100, 100));
}

REGISTER_CLASS(Cone, "cone")
