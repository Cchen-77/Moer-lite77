#include "Disk.h"
#include "ResourceLayer/Factory.h"
bool Disk::rayIntersectShape(Ray& ray, int* primID, float* u, float* v) const {

	//* todo 完成光线与圆环的相交 填充primId,u,v.如果相交，更新光线的tFar
	//* 1.光线变换到局部空间
	//* 2.判断局部光线的方向在z轴分量是否为0
	//* 3.计算光线和平面交点
	//* 4.检验交点是否在圆环内
	//* 5.更新ray的tFar,减少光线和其他物体的相交计算次数
	//* Write your code here.
	auto inv_ray = transform.inverseRay(ray);
	Point3f origin = inv_ray.origin;
	Vector3f direction = inv_ray.direction;
	//direciton.z is Nearly zero
	if (std::abs(direction[2]) < 1e-8) {
		return false;
	}
	float t = (0 - origin[2]) / direction[2];
	if (t <= 0) {
		return false;
	}
	Point3f hit_point = origin + direction * t;
	float hit_radius = Vector2f(hit_point[0], hit_point[1]).len();
	if (hit_radius<innerRadius || hit_radius > radius) {
		return false;
	}
	bool hit = false;
	Vector2f hit_vec = Vector2f(hit_point[0], hit_point[1]) / hit_radius;
	float cos = hit_vec.dot(Vector2f(1, 0));
	float phi = fm::acos(cos);
	if (hit_vec[1] < 0) {
		phi = 2 * PI - phi;
	}
	if (phi < phiMax) {
		*primID = direction[2] > 0 ? 1 : 0;
		*u = phi / phiMax;
		*v = (hit_radius - innerRadius) / (radius - innerRadius);
		ray.tFar = t;
		return true;
	}
	return false;
}

void Disk::fillIntersection(float distance, int primID, float u, float v, Intersection* intersection) const {
	//* todo 填充圆环相交信息中的法线以及相交位置信息
	//* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
	//* 2.位置信息可以根据uv计算出，同样需要变换
	//* Write your code here.
	//disk is not two-sided?confused?
	intersection->normal = transform.toWorld(Vector3f(0, 0, 1));
	intersection->position = transform.toWorld(Point3f((v * (radius - innerRadius) + innerRadius) * fm::cos(u * phiMax), (v * (radius - innerRadius) + innerRadius) * fm::sin(u * phiMax), 0));

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

Disk::Disk(const Json& json) : Shape(json) {
	//    normal = transform.toWorld(Vector3f(0,0,1));
	//    origin = transform.toWorld(Point3f(0,0,0));
	//    auto
	//    //radius认为是三个方向的上的scale平均
	//    vecmat::vec4f v(1,1,1,0);
	//    auto radiusVec = transform.scale * v;
	//    radiusVec/=radiusVec[3];
	//    radius = (radiusVec[0]+radiusVec[1]+radiusVec[2])/3;
	radius = fetchOptional(json, "radius", 1.f);
	innerRadius = fetchOptional(json, "inner_radius", 0.f);
	phiMax = fetchOptional(json, "phi_max", 2 * PI);
	AABB local(Point3f(-radius, -radius, 0), Point3f(radius, radius, 0));
	boundingBox = transform.toWorld(local);
}

void Disk::uniformSampleOnSurface(Vector2f sample, Intersection* result, float* pdf) const {
	//采样光源 暂时不用实现
}
REGISTER_CLASS(Disk, "disk")

