#pragma once
#include "Acceleration.h"
class BVH : public Acceleration {
public:
	BVH() = default;
	~BVH();
	void build() override;
	bool rayIntersect(Ray& ray, int* geomID, int* primID, float* u, float* v) const override;
protected:
	static constexpr int bvhLeafMaxSize = 64;
	struct BVHNode;
	BVHNode* root = nullptr;
protected:
	BVHNode* recursiveBuild(const std::vector<int>& _shapeIndex);
};