#include "BVH.h"
#include<queue>
#include<stack>
struct  BVH::BVHNode {
	//* todo BVH节点结构设计
	BVH::BVHNode* lson = nullptr;
	BVH::BVHNode* rson = nullptr;
	AABB aabb;
	std::vector<int> shapeIndex;
	int splitAxis;
};

BVH::~BVH()
{
	//clear bvh nodes
	std::queue<BVHNode*> nodes;
	nodes.push(root);
	do {
		BVHNode* rt = nodes.front();
		nodes.pop();
		if (rt) {
			nodes.push(rt->lson);
			nodes.push(rt->rson);
		}
		delete rt;
	} while (!nodes.empty());
}

void BVH::build() {
	for (const auto& shape : shapes) {
		//* 自行实现的加速结构请务必对每个shape调用该方法，以保证TriangleMesh构建内部加速结构
		//* 由于使用embree时，TriangleMesh::getAABB不会被调用，因此出于性能考虑我们不在TriangleMesh
		//* 的构造阶段计算其AABB，因此当我们将TriangleMesh的AABB计算放在TriangleMesh::initInternalAcceleration中
		//* 所以请确保在调用TriangleMesh::getAABB之前先调用TriangleMesh::initInternalAcceleration
		shape->initInternalAcceleration();
	}
	std::vector<int> index;
	index.reserve(shapes.size());
	for (int i = 0; i < shapes.size(); ++i) {
		index.emplace_back(i);
	}
	root = recursiveBuild(index);
}
bool BVH::rayIntersect(Ray& ray, int* geomID, int* primID, float* u, float* v) const {
	//* todo 完成BVH求交
	bool hit = false;
	std::stack<BVHNode*> nodes;
	nodes.push(root);
	while (!nodes.empty()) {
		BVHNode* rt = nodes.top();
		nodes.pop();
		if (rt->aabb.RayIntersect(ray)) {
			if (!rt->lson) {
				for (int index : rt->shapeIndex) {
					if (shapes[index]->rayIntersectShape(ray, primID, u, v)) {
						hit = true;
						*geomID = index;
					}
				}
			}
			else {
				if (ray.direction[rt->splitAxis] > 0) {
					nodes.push(rt->rson);
					nodes.push(rt->lson);
				}
				else {
					nodes.push(rt->lson);
					nodes.push(rt->rson);
				}
			}
		}
	}
	return hit;
}

BVH::BVHNode* BVH::recursiveBuild(const std::vector<int>& _shapeIndex)
{
	auto reorder_func = [&](int axis, int lhs, int rhs) {
		return shapes[lhs]->getAABB().Center()[axis] < shapes[rhs]->getAABB().Center()[axis];
		};

	BVHNode* node = new BVHNode{};
	node->shapeIndex = _shapeIndex;
	for (int index : _shapeIndex) {
		node->aabb.Expand(shapes[index]->getAABB());
	}
	if (_shapeIndex.size() > bvhLeafMaxSize) {
		std::vector<int> ordered_index = _shapeIndex;
		int splitAxis = 0;
		int max_extent = node->aabb.pMax[0] - node->aabb.pMin[0];
		for (int i = 1; i < 3; ++i) {
			int extent = node->aabb.pMax[i] - node->aabb.pMin[i];
			if (extent > max_extent) {
				max_extent = extent;
				splitAxis = i;
			}
		}
		node->splitAxis = splitAxis;
		sort(ordered_index.begin(), ordered_index.end(), std::bind(reorder_func, splitAxis, std::placeholders::_1, std::placeholders::_2));
		float midv = node->aabb.Center()[splitAxis];
		for (auto it = ordered_index.begin(); it != ordered_index.end(); ++it) {
			if (shapes[*it]->getAABB().Center()[splitAxis] > midv) {
				node->lson = recursiveBuild(std::vector<int>(ordered_index.begin(), it));
				node->rson = recursiveBuild(std::vector<int>(it, ordered_index.end()));
				break;
			}
		}
	}
	return node;
}


