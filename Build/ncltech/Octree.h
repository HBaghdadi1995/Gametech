#pragma once

#include <nclgl\Vector4.h>
#include <vector>
#include <ncltech\PhysicsNode.h>
#include <map>
#include <algorithm>
#include <nclgl\NCLDebug.h>

#define MAXOBJECTS 5

class Octree {
public:
	Octree(Vector3, Vector3);
	virtual ~Octree();

	void ClearChildren();
	void ClearObjects();
	void CreateChildren();

	Vector3 getOrigin() { return origin; }
	Vector3 getRange() { return range; }

	void AddParent(Octree* parent) { this->parent = parent; }

	void AddObjects(std::vector<PhysicsNode*> p);
	void AddObject(PhysicsNode* p) {
		objects.push_back(p);
		CreateChildren();
	}
	void SetChildrenObjects();

	std::vector<PhysicsNode*> GetObjects() { return objects; }
	std::vector<PhysicsNode*> *GetChildrenObjects() { return &childrenObjects; }
	std::vector<Octree*> GetChildren() { return children; }

	bool HasChildren() { return childrenExist; }

	void DebugDraw();
	void SendUp();
private:


	/*
	   3--------7
	  /|       /|
	 / |      / |
	2--------6  |
	|  |     |  |
	|  1-----|--5
	| /      | /
	|/       |/
	0--------4

	*/

	std::vector<Octree*> children;
	std::vector<PhysicsNode*> objects;
	std::vector<PhysicsNode*> childrenObjects;

	Octree* parent;

	bool childrenExist = false;
	bool parentExist = false;

	Vector3 origin;
	Vector3 range;

};