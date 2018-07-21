#include "Octree.h"

Octree::Octree(Vector3 origin, Vector3 range) {
	this->origin = origin;
	this->range = range;
}

Octree::~Octree() {
	ClearChildren();
	ClearObjects();
}

void Octree::ClearChildren() {
	if (childrenExist) {
		for (std::vector<PhysicsNode*>::iterator it = objects.begin();it != objects.end();it++) {
			delete *it;
		}
		children.clear();
	}
}

void Octree::ClearObjects() {
	if (parentExist) {
		parent->AddObjects(objects);
	}
	objects.clear();
}

void Octree::CreateChildren() {
	std::map<Octree*, std::vector<PhysicsNode*>> sendMap;
	if (!childrenExist) {

		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 2; j++) {
				for (int k = 0; k < 2; k++) {
					Octree* child = new Octree(origin + Vector3(range.x * 0.5 * i,range.y * 0.5f * j,range.z * 0.5f * k), range/2.0f);
					child->AddParent(this);
					child->parentExist = true;
					children.push_back(child);
				}
			}
		}
		childrenExist = true;
	}
	for (std::vector<PhysicsNode*>::iterator it = objects.begin();it != objects.end();it++) {
		Vector3 position = (*it)->GetPosition();
		for (std::vector<Octree*>::iterator jt = children.begin(); jt != children.end(); jt++) {
			Vector3 origin = (*jt)->getOrigin();
			Vector3 range = (*jt)->getRange();
			vector<PhysicsNode*> xlx;
			sendMap.insert(std::pair<Octree*, std::vector<PhysicsNode*>>(*jt, xlx));
			if (position.x > origin.x &&
				position.x < origin.x + range.x &&
				position.y > origin.y &&
				position.y < origin.y + range.y &&
				position.z > origin.z &&
				position.z < origin.z + range.z) {
				float radius = (*it)->GetCollisionShape()->GetRadius();
				if ((position.x - radius > origin.x &&
					position.x + radius < origin.x + range.x&&
					position.y - radius > origin.y &&
					position.y + radius < origin.y + range.y &&
					position.z - radius > origin.z &&
					position.z + radius < origin.z + range.z
					)) {
					sendMap.at(*jt).push_back(*it);
				}
			}
		}
	}
	for (std::vector<Octree*>::iterator it = children.begin(); it != children.end(); it++) {
		(*it)->AddObjects(sendMap.at(*it));
		for (std::vector<PhysicsNode*>::iterator jt = sendMap.at(*it).begin(); jt != sendMap.at(*it).end(); jt++) {
			objects.erase(std::remove(objects.begin(), objects.end(), (*jt)), objects.end());
		}
	}
	//cout << endl;
}

void Octree::AddObjects(std::vector<PhysicsNode*> p) {
	for (std::vector<PhysicsNode*>::iterator it = p.begin();it != p.end();it++) {
		objects.push_back(*it);
	}
	if (objects.size() > MAXOBJECTS) CreateChildren();
}

void Octree::SetChildrenObjects() {
	childrenObjects.clear();
	if (childrenExist) {
		for (std::vector<Octree*>::iterator it = children.begin(); it != children.end(); it++)
		{
			(*it)->SetChildrenObjects();
			std::vector<PhysicsNode*> *childrenObject = (*it)->GetChildrenObjects();
			childrenObjects.insert(childrenObjects.end(), childrenObject->begin(), childrenObject->end());
			std::vector<PhysicsNode*> childrensObjects = (*it)->GetObjects();
			childrenObjects.insert(childrenObjects.end(), childrensObjects.begin(), childrensObjects.end());
		}
	}
}

void Octree::DebugDraw() {

	NCLDebug::DrawThickLineNDT(origin, Vector3(origin.x, origin.y, range.z), 0.02f, Vector4(1.0f, 0.3f, 1.0f, 1.0f));
	NCLDebug::DrawThickLineNDT(origin, Vector3(origin.x, range.y, origin.z), 0.02f, Vector4(1.0f, 0.3f, 1.0f, 1.0f));
	NCLDebug::DrawThickLineNDT(origin, Vector3(range.x, origin.y, origin.z), 0.02f, Vector4(1.0f, 0.3f, 1.0f, 1.0f));

	NCLDebug::DrawThickLineNDT(Vector3(origin.x, origin.y, range.z), Vector3(origin.x, range.y, range.z), 0.02f, Vector4(1.0f, 0.3f, 1.0f, 1.0f));
	NCLDebug::DrawThickLineNDT(Vector3(origin.x, origin.y, range.z), Vector3(range.x, origin.y, range.z), 0.02f, Vector4(1.0f, 0.3f, 1.0f, 1.0f));

	NCLDebug::DrawThickLineNDT(Vector3(origin.x, range.y, origin.z), Vector3(origin.x, range.y, range.z), 0.02f, Vector4(1.0f, 0.3f, 1.0f, 1.0f));
	NCLDebug::DrawThickLineNDT(Vector3(origin.x, range.y, origin.z), Vector3(range.x, range.y, origin.z), 0.02f, Vector4(1.0f, 0.3f, 1.0f, 1.0f));

	NCLDebug::DrawThickLineNDT(Vector3(range.x, origin.y, origin.z), Vector3(range.x, origin.y, range.z), 0.02f, Vector4(1.0f, 0.3f, 1.0f, 1.0f));
	NCLDebug::DrawThickLineNDT(Vector3(range.x, origin.y, origin.z), Vector3(range.x, range.y, origin.z), 0.02f, Vector4(1.0f, 0.3f, 1.0f, 1.0f));

	NCLDebug::DrawThickLineNDT(range, Vector3(range.x, range.y, origin.z), 0.02f, Vector4(1.0f, 0.3f, 1.0f, 1.0f));
	NCLDebug::DrawThickLineNDT(range, Vector3(range.x, origin.y, range.z), 0.02f, Vector4(1.0f, 0.3f, 1.0f, 1.0f));
	NCLDebug::DrawThickLineNDT(range, Vector3(origin.x, range.y, range.z), 0.02f, Vector4(1.0f, 0.3f, 1.0f, 1.0f));
}

void Octree::SendUp() {
	if (parentExist) {
		vector<PhysicsNode*> sendMap;
		for (std::vector<PhysicsNode*>::iterator it = objects.begin();it != objects.end();it++) {
			Vector3 position = (*it)->GetPosition();

			float radius = (*it)->GetCollisionShape()->GetRadius();
			if (!(position.x - radius > origin.x &&
				position.x + radius < origin.x + range.x&&
				position.y - radius > origin.y &&
				position.y + radius < origin.y + range.y &&
				position.z - radius > origin.z &&
				position.z + radius < origin.z + range.z
				)) {
				sendMap.push_back(*it);
			}
		}
		parent->AddObjects(sendMap);
		for (std::vector<PhysicsNode*>::iterator jt = sendMap.begin(); jt != sendMap.end(); jt++) {
			objects.erase(std::remove(objects.begin(), objects.end(), (*jt)), objects.end());
		}
	}
	for (std::vector<Octree*>::iterator it = children.begin(); it != children.end(); it++) {
		(*it)->SendUp();
	}
}