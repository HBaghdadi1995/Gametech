#pragma once
#include "SearchAlgorithm.h"
#include <vector>

using namespace std;

class Avatar {
public:
	Avatar(vector<GraphNode*> p) {
		path = p;
	}
	~Avatar() {
		for (vector<GraphNode*>::iterator it = path.begin(); it != path.end(); it++) {
			//delete (*it);
		}
	}

	void update(float dt) {
		if (nextPoint == 0) {
			location = (*path.begin())->_pos;
			nextPoint = 1;
		}
		else if (nextPoint == path.size()) {
			deletion = true;
		}
		else {
			float distGone = 1 * dt;
			Vector3 hi = ((*(path.begin() + nextPoint))->_pos - location);
			if (hi.Length() < distGone) {
				location = (*(path.begin() + nextPoint))->_pos;
				nextPoint++;
			}
			else {
				location = location + ((*(path.begin() + nextPoint))->_pos - location).Normalise() * distGone;
			}
			/*location = location + ((*(path.begin() + nextPoint))->_pos - location).Normalise() * distGone;
			if (abs((location - (*(path.begin() + nextPoint))->_pos).x < 0.1) && 
				abs((location - (*(path.begin() + nextPoint))->_pos).y < 0.1)) {
				nextPoint++;
			}*/
		}
	}

	Vector3 GetLocation() { return location; }

	bool deletion = false;

protected:
	vector<GraphNode*> path;
	int nextPoint = 0;
	Vector3 location;
};