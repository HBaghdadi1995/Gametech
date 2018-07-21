#pragma once

//This is almost identicle to the SearchAStar class in Tuts_GameLogic.
// The only difference is the data it traverses and the distance
// check used in the DistanceHueristic function.

#include "NavMesh.h"
#include <nclgl\common.h>
#include <nclgl\Vector3.h>
#include <deque>
#include <algorithm>
#include <map>
#include <list>

typedef std::pair<NavVertex*, NavVertex*> SearchHistoryElement;
typedef std::list<SearchHistoryElement> SearchHistory;
typedef std::map<NavVertex*, NavVertex*> ParentMap;
typedef std::map<NavVertex*, float> gmap;
typedef std::pair<NavVertex*, float> gkeyvalue;

class Nav_SearchAStar
{
public:
	Nav_SearchAStar()
		: g_weighting(1.0f)
		, h_weighting(10.0f)
	{
	}
	~Nav_SearchAStar() {}

	void SetWeightings(float g, float h)
	{
		g_weighting = g;
		h_weighting = h;
	}

	float GetWeighting_G() const { return g_weighting; }
	float GetWeighting_H() const { return h_weighting; }


	void ClearPath()
	{
		searchHistory.clear();
		finalPath.clear();

		open_set.clear();
		closed_set.clear();
		came_from.clear();
		g_score.clear();
		f_score.clear();
	}

	float DistanceHueristic(const NavVertex* node, const Vector3& goal)
	{
		//Two different distance measurements (Try both!!)
		Vector3 diff = goal - node->_pos;

		return diff.Length();									//True omni directional system  (Euclidean distance)
		//return fabs(diff.x) + fabs(diff.y) + fabs(diff.z);	//Grid based system				(Manhatten distance)
	};

	NavVertex* GetNearestVertex(const NavTri* tri, const Vector3& point)
	{
		float best_dist = FLT_MAX;
		uint best_idx = 0;
		for (uint i = 0; i < 3; ++i)
		{
			float dist = (tri->_verts[i]->_pos - point).Length();
			if (dist < best_dist)
			{
				best_dist = dist;
				best_idx = i;
			}
		}
		return tri->_verts[best_idx];
	}

	bool ContainsVertex(const NavTri* tri, const NavVertex* vert)
	{
		return tri->_a == vert || tri->_b == vert || tri->_c == vert;
	}

	bool FindBestPath(const NavTri* start_tri, const Vector3& start_pos, const NavTri* goal_tri, const Vector3& goal_pos)
	{
		ClearPath();

		//Do both triangles exist?
		if (!start_tri || !goal_tri)
			return false;

		//Are both start and goal points in the same triangle?
		if (start_tri == goal_tri)
			return true;


		goal = goal_pos;
		start = start_pos;

		NavVertex* first = GetNearestVertex(start_tri, start);


		//https://en.wikipedia.org/wiki/A*_search_algorithm
		g_score[first] = 0.0f;
		f_score[first] = DistanceHueristic(first, goal);
		open_set.push_back(first);

		NavVertex *current = NULL;
		while (open_set.size() > 0)
		{
			//Find the node in the open_set with the lowest f_score
			auto min = min_element(open_set.begin(), open_set.end(), [&](NavVertex* left, NavVertex* right) {
				return f_score.at(left) < f_score.at(right);
			});

			current = *min;
			open_set.erase(min);

			if (came_from[current] != NULL) searchHistory.push_back({ current, came_from[current] });

			//Have we made it to the goal yet?
			if (ContainsVertex(goal_tri, current))
			{
				BuildPathFromParentMap(first, current, came_from);
				
				return true;
			}

			//Go down the node with the lowest f_score and explore all it's adjacent neighbours
			for (NavVertexTriLookup& ref : current->_tris)
			{
				//Each triangle that is attached to this vertex will have 2 edges we can traverse
				NavVertex* edge_end1 = ref._parent->_verts[(ref._parentVertIdx + 1) % 3];
				NavVertex* edge_end2 = ref._parent->_verts[(ref._parentVertIdx + 2) % 3];
				
				TraverseVertex(current, edge_end1);
				TraverseVertex(current, edge_end2);
			}

			closed_set.push_back(current);
		}

		return false;
	}


	void TraverseVertex(NavVertex* current, NavVertex* vertex)
	{
		//Make sure we haven't visted the neighbor already
		if (std::find(closed_set.begin(), closed_set.end(), vertex) != closed_set.end())
			return;

		//Make sure we haven't already listed the neighbour previously
		if (std::find(open_set.begin(), open_set.end(), vertex) == open_set.end())
		{
			open_set.push_back(vertex);
		}

		//Distance from start to neighbour node
		float gscore = g_score[current] + (current->_pos - vertex->_pos).Length();

		//Check to see if the node has already been visited previously (and if our new path to it is more expensive)
		if (g_score.find(vertex) != g_score.end() && gscore >= g_score[vertex])
			return; //More expensive route found - just leave it with the previous routes f_score

		came_from[vertex] = current;
		g_score[vertex] = gscore;

		// Different weightings have vastly different results, the default 1x1 is the traditional A* algorithm
		f_score[vertex] = gscore * g_weighting + DistanceHueristic(vertex, goal) * h_weighting;
	}

	const SearchHistory&		 GetSearchHistory()	const { return searchHistory; }
	const std::list<NavVertex*>& GetFinalPath()		const { return finalPath; }

protected:
	void BuildPathFromParentMap(NavVertex* start, NavVertex* goal, const ParentMap& map)
	{
		//To build a final path we can keep traversing back to the start node from the goal
		NavVertex* current = goal;

		while (current != NULL)
		{
			finalPath.push_front(current);
			current = map.at(current);
		}
	}

protected:
	float g_weighting;
	float h_weighting;

	Vector3 start, goal;

	SearchHistory			  searchHistory;
	std::list<NavVertex*>     finalPath;

	std::list<NavVertex*> open_set;
	std::list<NavVertex*> closed_set;

	typedef std::map<NavVertex*, float> gmap;
	typedef std::pair<NavVertex*, float> gkeyvalue;

	ParentMap came_from;
	gmap g_score;
	gmap f_score;
};