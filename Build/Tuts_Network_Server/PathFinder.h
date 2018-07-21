#pragma once

#include "MazeGenerator.h"
#include "SearchAStar.h"
#include "Avatar.h"

struct MazePacket {
	int grid_size;
	string binary;
};

class PathFinder : public Scene {
public:
	PathFinder(const std::string& friendly_name);
	~PathFinder();

	void GenerateNewMaze(int, float);
	void onUpdate(float);

	string GeneratePacket();

	void CalculatePath(string s);

	string GetPathString();

	void UpdateAStarPreset();

	void AddAvatar(string s);

	vector<Avatar*> avatars;

	string avatarPacket;

protected:

	int astar_preset_idx = 2;
	std::string astar_preset_text;

	SearchAStar*		search_as = new SearchAStar();

	string GetCoordinates(GraphNode* node);

	MazeGenerator *generator;

	int grid_size;
	float density;

	string getFinalPathPacket(std::list<const GraphNode*>);
};