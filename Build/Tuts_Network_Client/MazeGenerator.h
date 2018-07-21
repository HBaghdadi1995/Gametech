#pragma once
#include <ncltech\GameObject.h>
#include <ncltech\Scene.h>
#include "SearchAlgorithm.h"

struct MazePacket {
	int grid_size;
	string binary;
};


class MazeGenerator
{
public:
	MazeGenerator(); //Maze_density goes from 1 (single path to exit) to 0 (no walls at all)
	virtual ~MazeGenerator();

	void Generate(int size, float maze_density);

	//All points on the maze grid are connected in some shape or form
	// so any two nodes picked randomly /will/ have a path between them
	GraphNode* GetStartNode() const { return start; }
	GraphNode* GetGoalNode()  const { return end; }
	uint GetSize() const { return size; }


	//Used as a hack for the MazeRenderer to generate the walls more effeciently
	GraphNode* GetAllNodesArr() { return allNodes; }

	void interpret(string);

	string CreateCoordPacket();

	GraphNode *getNode(int x, int y) {
		return &allNodes[y * size + x];
	}

protected:
	void GetRandomStartEndNodes();

	void Initiate_Arrays();

	void Generate_Prims();
	void Generate_Sparse(float density);
	string GetCoordinates(GraphNode*);
public:
	uint size;
	GraphNode *start, *end;

	GraphNode* allNodes;
	GraphEdge* allEdges;
};