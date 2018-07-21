
#pragma once

#include <ncltech\Scene.h>
#include <ncltech\NetworkBase.h>
#include "MazeGenerator.h"
#include "MazeRenderer.h"
#include <nclgl\OBJMesh.h>

//Basic Network Example

class Net1_Client : public Scene
{
public:
	Net1_Client(const std::string& friendly_name);

	virtual void OnInitializeScene() override;
	virtual void OnCleanupScene() override;
	virtual void OnUpdateScene(float dt) override;


	void ProcessNetworkEvent(const ENetEvent& evnt);

	void GenerateNewMaze();

	void SetGridSize(int i) { grid_size = i; }

	void InterpretPath(string);

protected:
	Mesh* wallmesh;

	//GameObject* box;

	NetworkBase network;
	ENetPeer*	serverConnection;

	MazeGenerator* mazeGenerator;

	MazeRenderer*	maze;

	void ResetScene();
	void SendCoord();
	void SendPath();

	int grid_size = 16;

	SearchHistory searchHistory;

	bool showPath = false;

	string path;

	void SetAvatars(string);
	vector<Vector3> avatars;

	void DrawAvatars();

	GameObject** cubeAvatars = new GameObject*[10];

	void SendSizeSmaller();
	void SendSizeBigger();
};