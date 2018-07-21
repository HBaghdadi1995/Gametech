#include "PathFinder.h"

PathFinder::PathFinder(const std::string& friendly_name)
	: Scene(friendly_name)
	, grid_size(16)
	, density(0.5f)
	/*, astar_preset_idx(2)
	, astar_preset_text("")
	, search_df(new SearchDepthFirst())
	, search_bf(new SearchBreadthFirst())
	, search_as(new SearchAStar())*/
	, generator(new MazeGenerator())
{
	//wallmesh = new OBJMesh(MESHDIR"cube.obj");

	/*GLuint whitetex;
	glGenTextures(1, &whitetex);
	glBindTexture(GL_TEXTURE_2D, whitetex);
	unsigned int pixel = 0xFFFFFFFF;
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1, 1, 0, GL_RGB, GL_UNSIGNED_BYTE, &pixel);
	glBindTexture(GL_TEXTURE_2D, 0);

	wallmesh->SetTexture(whitetex);*/

	//srand(93225); //Set the maze seed to a nice consistent example :)
	srand(time(NULL));
}

PathFinder::~PathFinder() {

}

void PathFinder::GenerateNewMaze(int size, float d) {
	this->DeleteAllGameObjects(); //Cleanup old mazes

	grid_size = size;
	density = d;
	generator->Generate(grid_size, density);

	//The maze is returned in a [0,0,0] - [1,1,1] cube (with edge walls outside) regardless of grid_size,
	// so we need to scale it to whatever size we want
	Matrix4 maze_scalar = Matrix4::Scale(Vector3(5.f, 5.0f / float(grid_size), 5.f)) * Matrix4::Translation(Vector3(-0.5f, 0.f, -0.5f));

	/*mazes[0] = new MazeRenderer(generator, wallmesh);
	mazes[0]->Render()->SetTransform(Matrix4::Translation(pos_maze1) * maze_scalar);
	this->AddGameObject(mazes[0]);*/

	/*mazes[1] = new MazeRenderer(generator, wallmesh);
	mazes[1]->Render()->SetTransform(Matrix4::Translation(pos_maze2) * maze_scalar);
	this->AddGameObject(mazes[1]);*/

	/*mazes[2] = new MazeRenderer(generator, wallmesh);
	mazes[2]->Render()->SetTransform(Matrix4::Translation(pos_maze3) * maze_scalar);
	this->AddGameObject(mazes[2]);*/

	//Create Ground (..we still have some common ground to work off)
	/*GameObject* ground = CommonUtils::BuildCuboidObject(
	"Ground",
	Vector3(0.0f, -1.0f, 0.0f),
	Vector3(20.0f, 1.0f, 20.0f),
	false,
	0.0f,
	false,
	false,
	Vector4(0.2f, 0.5f, 1.0f, 1.0f));*/

	//this->AddGameObject(ground);

	GraphNode* start = generator->GetStartNode();
	GraphNode* end = generator->GetGoalNode();

	/*----Activate for A----*/

	//search_df->FindBestPath(start, end);
	//search_bf->FindBestPath(start, end);
	//UpdateAStarPreset();
}

void PathFinder::onUpdate(float dt) {
	//if (avatars.size() > 0) {
		string avatarC = "@avtrs";
		
		vector<Avatar*> deleteCandidates;

		for (vector<Avatar*>::iterator it = avatars.begin(); it != avatars.end(); it++) {
			(*it)->update(dt);
			if ((*it)->deletion) {
				deleteCandidates.push_back(*it);
			}
			avatarC += to_string((*it)->GetLocation().x) + "," + to_string((*it)->GetLocation().y) + "|";
		}
		for (vector<Avatar*>::iterator it = deleteCandidates.begin(); it != deleteCandidates.end(); it++) {
			avatars.erase(std::remove(avatars.begin(), avatars.end(), *it), avatars.end());
			delete *it;
		}
		//cout << avatarC << endl;
		avatarPacket = avatarC;
	//}
}

string PathFinder::GeneratePacket() {
	string m;
	m = "@mapps" + std::to_string(grid_size) + "_";
	//m.binary = "";
	for (int i = 0; i < (grid_size * (grid_size - 1)) * 2; ++i) {
	//for (int i = 0; i < (grid_size * (grid_size - 1)) * 2; ++i) {
		if (generator->allEdges[i]._iswall) {
			m += '1';
		}
		else {
			m += '0';
		}
	}
	/*m += "||";

	m += GetCoordinates(generator->start);
	m += "|";
	m += GetCoordinates(generator->end);
	m += "|";*/
	
	return m;
}

string PathFinder::GetCoordinates(GraphNode* node){
	string s = "";
	s += to_string((int)node->_pos.x);
	s += ",";
	s += to_string((int)node->_pos.y);
	return s;
}

void PathFinder::CalculatePath(string s) {
	int parser;
	string integers = "";
	for (int i = 6; s.at(i) != ','; i++) {
		integers += s.at(i);
		parser = i + 2;
	}
	int x = stoi(integers);
	integers = "";
	for (int i = parser; s.at(i) != '|'; i++) {
		integers += s.at(i);
		parser = i + 2;
	}
	int y = stoi(integers);
	generator->start = &generator->allNodes[y*grid_size + x];
	integers = "";
	//int parser = 1;
	for (int i = parser; s.at(i) != ','; i++) {
		integers += s.at(i);
		parser = i + 2;
	}
	x = stoi(integers);
	integers = "";
	for (int i = parser; s.at(i) != '|'; i++) {
		integers += s.at(i);
		parser = i + 2;
	}
	y = stoi(integers);
	generator->end = &generator->allNodes[y*grid_size + x];
}

string PathFinder::GetPathString() {
	//generator->end;
	
	//UpdateAStarPreset();
	UpdateAStarPreset();

	/*earch_as = new SearchAStar();

	search_as->SetWeightings(1, 1);

	GraphNode* start = generator->GetStartNode();
	GraphNode* end = generator->GetGoalNode();

	search_as->FindBestPath(start, end);*/

	string s = "@paths";
	s+= getFinalPathPacket(search_as->GetFinalPath());
	return s;
}

void PathFinder::UpdateAStarPreset()
{
	//Example presets taken from:
	// http://movingai.com/astar-var.html
	float weightingG, weightingH;
	switch (astar_preset_idx)
	{
	default:
	case 0:
		//Only distance from the start node matters - fans out from start node
		weightingG = 1.0f;
		weightingH = 0.0f;
		astar_preset_text = "Dijkstra - None heuristic search";
		break;
	case 1:
		//Only distance to the end node matters
		weightingG = 0.0f;
		weightingH = 1.0f;
		astar_preset_text = "Pure Hueristic search";
		break;
	case 2:
		//Equal weighting
		weightingG = 1.0f;
		weightingH = 1.0f;
		astar_preset_text = "Traditional A-Star";
		break;
	case 3:
		//Greedily goes towards the goal node where possible, but still cares about distance travelled a little bit
		weightingG = 1.0f;
		weightingH = 2.0f;
		astar_preset_text = "Weighted Greedy A-Star";
		break;
	}
	search_as->SetWeightings(weightingG, weightingH);

	GraphNode* start = generator->GetStartNode();
	GraphNode* end = generator->GetGoalNode();
	delete search_as;
	search_as = new SearchAStar();
	search_as->FindBestPath(start, end);
}

string PathFinder::getFinalPathPacket(std::list<const GraphNode*> finalPath) {
	string s = "";

	for (std::list<const GraphNode*>::iterator it = finalPath.begin(); it != finalPath.end(); it++) {
		s += "(" + to_string((int)(*it)->_pos.x) + "," + to_string((int)(*it)->_pos.y) + ")";
	}
	s += "@@end";
	return s;
}

void PathFinder::AddAvatar(string s) {
	//SearchHistory searchHistory;
	//searchHistory.clear();
	vector<GraphNode*> path;
	int i = 7;
	while (s.at(i) != '@') {
		//Vector3 point = Vector3(0, 0, 0);
		string integers = "";
		for (int j = i; s.at(j) != '(' && s.at(j) != ',' && s.at(j) != ')'; j++) {
			integers += s.at(j);
			i = j + 2;
		}
		int x = stoi(integers);
		integers = "";
		for (int j = i; s.at(j) != '(' && s.at(j) != ',' && s.at(j) != ')'; j++) {
			integers += s.at(j);
			i = j + 3;
		}
		int y = stoi(integers);
		integers = "";

		path.push_back(generator->getNode(x, y));
	}

	/*for (vector<GraphNode*>::iterator it = path.begin(); it != path.end() - 1; it++) {
		//SearchHistoryElement se (*(it + 1), *it);

		searchHistory.push_back({ *(it + 1), *it });
	}*/

	Avatar* a = new Avatar(path);
	avatars.push_back(a);
}