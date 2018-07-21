#pragma once
#include <ncltech\Scene.h>
#include <ncltech\CommonUtils.h>
#include <ncltech\CommonMeshes.h>
#include "NavMesh.h"
#include "SimpleMeshLoader.h"
#include "Nav_SearchAStar.h"

class Scene_NavMesh : public Scene
{
public:
	Scene_NavMesh(const std::string& friendly_name)
		: Scene(friendly_name)
	{
	}

	virtual ~Scene_NavMesh()
	{
	}

	virtual void OnInitializeScene() override
	{
		start = CommonUtils::BuildCuboidObject(
			"",
			Vector3(8, 3, 5),
			Vector3(0.1f, 0.5f, 0.1f),
			false,
			0.f,
			false,
			true,
			CommonUtils::GenColor(0.5f));
		this->AddGameObject(start);

		end = CommonUtils::BuildCuboidObject(
			"",
			Vector3(2, 5.25, -3),
			Vector3(0.1f, 0.5f, 0.1f),
			false,
			0.f,
			false,
			true,
			CommonUtils::GenColor(0.0f));
		this->AddGameObject(end);


		//Because the navmesh is normal sized, and our camera range is tiny, for this
		// example we'll just shrink the world down to 1/3rd scale.
		Matrix4 transform = Matrix4::Translation(Vector3(0, 5, -15)) * Matrix4::Scale(Vector3(0.3f, 0.3f, 0.3f));

		Mesh* world_mesh = SimpleMeshLoader::LoadMeshFile("originalMesh.txt", transform);
		world_mesh->SetTexture(CommonMeshes::CheckerboardTex());
		this->AddGameObject(new GameObject("", new RenderNode(world_mesh), NULL));

		navMesh = new NavMesh("NavMesh.txt", transform);
	}

	virtual void OnCleanupScene() override
	{
		Scene::OnCleanupScene();
		delete navMesh;
	}

	virtual void OnUpdateScene(float dt) override
	{
		NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "--- Info ---");
		NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "    To make the transition from 2D mazes to dynamic worlds");
		NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "    we have to start using navigation meshes. ");
		NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "");
		NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "    Try moving the start and goal markers around the map to see");
		NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "    how the A-Star algorithm paths around.");
		NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "");
		NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "    The path is currently restricted to the edges of the triangles, ");
		NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "    but it is still correct. To polish the path, and stop it zig-zagging");
		NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "    along the edges, a funnel/string-pulling algorithm will have to be");
		NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "    implemented.");



		Scene::OnUpdateScene(dt);
		navMesh->DebugDraw();

		start_pos = start->Render()->GetWorldTransform().GetPositionVector();
		goal_pos = end->Render()->GetWorldTransform().GetPositionVector();

		NavTri* new_tri_start = navMesh->GetFirstTriThatIntersectsLine(start_pos + Vector3(0, 0.5f, 0), start_pos - Vector3(0, 0.5f, 0), start_pos);
		NavTri* new_tri_goal = navMesh->GetFirstTriThatIntersectsLine(goal_pos + Vector3(0, 0.5f, 0), goal_pos - Vector3(0, 0.5f, 0), goal_pos);
		
		//Try not to do the costly search if we don't have to
		bool repathing_required = false;
		if (new_tri_start != tri_start)
		{
			tri_start = new_tri_start;
			repathing_required = true;
		}
		if (new_tri_goal != tri_goal)
		{
			tri_goal = new_tri_goal;
			repathing_required = true;
		}

		if (repathing_required)
			pathValid = pathFinder.FindBestPath(tri_start, start_pos, tri_goal, goal_pos);






		//Draw start/goal tri's and best path
		const Vector4 tri_color_start = Vector4(0.6f, 1.f, 0.6f, 1.0f);
		const Vector4 tri_color_goal = Vector4(1.0f, 0.6f, 0.6f, 1.0f);
		if (tri_goal)
		{
			NCLDebug::DrawTriangle(tri_goal->_a->_pos, tri_goal->_b->_pos, tri_goal->_c->_pos, tri_color_goal);
			NCLDebug::DrawTriangle(tri_goal->_b->_pos, tri_goal->_a->_pos, tri_goal->_c->_pos, tri_color_goal);
		}
		if (tri_start)
		{
			NCLDebug::DrawTriangle(tri_start->_a->_pos, tri_start->_b->_pos, tri_start->_c->_pos, tri_color_start);
			NCLDebug::DrawTriangle(tri_start->_b->_pos, tri_start->_a->_pos, tri_start->_c->_pos, tri_color_start);
		}

		if (pathValid)
		{
			DrawBestPath(start_pos, goal_pos, pathFinder.GetFinalPath());
		}
	}


	void DrawBestPath(Vector3 start, const Vector3& goal, const std::list<NavVertex*> path)
	{
		const float line_thickness = 0.02f;
		const Vector4 line_color = Vector4(1.f, 0.f, 1.f, 1.0f);

		for (NavVertex* v : path)
		{
			Vector3 end = v->_pos;
			NCLDebug::DrawThickLine(start, end, line_thickness, line_color);
			start = end;
		}
		NCLDebug::DrawThickLine(start, goal, line_thickness, line_color);
	}

protected:
	GameObject* start;
	GameObject* end;


	Vector3 start_pos;
	Vector3 goal_pos;

	NavTri* tri_start;
	NavTri* tri_goal;

	bool			pathValid;
	Nav_SearchAStar pathFinder;
	//-----			pathSmoother;

	NavMesh*	navMesh;
};