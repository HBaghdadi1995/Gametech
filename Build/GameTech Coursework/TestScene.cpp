#include "TestScene.h"

#include <nclgl\Vector4.h>
#include <ncltech\GraphicsPipeline.h>
#include <ncltech\PhysicsEngine.h>
#include <ncltech\DistanceConstraint.h>
#include <ncltech\SceneManager.h>
#include <ncltech\CommonUtils.h>
using namespace CommonUtils;

TestScene::TestScene(const std::string& friendly_name)
	: Scene(friendly_name)
	, m_AccumTime(0.0f)
	, m_pPlayer(NULL)
{
}

TestScene::~TestScene()
{

}


void TestScene::OnInitializeScene()
{
	//Disable the physics engine (We will be starting this later!)
	PhysicsEngine::Instance()->SetPaused(true);

	//Set the camera position
	GraphicsPipeline::Instance()->GetCamera()->SetPosition(Vector3(15.0f, 10.0f, -15.0f));
	GraphicsPipeline::Instance()->GetCamera()->SetYaw(140.f);
	GraphicsPipeline::Instance()->GetCamera()->SetPitch(-20.f);

	m_AccumTime = 0.0f;

	//Example usage of Log 
	//- The on screen log can be opened from the bottom left though the
	//  main point of this is the ability to output to file easily from anywhere.
	//  So if your having trouble debugging with hundreds of 'cout << vector3()' just
	//  through them into NCLLOG() and look at the 'program_output.txt' later =]
	NCLDebug::Log("This is a log entry - It will printed to the console, on screen log and <project_dir>\program_output.txt");
	//NCLERROR("THIS IS AN ERROR!"); // <- On Debug mode this will also trigger a breakpoint in your code!



	//<--- SCENE CREATION --->
	//Create Ground
	GameObject* ground = BuildCuboidObject("Ground", Vector3(0.0f, -1.0f, 0.0f), Vector3(20.0f, 1.0f, 20.0f), true, 0.0f, true, false, Vector4(0.2f, 0.5f, 1.0f, 1.0f));
	PhysicsEngine::Instance()->octree = new Octree(ground->Physics()->GetPosition() - Vector3(ground->Physics()->GetCollisionShape()->GetRadius(), ground->Physics()->GetCollisionShape()->GetRadius(), ground->Physics()->GetCollisionShape()->GetRadius()), Vector3(ground->Physics()->GetCollisionShape()->GetRadius() * 2, ground->Physics()->GetCollisionShape()->GetRadius() *10, ground->Physics()->GetCollisionShape()->GetRadius() * 2));
	this->AddGameObject(ground);

	//Create Player (See OnUpdateScene)
	//m_pPlayer->SetRender(new RenderNode(new OBJMesh(MESHDIR"trex.obj")));

	m_pPlayer = BuildCuboidObject(
		"Player1",					// Optional: Name
		Vector3(5.f, 0.5f, 0.0f),	// Position
		Vector3(0.5f, 0.5f, 1.0f),  // Half-Dimensions
		true,						// Physics Enabled?
		0.f,						// Physical Mass (must have physics enabled)
		true,						// Physically Collidable (has collision shape)
		false,						// Dragable by user?
		Vector4(0.1f, 0.1f, 0.1f, 1.0f)); // Render color*/
										  //m_pPlayer->SetRender(new RenderNode(new OBJMesh(MESHDIR"trex.obj")));
	this->AddGameObject(m_pPlayer);


	auto create_cube_tower = [&](const Vector3& offset, float cubewidth)
	{
		const Vector3 halfdims = Vector3(cubewidth, cubewidth, cubewidth) * 0.5f;
		for (int x = 0; x < 2; ++x)
		{
			for (int y = 0; y < 6; ++y)
			{
				uint idx = x * 5 + y;
				Vector4 color = CommonUtils::GenColor(idx / 10.f, 0.5f);
				Vector3 pos = offset + Vector3(x * cubewidth, 1e-3f + y * cubewidth, cubewidth * (idx % 2 == 0) ? 0.5f : -0.5f);

				GameObject* cube = BuildCuboidObject(
					"Bad Target",						// Optional: Name
					pos,					// Position
					halfdims,				// Half-Dimensions
					true,					// Physics Enabled?
					1.f,					// Physical Mass (must have physics enabled)
					true,					// Physically Collidable (has collision shape)
					true,					// Dragable by user?
					color);					// Render color
				this->AddGameObject(cube);
			}
		}
	};

	auto create_ball_cube = [&](const Vector3& offset, const Vector3& scale, float ballsize)
	{
		const int dims = 4;
		const Vector4 col = Vector4(1.0f, 0.5f, 0.2f, 1.0f);

		for (int x = 0; x < dims; ++x)
		{
			for (int y = 0; y < dims; ++y)
			{
				for (int z = 0; z < dims; ++z)
				{
					Vector3 pos = offset + Vector3(scale.x *x, scale.y * y, scale.z * z);
					GameObject* sphere = BuildSphereObject(
						"",					// Optional: Name
						pos,				// Position
						ballsize,			// Half-Dimensions
						true,				// Physics Enabled?
						10.f,				// Physical Mass (must have physics enabled)
						true,				// Physically Collidable (has collision shape)
						false,				// Dragable by user?
						col);// Render color
					this->AddGameObject(sphere);
				}
			}
		}
	};

	auto create_soft_body = [&](const Vector3& offset, const Vector3& scale, float ballsize)
	{
		const int dims = 15;
		const Vector4 col = Vector4(1.0f, 0.5f, 0.2f, 1.0f);

		GameObject *objects[dims][dims];

		for (int x = 0; x < dims; ++x)
		{
			for (int y = 0; y < dims; ++y)
			{
				Vector3 pos = offset + Vector3(scale.x *x, scale.y * y, scale.z);
				GameObject* sphere;
				if ((x == 0 || x == dims - 1) && y == dims - 1) {
					sphere = BuildSphereObject(
						"soft_body",					// Optional: Name
						pos,				// Position
						ballsize,			// Half-Dimensions
						true,				// Physics Enabled?
						0.0f,				// Physical Mass (must have physics enabled)
						true,				// Physically Collidable (has collision shape)
						true,				// Dragable by user?
						col);// Render color
					objects[x][y] = sphere;
				}
				else {
					sphere = BuildSphereObject(
						"soft_body",					// Optional: Name
						pos,				// Position
						ballsize,			// Half-Dimensions
						true,				// Physics Enabled?
						10.f,				// Physical Mass (must have physics enabled)
						true,				// Physically Collidable (has collision shape)
						true,				// Dragable by user?
						col);// Render color
					objects[x][y] = sphere;
				}
				//sphere->Physics()->SetElasticity(0.1);
				if (x != 0) {
					GameObject* obj1 = objects[x][y];
					GameObject* obj2 = objects[x - 1][y];


					Constraint *c = new DistanceConstraint(obj1->Physics(), obj2->Physics(), obj1->Physics()->GetPosition(), obj2->Physics()->GetPosition());
					PhysicsEngine::Instance()->AddConstraint(c);
				}
				if (y != 0) {
					GameObject* obj1 = objects[x][y];
					GameObject* obj2 = objects[x][y - 1];


					Constraint *c = new DistanceConstraint(obj1->Physics(), obj2->Physics(), obj1->Physics()->GetPosition(), obj2->Physics()->GetPosition());
					PhysicsEngine::Instance()->AddConstraint(c);
				}
				this->AddGameObject(sphere);
			}
		}
	};

	//Create Cube Towers
	create_cube_tower(Vector3(3.0f, 0.5f, 3.0f), 1.0f);
	create_cube_tower(Vector3(-3.0f, 0.5f, -3.0f), 1.0f);

	//Create Test Ball Cubey things
	create_ball_cube(Vector3(-8.0f, 0.5f, 12.0f), Vector3(0.5f, 0.5f, 0.5f), 0.1f);
	create_ball_cube(Vector3(8.0f, 0.5f, 12.0f), Vector3(0.3f, 0.3f, 0.3f), 0.1f);
	create_ball_cube(Vector3(-8.0f, 0.5f, -12.0f), Vector3(0.2f, 0.2f, 0.2f), 0.1f);
	create_ball_cube(Vector3(8.0f, 0.5f, -12.0f), Vector3(0.5f, 0.5f, 0.5f), 0.1f);

	create_soft_body(Vector3(10.0f, 10.0f, 10.0f), Vector3(0.25f, 0.25f, 0.25f), 0.1f);

	GameObject* sphere = BuildCuboidObject("Good Target", Vector3(10.0, 10.0, 0.0), Vector3(1.0f, 1.0f, 1.0f), true, 10.0f, true, true, Vector4(1, 0, 0, 1));
	this->AddGameObject(sphere);

	GameObject* p = BuildSphereObject("", Vector3(0.0f, 10.0f, 0.0f), 1.0f, true, 0, true, true, Vector4(0, 1, 0, 1));

	this->AddGameObject(p);

	Constraint *c = new DistanceConstraint(p->Physics(), sphere->Physics(), p->Physics()->GetPosition(), sphere->Physics()->GetPosition());

	PhysicsEngine::Instance()->AddConstraint(c);
	//PhysicsEngine::Instance()->SetGravity(Vector3(0.0f, 0.81f, 0.0f));

	GameObject* MultiMesh = BuildCombinedObject("Good Target", Vector3(10.0, 15.0, 0.0), 1.0f, true, 10.0f, true, true, Vector4(1, 0, 0, 1));

	this->AddGameObject(MultiMesh);

	S_texture = SOIL_load_OGL_texture(TEXTUREDIR"TexturesCom_WoodBamboo0084_4_XL.jpg", SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, SOIL_FLAG_COMPRESS_TO_DXT);

	if (S_texture)
	{
		glBindTexture(GL_TEXTURE_2D, S_texture);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); //No linear interpolation to get crisp checkerboard no matter the scalling
		glBindTexture(GL_TEXTURE_2D, 0);
	}
	else
	{
		NCLERROR("Unable to load checkerboard texture!");
	}
}

void TestScene::OnCleanupScene()
{
	//Just delete all created game objects 
	//  - this is the default command on any Scene instance so we don't really need to override this function here.
	Scene::OnCleanupScene();
}

void TestScene::OnUpdateScene(float dt)
{
	m_AccumTime += dt;

	// You can add status entries to the top left from anywhere in the program
	NCLDebug::AddStatusEntry(Vector4(1.0f, 0.2f, 0.2f, 1.0f), "Welcome to the Game Tech Framework!");
	NCLDebug::AddStatusEntry(Vector4(1.0f, 0.4f, 0.4f, 1.0f), "   You can move the black car with the arrow keys");

	// You can print text using 'printf' formatting
	//bool donkeys = true;
	bool donkeys = false;
	NCLDebug::AddStatusEntry(Vector4(1.0f, 0.4f, 0.4f, 1.0f), "   The %s in this scene are dragable", donkeys ? "donkeys" : "cubes");
	NCLDebug::AddStatusEntry(Vector4(1.0f, 0.4f, 0.4f, 1.0f), "   - Left click to move");
	NCLDebug::AddStatusEntry(Vector4(1.0f, 0.4f, 0.4f, 1.0f), "   - Right click to rotate (They will be more spinnable after tutorial 2)");

	//Or move our car around the scene..
	{
		const float mv_speed = 5.f;				//Motion: Meters per second
		const float rot_speed = 90.f * dt;		//Rotation: Degrees per second

		bool boosted = false;

		PhysicsNode* pobj = m_pPlayer->Physics();
		Vector3 vel;
		if (Window::GetKeyboard()->KeyDown(KEYBOARD_UP))
		{
			vel = pobj->GetOrientation().ToMatrix3()* Vector3(0.0f, 0.0f, -mv_speed);
			boosted = true;
		}

		if (Window::GetKeyboard()->KeyDown(KEYBOARD_DOWN))
		{
			vel = pobj->GetOrientation().ToMatrix3()* Vector3(0.0f, 0.0f, mv_speed);
		}
		pobj->SetLinearVelocity(vel);
		pobj->SetPosition(pobj->GetPosition() + vel * dt);

		if (Window::GetKeyboard()->KeyDown(KEYBOARD_LEFT))
		{
			pobj->SetOrientation(pobj->GetOrientation() *
				Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), rot_speed));
		}

		if (Window::GetKeyboard()->KeyDown(KEYBOARD_RIGHT))
		{
			pobj->SetOrientation(pobj->GetOrientation() *
				Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), -rot_speed));
		}

		if (Window::GetKeyboard()->KeyTriggered(KEYBOARD_J))
			SpawnSphere();


		// If you ever need to see any visual output from your algorithms, for debugging or otherwise
		// you can now use NCLDebug:: from anywhere in the program using world space coordinates. The only
		// requirement is that it gets called each frame you need something renderered.
		//
		// All NCLDebug::Draw functions come in either normal or NDT (not depth tested) variants relating to if
		// you want them to clip with objects in the world or always be visible.
		if (boosted)
		{
			//Draw the rocket booster on the car using NCLDebug
			Vector3 backward_dir = pobj->GetOrientation().ToMatrix3() * Vector3(0, 0, 1);
			NCLDebug::DrawPoint(pobj->GetPosition() + backward_dir, 0.3f, Vector4(1.f, 0.7f, 0.0f, 1.0f));
			NCLDebug::DrawPoint(pobj->GetPosition() + backward_dir * 1.333f, 0.26f, Vector4(0.9f, 0.5f, 0.0f, 1.0f));
			NCLDebug::DrawPoint(pobj->GetPosition() + backward_dir * 1.666f, 0.23f, Vector4(0.8f, 0.3f, 0.0f, 1.0f));
			NCLDebug::DrawPoint(pobj->GetPosition() + backward_dir * 2.f, 0.2f, Vector4(0.7f, 0.2f, 0.0f, 1.0f));
		}
	}
}

void TestScene::SpawnSphere() {
	GameObject* sphere = BuildSphereObject("Spawn", GraphicsPipeline::Instance()->GetCamera()->GetPosition(), 1.0f, true, 10.0f, true, true, Vector4(1, 0, 0, 1), S_texture);
	sphere->Physics()->SetLinearVelocity(Matrix3::Transpose(GraphicsPipeline::Instance()->GetCamera()->BuildViewMatrix()) * Vector3(0, 0, -1) * 100);
	this->AddGameObject(sphere);
}