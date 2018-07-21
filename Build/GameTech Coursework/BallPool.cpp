/*#include "BallPool.h"

#include <ncltech\CommonUtils.h>
using namespace CommonUtils;

BallPool::BallPool(const std::string& friendly_name)
	: Scene(friendly_name)
	, m_AccumTime(0.0f)
{
}

BallPool::~BallPool()
{

}

void BallPool::OnInitializeScene()
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
	GameObject* ground = BuildCuboidObject("InvisibleWall", Vector3(0.0f, -1.0f, 0.0f), Vector3(20.0f, 1.0f, 20.0f), true, 0.0f, true, false, Vector4(0.2f, 0.5f, 1.0f, 0.0f));
	GameObject* top = BuildCuboidObject("InvisibleWall", Vector3(0.0f, 39.0f, 0.0f), Vector3(20.0f, 1.0f, 20.0f), true, 0.0f, true, false, Vector4(0.2f, 0.5f, 1.0f, 0.0f));
	GameObject* back = BuildCuboidObject("InvisibleWall", Vector3(-20.0f, 19.0f, 0.0f), Vector3(1.0f, 20.0f, 20.0f), true, 0.0f, true, false, Vector4(0.2f, 0.5f, 1.0f, 0.0f));
	GameObject* front = BuildCuboidObject("InvisibleWall", Vector3(20.0f, 19.0f, 0.0f), Vector3(1.0f, 20.0f, 20.0f), true, 0.0f, true, false, Vector4(0.2f, 0.5f, 1.0f, 0.0f));
	GameObject* right = BuildCuboidObject("InvisibleWall", Vector3(0.0f, 19.0f, -20.0f), Vector3(20.0f, 20.0f, 1.0f), true, 0.0f, true, false, Vector4(0.2f, 0.5f, 1.0f, 0.0f));
	GameObject* left = BuildCuboidObject("InvisibleWall", Vector3(0.0f, 19.0f, 20.0f), Vector3(20.0f, 20.0f, 1.0f), true, 0.0f, true, false, Vector4(0.2f, 0.5f, 1.0f, 0.0f));
	this->AddGameObject(ground);
	this->AddGameObject(top);
	this->AddGameObject(back);
	this->AddGameObject(front);
	this->AddGameObject(right);
	this->AddGameObject(left);

	auto create_ball_cube = [&](const Vector3& offset, const Vector3& scale, float ballsize)
	{
		const int dims = 13;
		const Vector4 col = Vector4(0.0f, 0.0f, 1.0f, 1.0f);

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
						true,				// Dragable by user?
						col);// Render color
					sphere->Physics()->SetElasticity(0.0f);
					this->AddGameObject(sphere);
				}
			}
		}
	};

	create_ball_cube(Vector3(-18.0f, 1.0f, -18.0f), Vector3(3.0f, 3.0f, 3.0f), 1.0f);

	S_texture = SOIL_load_OGL_texture(TEXTUREDIR"TexturesCom_WoodBamboo0084_4_XL.jpg", SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, SOIL_FLAG_COMPRESS_TO_DXT);;
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

void BallPool::OnCleanupScene()
{
	//Just delete all created game objects 
	//  - this is the default command on any Scene instance so we don't really need to override this function here.
	Scene::OnCleanupScene();
}


void BallPool::OnUpdateScene(float dt)
{
	m_AccumTime += dt;

	if (Window::GetKeyboard()->KeyTriggered(KEYBOARD_J))
		SpawnSphere();
}



void BallPool::SpawnSphere() {
	GameObject* sphere = BuildSphereObject("Spawn", GraphicsPipeline::Instance()->GetCamera()->GetPosition(), 1.0f, true, 10.0f, true, true, Vector4(1, 0, 0, 1), S_texture);
	sphere->Physics()->SetLinearVelocity(Matrix3::Transpose(GraphicsPipeline::Instance()->GetCamera()->BuildViewMatrix()) * Vector3(0, 0, -1) * 100);
	this->AddGameObject(sphere);
}*/