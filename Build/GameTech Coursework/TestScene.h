#pragma once

#include <ncltech\Scene.h>
#include <nclgl\OBJMesh.h>
#include <ncltech\SpringConstraint.h>

class TestScene : public Scene
{
public:
	TestScene(const std::string& friendly_name);
	virtual ~TestScene();

	virtual void OnInitializeScene()	 override;
	virtual void OnCleanupScene()		 override;
	virtual void OnUpdateScene(float dt) override;

	void SpawnSphere();

protected:
	float m_AccumTime;
	GameObject* m_pPlayer;
	GLuint S_texture;
};