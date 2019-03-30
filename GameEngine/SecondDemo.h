#pragma once

#include "DemoBase.h"
#include "3DGeometry.h"
#include "PhysicsSystem.h"
#include "RigidbodyVolume.h"

class SecondDemo : public DemoBase {
protected:
	PhysicsSystem physicsSystem;
	std::vector<RigidbodyVolume> bodies;
	RigidbodyVolume groundBox;

	bool size_imgui_window;
protected:
	void ResetDemo();
public:
	void Initialize(int width, int height);
	void Render();
	void Update(float dt);
	void ImGUI();
};