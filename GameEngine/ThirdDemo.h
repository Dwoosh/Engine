#pragma once

#include "DemoBase.h"
#include "3DGeometry.h"
#include "PhysicsSystem.h"
#include "Cloth.h"

class ThirdDemo : public DemoBase {
protected:
	PhysicsSystem physicsSystem;
	Cloth cloth;
	OBB ground;
	std::vector<OBB> renderObjects;

	int num_part;
	float part_dist;
	float k;
	float d;

	bool size_imgui_window;
protected:
	void ResetDemo();
	float Random(float min, float max);
public:
	inline ThirdDemo() : DemoBase(), size_imgui_window(true) { }

	void Initialize(int width, int height);
	void Render();
	void Update(float dt);
	void ImGUI();
};