#pragma once

#include "DemoBase.h"

#include "PhysicsSystem.h"
#include "Particle.h"
#include <vector>
#include <list>

class ParticleDemo : public DemoBase {
protected:
	PhysicsSystem physicsSystem;
	int numParticles;
	float bounce;

	std::vector<Particle> particles;
	std::list<float> deltaTimes;

	float lastFrameTime;
	bool size_imgui_window;
protected:
	void ResetDemo();
public:
	inline ParticleDemo() : DemoBase(), size_imgui_window(true) { }

	void Initialize(int width, int height);
	void Render();
	void Update(float dt);
	void ImGUI();

	float Random(float min, float max);
	vec3 Random(vec3 min, vec3 max);
};