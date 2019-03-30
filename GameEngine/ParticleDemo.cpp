#include "stdafx.h"
#include "ParticleDemo.h"
#include "FixedFunctionPrimitives.h"
#include "glad/glad.h"
#include "imgui/imgui.h"
#include <iostream>

extern double GetMilliseconds();

void ParticleDemo::Initialize(int width, int height) {
	DemoBase::Initialize(width, height);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	float val[] = { 0.5f, 1.0f, -1.5f, 0.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, val);

	numParticles = 50;
	bounce = 0.0f;
	size_imgui_window = true;
	lastFrameTime = GetMilliseconds();

	camera.setTarget(vec3(3.75622f, 2.98255f, 0.0f));
	camera.setZoom(9.0f);
	camera.setRotation(vec2(-67.9312f, 19.8f));

	ResetDemo();
}

void ParticleDemo::ResetDemo() {
	particles.clear();
	particles.reserve(numParticles);

	physicsSystem.clearRigidbodies();
	physicsSystem.clearConstraints();


	OBB ground;
	ground.size = vec3(10.0f, 0.15f, 10.0f);

	OBB obb1;
	obb1.center = vec3(0.0f, 1.86f, -1.92f);
	obb1.orientation = rotationMat3(30.716f, 0.0f, 0.0f);
	obb1.size = vec3(2.0f, 0.15f, 2.0f);

	OBB obb2;
	obb2.center = vec3(-1.0f, 3.6f, 1.2f);
	obb2.orientation = rotationMat3(-33.964f, -24.233f, 9.128f);
	obb2.size = vec3(2.0f, 0.15f, 2.0f);

	OBB obb3;
	obb3.center = vec3(0.0f, 3.93f, -2.27f);
	obb3.orientation = rotationMat3(24.702f, 0.0f, 0.0f);
	obb3.size = vec3(2.0f, 0.15f, 0.7817011f);

	physicsSystem.addConstraint(ground);
	physicsSystem.addConstraint(obb1);
	physicsSystem.addConstraint(obb2);
	physicsSystem.addConstraint(obb3);

	vec3 spawnPos = vec3(-0.5f, 6.5f, -1.01f);
	vec3 spawnSize = vec3(3.8505f, 2, 4.034834f);
	vec3 spawnMin = spawnPos - spawnSize;
	vec3 spawnMax = spawnPos + spawnSize;

	for (int i = 0; i < numParticles; ++i) {
		particles.push_back(Particle());
		particles[i].setPosition(Random(spawnMin, spawnMax));
		if (bounce == 0.0f) {
			particles[i].setBounce(Random(0, 1));
		}
		else {
			particles[i].setBounce(bounce);
		}
		physicsSystem.addRigidbody(&particles[i]);
	}

	/* Debug
	particles.push_back(Particle());
	particles[0].SetPosition(vec3(-1.07645, 7.29192, 0.292606));
	physicsSystem.AddRigidbody(&particles[0]);
	*/
}

void ParticleDemo::Render() {
	DemoBase::Render();

	float val[] = { 0.0f, 1.0f, 0.0f, 0.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, val);

	physicsSystem.render();
}

void ParticleDemo::ImGUI() {
	DemoBase::ImGUI();

	float avgTime = 0;
	std::list<float>::iterator iterator = deltaTimes.begin();
	for (; iterator != deltaTimes.end(); iterator++) {
		avgTime += *iterator;
	}
	if (deltaTimes.size() > 0) {
		avgTime /= deltaTimes.size();
	}

	if (size_imgui_window) {
		size_imgui_window = false;
		ImGui::SetNextWindowPos(ImVec2(400, 10));
		ImGui::SetNextWindowSize(ImVec2(400, 125));
	}

	ImGui::Begin("Particle Demo", 0, ImGuiWindowFlags_NoResize);
	ImGui::Text("Simulation delta: %.3f ms/frame", avgTime);

	ImGui::Text("Particle count");
	ImGui::SameLine();
	ImGui::PushItemWidth(100);
	ImGui::SliderInt("", &numParticles, 1, 110);

	ImGui::InputFloat("Bounce (should be between 0.0-1.0)", &bounce);

	if (ImGui::Button("Reset")) {
		ResetDemo();
	}
	ImGui::SameLine();
	if (ImGui::Button("Show Help")) {
		show_help = true;
	}
	ImGui::End();
}

void ParticleDemo::Update(float dt) {
	DemoBase::Update(dt);

	double thisFrameTime = GetMilliseconds();
	deltaTimes.push_back(float(thisFrameTime - lastFrameTime));
	while (deltaTimes.size() > 120) {
		deltaTimes.pop_front();
	}
	lastFrameTime = thisFrameTime;

	physicsSystem.update(dt);

	// in case the simulation becomes unstable and something 
	// falls below the floor
	for (int i = 0; i < particles.size(); ++i) {
		vec3 position = particles[i].getPosition();
		if (position.y < -5.0f) {
			position.y = -5.0f;
			particles[i].setPosition(position);
		}
	}
}

float ParticleDemo::Random(float min, float max) {
	if (max < min) {
		float t = min;
		min = max;
		max = t;
	}

	float random = ((float)rand()) / (float)RAND_MAX;

	float range = max - min;
	return (random*range) + min;
}

vec3 ParticleDemo::Random(vec3 min, vec3 max) {
	vec3 result;
	result.x = Random(min.x, max.x);
	result.y = Random(min.y, max.y);
	result.z = Random(min.z, max.z);
	return result;
}