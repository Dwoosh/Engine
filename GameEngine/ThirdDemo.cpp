#include "ThirdDemo.h"
#include "FixedFunctionPrimitives.h"
#include "glad/glad.h"
#include "imgui/imgui.h"
#include "imgui/ImGuizmo.h"
#include <iostream>

void ThirdDemo::Initialize(int width, int height) {
	DemoBase::Initialize(width, height);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	float val[] = { 0.5f, 1.0f, -1.5f, 0.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, val);

	size_imgui_window = true;

	camera.setTarget(vec3(3.75622f, 2.98255f, 0.0f));
	camera.setZoom(9.0f);
	camera.setRotation(vec2(-67.9312f, 19.8f));

	num_part = 15;
	part_dist = 0.2f;
	k = -3.0f;
	d = 0.0f;
	ground.size = vec3(10.0f, 0.1f, 10.0f);

	ResetDemo();
}

float ThirdDemo::Random(float min, float max) {
	if (max < min) {
		float t = min;
		min = max;
		max = t;
	}

	float random = ((float)rand()) / (float)RAND_MAX;

	float range = max - min;
	return (random*range) + min;
}

void ThirdDemo::ResetDemo() {
	physicsSystem.clearRigidbodies();
	physicsSystem.clearConstraints();
	physicsSystem.clearSprings();
	physicsSystem.clearCloths();
	renderObjects.clear();

	cloth.initialize(num_part, part_dist, vec3(0, 6, 0));
	cloth.setStructuralSprings(k, d);
	cloth.setBendSprings(k, d);
	cloth.setShearSprings(k, d);
	physicsSystem.addCloth(&cloth);

	physicsSystem.addConstraint(ground);
	renderObjects.resize(4);

	float d = 0.5f;
	renderObjects[0].center = vec3(d, 2.4f, d);
	renderObjects[1].center = vec3(-d, 2.3f, d);
	renderObjects[2].center = vec3(-d, 2.4f, -d);
	renderObjects[3].center = vec3(d, 2.3f, -d);

	for (int i = 0; i < 4; ++i) {
		renderObjects[i].size = vec3(0.3f, 0.5f, 0.3f);
		physicsSystem.addConstraint(renderObjects[i]);
		renderObjects[i].size = vec3(0.1f, 0.5f, 0.1f);
	}
}

void ThirdDemo::ImGUI() {
	DemoBase::ImGUI();

	if (size_imgui_window) {
		size_imgui_window = false;
		ImGui::SetNextWindowPos(ImVec2(400, 10));
		ImGui::SetNextWindowSize(ImVec2(370, 75));
	}

	ImGui::Begin("Cloth Demo", 0, ImGuiWindowFlags_NoResize);


	ImGui::PushItemWidth(50);
	ImGui::SliderInt("Count", &num_part, 5, 30);
	ImGui::SameLine();
	ImGui::PushItemWidth(50);
	ImGui::SliderFloat("Distance", &part_dist, 0.1f, 1.0f);
	ImGui::SameLine();
	ImGui::PushItemWidth(50);
	ImGui::SliderFloat("Spring k", &k, -25.0f, 0.0f);

	if (ImGui::Button("Reset")) {
		ResetDemo();
	}


	ImGui::End();
}

void ThirdDemo::Render() {
	DemoBase::Render();

	// Mesh bending is not implemented, so we render the 
	// obstacle which the cloth is resting on with a cheat

	static const float constraintDiffuse[]{ 0.0f, 200.0f / 255.0f, 0.0f, 0.0f };
	static const float constraintAmbient[]{ 50.0f / 255.0f, 200.0f / 255.0f, 50.0f / 255.0f, 0.0f };
	static const float groundDiffuse[]{ 0.0f, 0.0f, 200.0f / 255.0f, 0.0f };
	static const float groundAmbient[]{ 50.0f / 255.0f, 50.0f / 255.0f, 200.0f / 255.0f, 0.0f };
	static const float val[] = { 0.0f, 1.0f, 0.0f, 0.0f };
	static const float zero[] = { 0.0f, 0.0f, 0.0f, 0.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, val);

	glColor3f(constraintDiffuse[0], constraintDiffuse[1], constraintDiffuse[2]);
	glLightfv(GL_LIGHT0, GL_AMBIENT, constraintAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, constraintDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, zero);
	for (int i = 0; i < renderObjects.size(); ++i) {
		::Render(renderObjects[i]);
	}

	glColor3f(groundDiffuse[0], groundDiffuse[1], groundDiffuse[2]);
	glLightfv(GL_LIGHT0, GL_AMBIENT, groundAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, groundDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, zero);
	::Render(ground);

	cloth.render();
	//cloth.render(physicsSystem.DebugRender);

	//physicsSystem.Render();

	/*Line l(vec3(0, -8, 0), vec3(0, 8, 0));
	::Render(l);*/
}

void ThirdDemo::Update(float dt) {
	DemoBase::Update(dt);

	physicsSystem.update(dt);
}