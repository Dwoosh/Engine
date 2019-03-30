#include "stdafx.h"
#include "DemoWindow.h"
#include "imgui/imgui.h"
#include "ParticleDemo.h"
#include "SecondDemo.h"
#include "ThirdDemo.h"
#include "LinearDemo.h"
#include "JointDemo.h"

// This is the global instance of the IWindow singleton!
static DemoWindow g_WindowInstance("Physics Sandbox", 800, 600);

void DemoWindow::OnInitialize() {
	GLWindow::OnInitialize();

	m_prevMousePos = vec2(0, 0);
	m_selectedDemo = -1;
	m_pDemo = 0;
	imgui_init = true;
}

void DemoWindow::OnResize(int width, int height) {
	GLWindow::OnResize(width, height);
	if (m_pDemo != 0) {
		m_pDemo->Resize(width, height);
	}
	ApplyDemoCamera();
}

void DemoWindow::OnRender() {
	GLWindow::OnRender();

	if (m_pDemo != 0) {
		mat4 view = m_pDemo->camera.getViewMatrix();
		SetGLModelView(view.asArray);

		m_pDemo->Render();
	}
}

void DemoWindow::ApplyDemoCamera() {
	if (m_pDemo == 0) {
		return;
	}

	mat4 projection = m_pDemo->camera.getProjMatrix();
	mat4 view = m_pDemo->camera.getViewMatrix();

	SetGLProjection(projection.asArray);
	SetGLModelView(view.asArray);
}

void DemoWindow::OnUpdate(float deltaTime) {
	GLWindow::OnUpdate(deltaTime);

	if (imgui_init) {
		ImGui::SetNextWindowPos(ImVec2(10, 10));
		imgui_init = false;
	}
	ImGui::SetNextWindowSize(ImVec2(370, 145));
	ImGui::Begin("Physics Demo", 0, ImGuiWindowFlags_NoResize);
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	
	const char* listbox_items[] = { "Particle Demo", "Boxes Demo", "Cloth Demo", "Collision Demo", "Joint Demo" };
	int lastSelected = m_selectedDemo;
	ImGui::PushItemWidth(350);
	ImGui::ListBox("", &m_selectedDemo, listbox_items, 5, 5);

	if (m_selectedDemo != lastSelected) {
		StopDemo();

		switch (m_selectedDemo) {
		case 0: m_pDemo = new ParticleDemo(); break;
		case 1: m_pDemo = new SecondDemo(); break;
		case 2: m_pDemo = new ThirdDemo(); break;
		case 3: m_pDemo = new LinearDemo(); break;
		case 4: m_pDemo = new JointDemo(); break;
		}

		m_pDemo->Initialize(GetWidth(), GetHeight());
		ApplyDemoCamera();
	}
	
	
	/*
	if (m_selectedDemo == 0) {
		if (ImGui::Button("Stop Particle Demo")) {
			m_selectedDemo = -1;
			StopDemo();
		}
	}
	else {
		if (ImGui::Button(" Run Particle Demo")) {
			m_selectedDemo = 0;
			StartParticle();
		}
	}
	ImGui::SameLine();
	if (m_selectedDemo == 1) {
		if (ImGui::Button("Stop Boxes Demo")) {
			m_selectedDemo = -1;
			StopDemo();
		}
	}
	else {
		if (ImGui::Button(" Run Boxes Demo")) {
			m_selectedDemo = 1;
			StartSecond();
		}
	}
	ImGui::SameLine();
	if (m_selectedDemo == 2) {
		if (ImGui::Button("Stop Cloth Demo")) {
			m_selectedDemo = -1;
			StopDemo();
		}
	}
	else {
		if (ImGui::Button(" Run Cloth Demo")) {
			m_selectedDemo = 2;
			StartThird();
		}
	}
	*/
	ImGui::End();

	if (m_pDemo != 0) {
		m_pDemo->ImGUI();
	}
}

void DemoWindow::OnFixedUpdate(float deltaTime) {
	GLWindow::OnFixedUpdate(deltaTime);
	
	bool leftDown = MouseButonDown(MOUSE_LEFT);
	bool middleDown = MouseButonDown(MOUSE_MIDDLE);
	bool rightDown = MouseButonDown(MOUSE_RIGHT);

	vec2 mousePos = GetMousePosition();
	vec2 mouseDelta = mousePos - m_prevMousePos;
	mouseDelta.x /= (float)GetWidth();
	mouseDelta.y /= (float)GetHeight();

	if (m_pDemo != 0) {
		m_pDemo->SetMouseState(leftDown, middleDown, rightDown, mouseDelta, mousePos);
		m_pDemo->Update(deltaTime);
	}

	m_prevMousePos = mousePos;
}

void DemoWindow::OnShutdown() {
	GLWindow::OnShutdown();
	StopDemo();
}
void DemoWindow::StopDemo() {
	if (m_pDemo != 0) {
		m_pDemo->Shutdown();
		delete m_pDemo;
	}
	m_pDemo = 0;
}

void DemoWindow::StartParticle() {
	StopDemo();
	m_pDemo = new ParticleDemo();
	m_pDemo->Initialize(GetWidth(), GetHeight());
	ApplyDemoCamera();
}

void DemoWindow::StartSecond() {
	StopDemo();
	m_pDemo = new SecondDemo();
	m_pDemo->Initialize(GetWidth(), GetHeight());
	ApplyDemoCamera();
}

void DemoWindow::StartThird() {
	StopDemo();
	m_pDemo = new ThirdDemo();
	m_pDemo->Initialize(GetWidth(), GetHeight());
	ApplyDemoCamera();
}

//new demos here