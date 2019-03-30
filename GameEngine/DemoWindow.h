#ifndef _H_DEMO_WINDOW_
#define _H_DEMO_WINDOW_

#include "GLWindow.h"
#include "vectors.h"
#include "DemoBase.h"

class DemoWindow : public GLWindow {
protected:
	vec2 m_prevMousePos;
	int m_selectedDemo; // -1: None; 0, 1, 2....
	DemoBase* m_pDemo;
	bool imgui_init;
protected:
	void ApplyDemoCamera();
public:

	DemoWindow(const char* title, int width, int height)
		: GLWindow(title, width, height) {
		m_nFixedFPS = 30.0f;
		m_nTargetFPS = 60.0f;
	}
	virtual ~DemoWindow() { }

	virtual void OnRender();
	virtual void OnResize(int width, int height);
	virtual void OnInitialize();
	virtual void OnFixedUpdate(float deltaTime);
	virtual void OnUpdate(float deltaTime);
	virtual void OnShutdown();

	void StopDemo();
	void StartParticle();
	void StartSecond();
	void StartThird();
	//new demos here
};

// Static instance defined in .cpp file!

#endif 