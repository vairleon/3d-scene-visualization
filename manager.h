#ifndef MANAGER_H
#define MANAGER_H


#include <iostream>

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#include "model.h"



class GuiManager {

public :
	Model scenemodel;

	GuiManager(const Model& model);
	GuiManager();

	void ImguiInit(GLFWwindow* window);
	void setModel(const Model& model);
	void ShowManagerWindow(bool* p_open);

private:
	



	// Demonstrate the various window flags. Typically you would just use the default!
	bool no_titlebar = false;
	bool no_scrollbar = false;
	bool no_menu = false;
	bool no_move = false;
	bool no_resize = false;
	bool no_collapse = false;
	bool no_close = false;
	bool no_nav = false;
	bool no_background = false;
	bool no_bring_to_front = false;

	// Dear ImGui Apps (accessible from the "Tools" menu)
	bool show_app_metrics = false;
	bool show_app_style_editor = false;
	bool show_app_about = false;



	// -------- func ---------
	void HelpMarker(const char* desc)
	{
		ImGui::TextDisabled("(?)");
		if (ImGui::IsItemHovered())
		{
			ImGui::BeginTooltip();
			ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
			ImGui::TextUnformatted(desc);
			ImGui::PopTextWrapPos();
			ImGui::EndTooltip();
		}
	}
	void ShowExampleMenuFile();
	void ShowModelColumns();

	//void ShowExampleAppDocuments(bool* p_open);
	//void ShowExampleAppMainMenuBar();
	//void ShowExampleAppConsole(bool* p_open);
	//void ShowExampleAppLog(bool* p_open);
	//void ShowExampleAppLayout(bool* p_open);
	//void ShowExampleAppPropertyEditor(bool* p_open);
	//void ShowExampleAppLongText(bool* p_open);
	//void ShowExampleAppAutoResize(bool* p_open);
	//void ShowExampleAppConstrainedResize(bool* p_open);
	//void ShowExampleAppSimpleOverlay(bool* p_open);
	//void ShowExampleAppWindowTitles(bool* p_open);
	//void ShowExampleAppCustomRendering(bool* p_open);
	//void ShowExampleMenuFile();
};


#endif
