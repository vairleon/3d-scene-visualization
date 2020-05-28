#include "manager.h"

GuiManager::GuiManager() {

}

GuiManager::GuiManager(const Model& model) {
	Model tempmodel(model);
	scenemodel = tempmodel;
}

void GuiManager::setModel(const Model& model) {
	Model tempmodel(model);
    scenemodel = tempmodel;
}

void GuiManager::ImguiInit(GLFWwindow* window) {
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // Setup Platform/Renderer bindings
    const char* glsl_version = "#version 330";
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Load Fonts
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
    // - If the file cannot be loaded, the function will return NULL. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
    // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
    // - Read 'docs/FONTS.txt' for more instructions and details.
    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
    //io.Fonts->AddFontDefault();
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/ProggyTiny.ttf", 10.0f);
    //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesJapanese());
    //IM_ASSERT(font != NULL);

}

void GuiManager::ShowManagerWindow(bool* p_open) {
    IM_ASSERT(ImGui::GetCurrentContext() != NULL && "Missing dear imgui context. Refer to examples app!"); // Exceptionally add an extra assert here for people confused with initial dear imgui setup
   

    if (show_app_metrics) { ImGui::ShowMetricsWindow(&show_app_metrics); }
    if (show_app_style_editor) { ImGui::Begin("Style Editor", &show_app_style_editor); ImGui::ShowStyleEditor(); ImGui::End(); }
    if (show_app_about) { ImGui::ShowAboutWindow(&show_app_about); }


    ImGuiWindowFlags window_flags = 0;
    if (no_titlebar)        window_flags |= ImGuiWindowFlags_NoTitleBar;
    if (no_scrollbar)       window_flags |= ImGuiWindowFlags_NoScrollbar;
    if (!no_menu)           window_flags |= ImGuiWindowFlags_MenuBar;
    if (no_move)            window_flags |= ImGuiWindowFlags_NoMove;
    if (no_resize)          window_flags |= ImGuiWindowFlags_NoResize;
    if (no_collapse)        window_flags |= ImGuiWindowFlags_NoCollapse;
    if (no_nav)             window_flags |= ImGuiWindowFlags_NoNav;
    if (no_background)      window_flags |= ImGuiWindowFlags_NoBackground;
    if (no_bring_to_front)  window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus;
    if (no_close)           p_open = NULL; // Don't pass our bool* to Begin

    // We specify a default position/size in case there's no data in the .ini file. Typically this isn't required! We only do it to make the Demo applications a little more welcoming.
    ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(550, 680), ImGuiCond_FirstUseEver);

    // Main body of the Demo window starts here.
    if (!ImGui::Begin("processing tools", p_open, window_flags))
    {
        // Early out if the window is collapsed, as an optimization.
        ImGui::End();
        return;
    }

    // Most "big" widgets share a common width settings by default.
    //ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.65f);    // Use 2/3 of the space for widgets and 1/3 for labels (default)
    ImGui::PushItemWidth(ImGui::GetFontSize() * -12);           // Use fixed width for labels (by passing a negative value), the rest goes to widgets. We choose a width proportional to our font size.

    // Menu Bar
    if (ImGui::BeginMenuBar())
    {
        if (ImGui::BeginMenu("Menu"))
        {
            ShowExampleMenuFile();
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Tools"))
        {
            ImGui::MenuItem("Metrics", NULL, &show_app_metrics);
            ImGui::MenuItem("Style Editor", NULL, &show_app_style_editor);
            ImGui::MenuItem("About Dear ImGui", NULL, &show_app_about);
            ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }

    //ImGui::Text("dear imgui says hello. (%s)", IMGUI_VERSION);
    ImGui::Spacing();

    if (ImGui::CollapsingHeader("Help"))
    {
        ImGui::Text("ABOUT THIS DEMO:");
        ImGui::BulletText("Sections below are demonstrating many aspects of the library.");
        ImGui::BulletText("The \"Examples\" menu above leads to more demo contents.");
        ImGui::BulletText("The \"Tools\" menu above gives access to: About Box, Style Editor,\n"
            "and Metrics (general purpose Dear ImGui debugging tool).");
        ImGui::Separator();

        ImGui::Text("PROGRAMMER GUIDE:");
        ImGui::BulletText("See the ShowDemoWindow() code in imgui_demo.cpp. <- you are here!");
        ImGui::BulletText("See comments in imgui.cpp.");
        ImGui::BulletText("See example applications in the examples/ folder.");
        ImGui::BulletText("Read the FAQ at http://www.dearimgui.org/faq/");
        ImGui::BulletText("Set 'io.ConfigFlags |= NavEnableKeyboard' for keyboard controls.");
        ImGui::BulletText("Set 'io.ConfigFlags |= NavEnableGamepad' for gamepad controls.");
        ImGui::Separator();

        ImGui::Text("USER GUIDE:");
        ImGui::ShowUserGuide();
    }

    // All demo contents
   /* ShowDemoWindowWidgets();
    ShowDemoWindowLayout();
    ShowDemoWindowPopups();*/

    ShowModelColumns();
    //ShowDemoWindowMisc();

    // End of ShowDemoWindow()
    ImGui::End();
}


void GuiManager::ShowExampleMenuFile()
{
    ImGui::MenuItem("(dummy menu)", NULL, false, false);
    if (ImGui::MenuItem("New")) {}
    if (ImGui::MenuItem("Open", "Ctrl+O")) {}
    if (ImGui::BeginMenu("Open Recent"))
    {
        ImGui::MenuItem("fish_hat.c");
        ImGui::MenuItem("fish_hat.inl");
        ImGui::MenuItem("fish_hat.h");
        if (ImGui::BeginMenu("More.."))
        {
            ImGui::MenuItem("Hello");
            ImGui::MenuItem("Sailor");
            if (ImGui::BeginMenu("Recurse.."))
            {
                ShowExampleMenuFile();
                ImGui::EndMenu();
            }
            ImGui::EndMenu();
        }
        ImGui::EndMenu();
    }
    if (ImGui::MenuItem("Save", "Ctrl+S")) {}
    if (ImGui::MenuItem("Save As..")) {}

    ImGui::Separator();
    if (ImGui::BeginMenu("Options"))
    {
        static bool enabled = true;
        ImGui::MenuItem("Enabled", "", &enabled);
        ImGui::BeginChild("child", ImVec2(0, 60), true);
        for (int i = 0; i < 10; i++)
            ImGui::Text("Scrolling Text %d", i);
        ImGui::EndChild();
        static float f = 0.5f;
        static int n = 0;
        ImGui::SliderFloat("Value", &f, 0.0f, 1.0f);
        ImGui::InputFloat("Input", &f, 0.1f);
        ImGui::Combo("Combo", &n, "Yes\0No\0Maybe\0\0");
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Colors"))
    {
        float sz = ImGui::GetTextLineHeight();
        for (int i = 0; i < ImGuiCol_COUNT; i++)
        {
            const char* name = ImGui::GetStyleColorName((ImGuiCol)i);
            ImVec2 p = ImGui::GetCursorScreenPos();
            ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x + sz, p.y + sz), ImGui::GetColorU32((ImGuiCol)i));
            ImGui::Dummy(ImVec2(sz, sz));
            ImGui::SameLine();
            ImGui::MenuItem(name);
        }
        ImGui::EndMenu();
    }

    // Here we demonstrate appending again to the "Options" menu (which we already created above)
    // Of course in this demo it is a little bit silly that this function calls BeginMenu("Options") twice.
    // In a real code-base using it would make senses to use this feature from very different code locations.
    if (ImGui::BeginMenu("Options")) // <-- Append!
    {
        static bool b = true;
        ImGui::Checkbox("SomeOption", &b);
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Disabled", false)) // Disabled
    {
        IM_ASSERT(0);
    }
    if (ImGui::MenuItem("Checked", NULL, true)) {}
    if (ImGui::MenuItem("Quit", "Alt+F4")) {}


}


void GuiManager::ShowModelColumns() {
    if (!ImGui::CollapsingHeader("MeshInfo"))
        return;

    ImGui::PushID("MeshInfo");

    static bool disable_indent = false;
    ImGui::Checkbox("Disable tree indentation", &disable_indent);
    ImGui::SameLine();
    HelpMarker("Disable the indenting of tree nodes so demo columns can use the full window width.");
    if (disable_indent)
        ImGui::PushStyleVar(ImGuiStyleVar_IndentSpacing, 0.0f);


    if (ImGui::TreeNode("mapping table"))
    {
        ImGui::Text("index : label");
        ImGui::Columns(4, "mycolumns3", false);  // 4-ways, no border
        ImGui::Separator();
        char _label[20][15] = {"Unidentified", "cabinet", "bed", "chair", "sofa", "table", "door",
            "window", "bookshelf", "picutre", "counter", "desk", "curtain", "refrigerator", "shower curtain", "toilet", "sink", "bathtub", "otherfurniture"};
        for (int n = 0; n < 19; n++)
        {
            char label[32];
            sprintf(label, "%d : %s", n, _label[n]);
            if (ImGui::Selectable(label)) {}
            //if (ImGui::Button(label, ImVec2(-FLT_MIN,0.0f))) {}
            ImGui::NextColumn();
        }
        // ImGui::Columns(1);
        ImGui::Columns(1);
        ImGui::Separator();
        ImGui::TreePop();
    }
    // Basic columns
    if (ImGui::TreeNode("Basic"))
    {
        static int selected = -1;
        
        if (ImGui::Button("Clear")) {
            // bug, cannot be fixed
            // scenemodel.discardUselessMeshes();
        }
        ImGui::SameLine();
        if (ImGui::Button("delete")) { 
            scenemodel.delMesh(scenemodel.getMeshId(selected));
        }
        ImGui::SameLine();
        if (ImGui::Button("setVisible")) {
            scenemodel.setVisible(scenemodel.getMeshId(selected));
        }
        // ImGui::Text("With border:");
        ImGui::Columns(4, "set columns"); // 4-ways, with border
        ImGui::Separator();
        ImGui::Text("ID"); ImGui::NextColumn();
        ImGui::Text("Label"); ImGui::NextColumn();
        ImGui::Text("Visible"); ImGui::NextColumn();
        ImGui::Text("Points"); ImGui::NextColumn();
        ImGui::Separator();
        
        for (int i = 0; i < scenemodel.getMeshNum(); i++)
        {
            char buffer[32];
            sprintf(buffer, "%03d", scenemodel.getMeshId(i));
            if (ImGui::Selectable(buffer, selected == i, ImGuiSelectableFlags_SpanAllColumns))
                selected = i;
            // bool hovered = ImGui::IsItemHovered();
            ImGui::NextColumn();
            ImGui::Text("%d", scenemodel.getMeshLabel(i)); ImGui::NextColumn();
            ImGui::Text( scenemodel.getIfVisible(i) ? "true" : "false" ); ImGui::NextColumn();
            ImGui::Text("%d", scenemodel.getMeshPointsNum(i)); ImGui::NextColumn();
        }
        ImGui::Columns(1);
        ImGui::Separator();
        ImGui::TreePop();
    }

    if (ImGui::TreeNode("command"))
    {
        // NB: Future columns API should allow automatic horizontal borders.
        static bool h_borders = true;
        static bool v_borders = true;
        static int columns_count = 2;
        // const int lines_count = floor((scenemodel.getMeshNum() - scenemodel.getReplacingMap.size())/2) + 1 ;
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8);
        ImGui::DragInt("##columns_count", &columns_count, 0.1f, 2, 10, "%d columns");
        if (columns_count < 2)
            columns_count = 2;
        ImGui::SameLine();
        ImGui::Checkbox("horizontal", &h_borders);
        ImGui::SameLine();
        ImGui::Checkbox("vertical", &v_borders);
        ImGui::Columns(columns_count, NULL, v_borders);

        for (int i = 0; i < scenemodel.getMeshNum(); i++)
        {
            if (scenemodel.getMeshInfo(i).m_face == -1)
                continue;

            if (h_borders && ImGui::GetColumnIndex() == 0)
                ImGui::Separator();
            ImGui::Text("mesh %d", scenemodel.getMeshId(i));
            ImGui::Text("label %d", scenemodel.getMeshLabel(i));
            //ImGui::Text("Transformation matrix:");
            if (ImGui::Button(("replace##" + to_string(i)).c_str(), ImVec2(-FLT_MIN, 0.0f))) 
            {
                // Ëã·¨´ý¸üÐÂ... 
                if (scenemodel.getMeshLabel(i) == 3) 
                    scenemodel.replaceMeshWithCADmodel(i);
            }

            ImGui::NextColumn();
        }
        ImGui::Columns(1);
        if (h_borders)
            ImGui::Separator();
        ImGui::TreePop();
    }

    // Create multiple items in a same cell before switching to next column
    if (ImGui::TreeNode("RegConfig"))
    {
       
    }

    // Word wrapping
    if (ImGui::TreeNode("Word-wrapping"))
    {
        ImGui::Columns(2, "word-wrapping");
        ImGui::Separator();
        ImGui::TextWrapped("The quick brown fox jumps over the lazy dog.");
        ImGui::TextWrapped("Hello Left");
        ImGui::NextColumn();
        ImGui::TextWrapped("The quick brown fox jumps over the lazy dog.");
        ImGui::TextWrapped("Hello Right");
        ImGui::Columns(1);
        ImGui::Separator();
        ImGui::TreePop();
    }


    // Scrolling columns
    /*
    if (ImGui::TreeNode("Vertical Scrolling"))
    {
        ImGui::BeginChild("##header", ImVec2(0, ImGui::GetTextLineHeightWithSpacing()+ImGui::GetStyle().ItemSpacing.y));
        ImGui::Columns(3);
        ImGui::Text("ID"); ImGui::NextColumn();
        ImGui::Text("Name"); ImGui::NextColumn();
        ImGui::Text("Path"); ImGui::NextColumn();
        ImGui::Columns(1);
        ImGui::Separator();
        ImGui::EndChild();
        ImGui::BeginChild("##scrollingregion", ImVec2(0, 60));
        ImGui::Columns(3);
        for (int i = 0; i < 10; i++)
        {
            ImGui::Text("%04d", i); ImGui::NextColumn();
            ImGui::Text("Foobar"); ImGui::NextColumn();
            ImGui::Text("/path/foobar/%04d/", i); ImGui::NextColumn();
        }
        ImGui::Columns(1);
        ImGui::EndChild();
        ImGui::TreePop();
    }
    */

    if (ImGui::TreeNode("Horizontal Scrolling"))
    {
        ImGui::SetNextWindowContentSize(ImVec2(1500.0f, 0.0f));
        ImGui::BeginChild("##ScrollingRegion", ImVec2(0, ImGui::GetFontSize() * 20), false, ImGuiWindowFlags_HorizontalScrollbar);
        ImGui::Columns(10);
        int ITEMS_COUNT = 2000;
        ImGuiListClipper clipper(ITEMS_COUNT);  // Also demonstrate using the clipper for large list
        while (clipper.Step())
        {
            for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; i++)
                for (int j = 0; j < 10; j++)
                {
                    ImGui::Text("Line %d Column %d...", i, j);
                    ImGui::NextColumn();
                }
        }
        ImGui::Columns(1);
        ImGui::EndChild();
        ImGui::TreePop();
    }

    if (ImGui::TreeNode("Tree"))
    {
        ImGui::Columns(2, "tree", true);
        for (int x = 0; x < 3; x++)
        {
            bool open1 = ImGui::TreeNode((void*)(intptr_t)x, "Node%d", x);
            ImGui::NextColumn();
            ImGui::Text("Node contents");
            ImGui::NextColumn();
            if (open1)
            {
                for (int y = 0; y < 3; y++)
                {
                    bool open2 = ImGui::TreeNode((void*)(intptr_t)y, "Node%d.%d", x, y);
                    ImGui::NextColumn();
                    ImGui::Text("Node contents");
                    if (open2)
                    {
                        ImGui::Text("Even more contents");
                        if (ImGui::TreeNode("Tree in column"))
                        {
                            ImGui::Text("The quick brown fox jumps over the lazy dog");
                            ImGui::TreePop();
                        }
                    }
                    ImGui::NextColumn();
                    if (open2)
                        ImGui::TreePop();
                }
                ImGui::TreePop();
            }
        }
        ImGui::Columns(1);
        ImGui::TreePop();
    }

    if (disable_indent)
        ImGui::PopStyleVar();
    ImGui::PopID();
}