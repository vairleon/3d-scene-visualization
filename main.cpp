//
//#include <GL/gl3w.h> 
//#include <GLFW/glfw3.h>
//
//#include <glm/glm.hpp>
//#include <glm/gtc/matrix_transform.hpp>
//#include <glm/gtc/type_ptr.hpp>
//
//#define STB_IMAGE_IMPLEMENTATION
//#include"stb_image.h"
//
//// imgui
////#include "imgui/imgui.h"
////#include "imgui/imgui_impl_glfw.h"
////#include "imgui/imgui_impl_opengl3.h"
//
//// syslib
//#include <iostream>
//#include <cmath>
//#include <vector>
//
//// cuslib
//#include"shader.h"
//#include"camera.h"
//#include"target.h"
//#include"model.h"
//
//void _transformation(glm::mat4& target, const glm::mat4& trans);
//void framebuffer_size_callback(GLFWwindow* window, int width, int height);
//void mouse_callback(GLFWwindow* window, double xpos, double ypos);
//void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
//void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos);
//void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
//void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
//void processInput(GLFWwindow* window);
//void customAnimation(glm::mat4& model, const float& current_time, const vector<glm::vec3>& posit);
//
//// settings 
//const unsigned int SCR_WIDTH = 1080;
//const unsigned int SCR_HEIGHT = 720;
//
//
//// control bool 
//bool keepPressMouseLeft = false;
//bool keepPressModControl = false;
//
//// camera
//Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
//bool firstMouse = true;
//
//// target
//Target targetmodel(glm::mat4(1.0f));
//
//// cursor pos
//float cursor_x = SCR_WIDTH / 2.0f;
//float cursor_y = SCR_HEIGHT / 2.0f;
//float lastX = SCR_WIDTH / 2.0f;
//float lastY = SCR_HEIGHT / 2.0f;
//
//// timing
//float deltaTime = 0.0f;	// time between current frame and last frame
//float lastFrame = 0.0f;
//
//
//#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
//#pragma comment(lib, "legacy_stdio_definitions")
//#endif
//
//int main()
//{
//    // glfw: initialize and configure
//      // ------------------------------
//    glfwInit();
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
//    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//
//#ifdef __APPLE__
//    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // uncomment this statement to fix compilation on OS X
//#endif
//
//    // glfw window creation
//    // --------------------
//    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
//    if (window == NULL)
//    {
//        std::cout << "Failed to create GLFW window" << std::endl;
//        glfwTerminate();
//        return -1;
//    }
//
//    glfwMakeContextCurrent(window);
//    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
//    glfwSetKeyCallback(window, key_callback);
//    //glfwSetCursorPosCallback(window, mouse_callback);
//    glfwSetCursorPosCallback(window, cursor_pos_callback);
//    glfwSetMouseButtonCallback(window, mouse_button_callback);
//    glfwSetScrollCallback(window, scroll_callback);
//
//    // tell GLFW to capture mouse
//    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
//
//    // gl3w: load all OpenGL function pointers
//    // ---------------------------------------
//    if (gl3wInit() != 0 )
//    {
//        fprintf(stderr, "Failed to initialize OpenGL loader!\n");
//        return 1;
//    }
//    
//
//    // configure global opengl state
//    // -----------------------------
//    glEnable(GL_DEPTH_TEST);
//    // build and compile shaders
//    // -------------------------
//    //Shader sceneShader("shader\\scene_shader.vs", "shader\\scene_shader.fs"); 
//    Shader lightingShader("shader\\light_shader.vs", "shader\\light_shader.fs");
//    //Shader cubeShader("shader\\shader.vs", "shader\\shader.fs");
//    //Shader cursorShader("shader\\cursor_shader.vs", "shader\\cursor_shader.fs");
//
//    // load models
//    // -----------
//    vector<string> str;
//    //str.push_back("model\\scene0144_00_f.ply");
//    str.push_back("model\\scenes\\scene0423_00.ply");
//    //str.push_back("model\\scenes\\sc0378_00.ply");
//    //str.push_back("model\\cube_fn.obj");
//    //str.push_back("model\\cube_fn.ply");
//    //str.push_back("output\\temp\\1ab4c6ef68073113cf004563556ddb36\\models\\model_normalized.obj");
//    //str.push_back("output\\temp\\1b3c286bce219ddb7135fc51795b4038\\models\\model_normalized.obj");
//    Model sceneModel = Model(str);
//    //sceneModel.meshNormalization(2);
//    //sceneModel.meshNormalization(7);
//    // save mesh separation results
//    // -----------
//    //sceneModel.saveMesh("output\\temp\\");
//    sceneModel.centerMeshes();
//
//    unsigned int mesh_id = 2;
//    sceneModel.replaceMeshWithCADmodel(mesh_id);
//    mesh_id = 3;
//    sceneModel.replaceMeshWithCADmodel(mesh_id);
//    mesh_id = 5;
//    sceneModel.replaceMeshWithCADmodel(mesh_id);
//    mesh_id = 6;
//    sceneModel.replaceMeshWithCADmodel(mesh_id);
//
// 
//    // render loop
//    // -----------
//    while (!glfwWindowShouldClose(window))
//    {
//        // per-frame time logic
//        // --------------------
//        float currentFrame = glfwGetTime();
//        deltaTime = currentFrame - lastFrame;
//        lastFrame = currentFrame;
//
//        // input
//        // -----
//        processInput(window);
//
//        // render
//        // ------
//        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
//        //glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
//        glm::mat4 view = camera.GetViewMatrix();
//        view = view * targetmodel.GetTransformationMatrix();
//        //glm::mat4 model = targetmodel.GetTransformationMatrix();
//        glm::mat4 model = glm::mat4(1.0f);
//
//        //model = glm::translate(model, glm::vec3(-1.25f, -2.5f, -0.5f));
//
//        lightingShader.use();
//        // transformation settings
//        lightingShader.setMat4("view", view);
//        lightingShader.setMat4("model", model);
//        lightingShader.setMat4("projection", projection);
//        lightingShader.setVec3("light.direction", -1.0f, -1.0f, 1.0f);
//        lightingShader.setVec3("viewPos", camera.Position);
//        // light properties
//        lightingShader.setVec3("light.lightColor", 1.0f, 1.0f, 1.0f);
//        lightingShader.setVec3("light.ambient", 0.2f, 0.2f, 0.2f);
//        lightingShader.setVec3("light.diffuse", 1.0f, 1.0f, 1.0f);
//        lightingShader.setVec3("light.specular", 1.0f, 1.0f, 1.0f);
//        // material properties
//        lightingShader.setFloat("material.shininess", 8.0f);
//        // set blinn phong lighting model
//        lightingShader.setInt("blinn", true);
//        // adjust target color
//        lightingShader.setFloat("alpha", 1.0f);
//        lightingShader.setInt("if_setColor", false);
//        lightingShader.setVec3("objectColor", 1.0f, 0.5f, 0.31f);
//        
//        // draw target model
//        //glm::mat4 model = glm::mat4(1.0f);
//        //lightingShader.setMat4("model", model);
//        //sceneModel.Draw(lightingShader, 2);  // draw mesh where mesh_id == 1
//        //sceneModel.Draw(lightingShader);
//        
//        for (int i = 0; i < sceneModel.getMeshNum(); i++) {
//            if (!sceneModel.getIfVisible(i))   continue;
//            model = sceneModel.getMeshTransformation(i);
//            // _transformation(model, targetmodel.GetTransformationMatrix());
//            lightingShader.setMat4("model", model);
//            sceneModel.Draw(lightingShader, i);
//        }
//        
//
//        // draw cursor
//        //cursorShader.use();
//        //cursorShader.setVec3("aPos", cursor_x, cursor_y, 0.0f);
//        //cursorShader.setVec3("aColor", 0.0f, 1.0f, 0.0f);
//        //glPointSize(10);
//        //glDrawArrays(GL_POINTS, 0, 1);
//
//
//        // don't forget to enable shader before setting uniforms
//        // draw scene model
//        //sceneShader.use();
//        //sceneShader.setMat4("view", view);
//        //sceneShader.setMat4("model", model);
//        //sceneShader.setMat4("projection", projection);
//        //sceneModel.Draw(sceneShader);
//
//        //i++;
//        //if (i % 1000 == 0) {
//        //    /*targetmodel.outputTransformationMatrix();*/
//        //    for (int i = 0; i < 4; i++) {
//        //        std::cout << model[i].x << model[i].y << model[i].z << model[i].w << std::endl;
//        //    }
//        //}
//
//        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
//        // -------------------------------------------------------------------------------
//        glfwSwapBuffers(window);
//        glfwPollEvents();
//    }
//
//    // glfw: terminate, clearing all previously allocated GLFW resources.
//    // ------------------------------------------------------------------
//    glfwTerminate();
//    return 0;
//}
//
//
//void _transformation(glm::mat4& target, const glm::mat4& trans) {
//    glm::mat3 rotation_tgt;
//    glm::vec3 translation_tgt;
//    glm::mat3 rotation_trs;
//    glm::vec3 translation_trs;
//
//    for (int i = 0; i < 3; i++) {
//        rotation_tgt[i].x = target[i].x;
//        rotation_tgt[i].y = target[i].y;
//        rotation_tgt[i].z = target[i].z;
//        rotation_trs[i].x = trans[i].x;
//        rotation_trs[i].y = trans[i].y;
//        rotation_trs[i].z = trans[i].z;
//        translation_tgt[i] = target[i].w;
//        translation_trs[i] = trans[i].w;
//    }
//
//    rotation_tgt = rotation_tgt * rotation_trs;
//    translation_tgt = translation_tgt + translation_trs;
//
//    for (int i = 0; i < 3; i++) {
//        target[i].x = rotation_tgt[i].x;
//        target[i].y = rotation_tgt[i].y;
//        target[i].z = rotation_tgt[i].z;
//        target[i].w = translation_tgt[i];
//
//    }
//    return ;
//}
//
//// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
//// ---------------------------------------------------------------------------------------------------------
//void processInput(GLFWwindow* window)
//{
//    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
//        glfwSetWindowShouldClose(window, true);
//
//    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
//        camera.ProcessKeyboard(FORWARD, deltaTime);
//    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
//        camera.ProcessKeyboard(BACKWARD, deltaTime);
//    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
//        camera.ProcessKeyboard(LEFT, deltaTime);
//    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
//        camera.ProcessKeyboard(RIGHT, deltaTime);
//
//}
//
//void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) 
//{
//
//    if (mods == GLFW_MOD_CONTROL && action == GLFW_PRESS)
//        keepPressModControl = true;
//
//    if (action == GLFW_RELEASE)
//        keepPressModControl = false;
//
//}
//
//// glfw: whenever the window size changed (by OS or user resize) this callback function executes
//// ---------------------------------------------------------------------------------------------
//void framebuffer_size_callback(GLFWwindow* window, int width, int height)
//{
//    // make sure the viewport matches the new window dimensions; note that width and 
//    // height will be significantly larger than specified on retina displays.
//    glViewport(0, 0, width, height);
//}
//
//
//// glfw: whenever the mouse moves, this callback is called
//// -------------------------------------------------------
//void mouse_callback(GLFWwindow* window, double xpos, double ypos)
//{
//    if (firstMouse)
//    {
//        lastX = xpos;
//        lastY = ypos;
//        firstMouse = false;
//    }
//
//    float xoffset = xpos - lastX;
//    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
//
//    lastX = xpos;
//    lastY = ypos;
//
//    camera.ProcessMouseMovement(xoffset, yoffset);
//}
//
//void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos) {
//    //getting cursor position
//    float cursor_x = xpos;
//    float cursor_y = ypos;
//
//    if (!keepPressMouseLeft)
//    {
//        lastX = xpos;
//        lastY = ypos;
//
//    }
//    else if (keepPressModControl && keepPressModControl) {
//        float xoffset = xpos - lastX;
//        float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
//
//        lastX = xpos;
//        lastY = ypos;
//        targetmodel.ProcessMouseMovement(xoffset, yoffset, true);
//    }
//    else {
//        float xoffset = xpos - lastX;
//        float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
//
//        lastX = xpos;
//        lastY = ypos;
//        targetmodel.ProcessMouseMovement(xoffset, yoffset);
//    }
//}
//
//void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
//
//
//    //if (glfwGetKey(window, GLFW_MOD_CONTROL) == GLFW_REPEAT && button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_REPEAT)
//    //{
//    //    //getting cursor position
//    //    double xpos, ypos;
//    //    glfwGetCursorPos(window, &xpos, &ypos);
//
//    //    if (!keepPress)
//    //    {
//    //        lastX = xpos;
//    //        lastY = ypos;
//    //        keepPress = true;
//    //    }
//    //    else {
//    //        float xoffset = xpos - lastX;
//    //        float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
//
//    //        lastX = xpos;
//    //        lastY = ypos;
//    //        targetmodel.ProcessMouseMovement(xoffset, yoffset, true);
//    //    }
//    //}
//    //else 
//    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
//    {
//        keepPressMouseLeft = true;
//  
//    }
//    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
//    {
//        keepPressMouseLeft = false;
//    }
//}
//
//// glfw: whenever the mouse scroll wheel scrolls, this callback is called
//// ----------------------------------------------------------------------
//void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
//{
//    camera.ProcessMouseScroll(yoffset);
//}
//
//
//// animation function
//const unsigned int AnimeTime = 10;
//void customAnimation(glm::mat4& model, const float& current_time, const vector<glm::vec3>& posit)
//{
//    float postime = fmod(current_time, AnimeTime);
//    unsigned int stage = posit.size() - 1; // 运动轨迹(分段数)
//    // -------- 位置基于时间做线性插值 --------
//    float stageTime = AnimeTime / stage;
//    unsigned int index = (unsigned int)floor(postime / stageTime);
//    float stagePos = postime - index * stageTime; // stagetime
//    glm::vec3 speed = (posit[index + (unsigned int)1] - posit[index]) / stageTime; //方向，步长
//    model = glm::translate(model, posit[index] + stagePos * speed);
//}
//
