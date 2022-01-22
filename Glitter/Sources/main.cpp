#define GLM_ENABLE_EXPERIMENTAL
#include "BoneRig.h"
#include <iostream>
#include <string>
#include <sstream>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
//void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void processInput(GLFWwindow *window);
void drawBones(Shader& shader, Joint * rootJoint, glm::mat4 model);
// settings
const unsigned int SCR_WIDTH = 640;
const unsigned int SCR_HEIGHT = 480;

// camera
glm::vec3 cameraLoc = glm::vec3(0.0f, 50.0f, 100.0f);
Camera camera(cameraLoc);
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool comeBackFromMouseMode = false;

// timing
//deltaTime is for mouse speed
double deltaTime = 0;
//deltaTime2 is for FPS speed
double deltaTime2 = 0;
double lastFrame = 0;
//frameIndex is for reading in the BVH file's i-th frame data
int frameIndex = 0;
//FPSShower shows how many frames passed by for 1 second.
int FPSShower = 0;
//How many times are the render settings updated?
int update = 0;
double timer = 0;
unsigned int * VISITED;
bool resetMatrices = false;
bool mouseMode = false;
bool armMode = true;
bool upperBodyMode = false;
bool changeHand = false;
bool armMode_attempted = false;
bool upperBodyMode_attempted = false;
double limitFPS = 0;

BoneRig Rig;
//Target Position
glm::vec3 target= glm::vec3(20.0f, 50.0f, 40.0f);
//Target Orientation
float theta = -90.0f;
glm::vec3 v_unit = glm::vec3(1.0f, 0.0f, 0.0f);
glm::quat ori_d = glm::quat(glm::cos(glm::radians(theta/2)), v_unit * glm::sin(glm::radians(theta/2)));

void printManual(){
    std::cout << "WASD, Move mouse: Look around and move\nR: Reset model back to default\nM: Mouse Mode(Fix the screen and make mouse visible)\nN: Exit Mouse Mode\n1: IK with just the arm\n2: IK with the whole upper part of body\nH: Change hand position and orientation\nEsc: Exit" << std::endl;
}

int main(int argc, char **argv)
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // uncomment this statement to fix compilation on OS X
#endif
    
    // glfw window creation
    // --------------------
    
    
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "IKSolver", NULL, NULL);
    
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetKeyCallback(window, key_callback);
    //glfwSetMouseButtonCallback(window, mouse_button_callback);
    
    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    
    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }
    
    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    
    Rig.setHierarchy();
    Rig.setBoneVAOs();
    
    // build and compile shaders
    Shader shader("AnimationVertexShader.vs", "AnimationFragmentShader.fs");
    
    shader.use();
    // render loop
    // -----------
    
    printManual();
    
    limitFPS = 0.008342;
    lastFrame = glfwGetTime();
    timer = lastFrame;
    VISITED = new unsigned int[24];
    
    while (!glfwWindowShouldClose(window))
    {
        // per-frame time logic
        // --------------------
        for (int i = 0; i < 24; i++) {
            VISITED[i] = 0;
        }
        
        //get nowTime
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        deltaTime2 += (currentFrame - lastFrame) / limitFPS;
        lastFrame = currentFrame;
        
        processInput(window);
        
        
        // update settings here
        while (deltaTime2 >= 1.0) {
            // render
            // ------
            
            
            glClearColor(0.9f, 0.9f, 0.9f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            
            shader.setVec3("light.direction", 0.0f, -0.1f, 0.0f);
            shader.setVec3("viewPos", camera.Position);
            shader.setVec3("light.ambient", 0.3f, 0.3f, 0.2f);
            shader.setVec3("light.diffuse", 0.7f, 0.7f, 0.7f);
            shader.setVec3("light.specular", 1.0f, 1.0f, 1.0f);
            shader.setVec3("material.ambient", 0.2f, 0.9f, 0.0f);
            shader.setVec3("material.diffuse", 0.2f, 0.9f, 0.0f);
            shader.setVec3("material.specular", 0.2f, 0.9f, 0.0f); // specular lighting doesn't have full effect on this object's material
            shader.setFloat("material.shininess", 32.0f);
            if(armMode && !armMode_attempted ){
                Rig.solveIKArm(target, ori_d);
                armMode_attempted = true;
                //std::cout << "arm IK Solved" << std::endl;
            }
            else if(upperBodyMode && !upperBodyMode_attempted){
                Rig.solveIKUpperBody(target, ori_d);
                upperBodyMode_attempted = true;
                //std::cout << "upperBody IK Solved" << std::endl;
            }

            if (resetMatrices) {
                Rig.resetMatrices();
                resetMatrices = false;
                frameIndex = 0;
            }

            if(changeHand && mouseMode){
                std::cout<< "Current hand position: "<< target.x<<" "<<target.y<<" "<<target.z<< "\nChange position to (input all target positions and press enter): ";
                std::string line;
                std::getline(std::cin, line);
                std::istringstream ss(line);
                float temp1, temp2, temp3;
                ss >> temp1;
                ss >> temp2;
                ss >> temp3;
                target = glm::vec3(temp1, temp2, temp3);
                std::cout << "Current theta: "<<theta<<"\nCurrent v_unit: "<<v_unit.x<<" "<<v_unit.y<<" "<<v_unit.z;
                std::cout << "\nChange current theta and v_unit to (put all 4 components and press enter): ";
                std::getline(std::cin, line);
                std::istringstream ss2(line);
                ss2 >> theta;
                ss2 >> temp1;
                ss2 >> temp2;
                ss2 >> temp3;
                v_unit = glm::vec3(temp1, temp2, temp3);
                ori_d = glm::quat(glm::cos(glm::radians(theta/2)), v_unit * glm::sin(glm::radians(theta/2)));
                changeHand = false;
                armMode = true;
                upperBodyMode = false;
                armMode_attempted = false;
                upperBodyMode_attempted = false;
            }
            
            //update will probably be 120.
            update++;
            deltaTime2--;
        }
        
        //Render here. FPSShower shows maximum possible frames that were renderable!
        glm::mat4 model = glm::mat4(1.0f);
        drawBones(shader, &Rig.Joints[0], model);
        FPSShower++;
        
        //Shows how many frames passed for 1 second!
        if (glfwGetTime() - timer > 1.0) {
            timer++;
            //std::cout << "FPS: " << FPSShower << " Updated: " << update << " times" << std::endl;
            FPSShower = 0;
            update = 0;
        }
        
        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    
    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}


void drawBones(Shader& shader, Joint * rootJoint, glm::mat4 model) {
    //model = model * rootJoint->returnT() * glm::toMat4(rootJoint->returnR_quat());
    model = model * rootJoint->returnT() * glm::toMat4(rootJoint->returnR_quat());
    glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 1000.0f);
    glm::mat4 view = camera.GetViewMatrix();
    shader.setMat4("model", model);
    shader.setMat4("projection", projection);
    shader.setMat4("view", view);
    for (int i = 0; i < rootJoint->returnnChildren(); i++) {
        glBindVertexArray(rootJoint->returnVAOs(i));
        glDrawArrays(GL_TRIANGLES, 0, 36);
    }
    VISITED[rootJoint->returnJointID()] = 1;
    for (int i = 0; i < rootJoint->returnnChildren(); i++) {
        if (!VISITED[rootJoint->i_thChild(i)->returnJointID()]){
            drawBones(shader, rootJoint->i_thChild(i), model);
        }
    }
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
    if(mouseMode == false) {
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            camera.ProcessKeyboard(FORWARD, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            camera.ProcessKeyboard(BACKWARD, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            camera.ProcessKeyboard(LEFT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            camera.ProcessKeyboard(RIGHT, deltaTime);
    }
}

//Handle key every just once! (processInput handles key presses multiple times)
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (key == GLFW_KEY_R && action == GLFW_PRESS)
        resetMatrices = true;

    if (key == GLFW_KEY_M && action == GLFW_PRESS) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        mouseMode = true;
    }

    if (key == GLFW_KEY_N && action == GLFW_PRESS) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        if (mouseMode == true)
            comeBackFromMouseMode = true;
        mouseMode = false;
    }

    if (key == GLFW_KEY_1 && action == GLFW_PRESS) {
        //Rig.clear();
        std::cout << "move the arm" << std::endl;
        armMode = true; //Move just the arm
        upperBodyMode = false;
        armMode_attempted = false;
    }
    if (key == GLFW_KEY_2 && action == GLFW_PRESS) {
        std::cout << "move the upper body" << std::endl;
        upperBodyMode = true;
        armMode = false; //Now move the full body
        upperBodyMode_attempted = false;
    }
    if (key == GLFW_KEY_H && action == GLFW_PRESS) {
        //Rig.clear();
        changeHand = true;
    }
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if (comeBackFromMouseMode)
    {
        glfwSetCursorPos(window, lastX, lastY);
        comeBackFromMouseMode = false;
    }
    if(mouseMode == false){
        float xoffset = xpos - lastX;
        float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
    
        lastX = xpos;
        lastY = ypos;
        camera.ProcessMouseMovement(xoffset, yoffset);
    }
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
}