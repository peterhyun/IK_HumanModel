#define GLM_ENABLE_EXPERIMENTAL
#include "BoneRig.h"
#include <iostream>
#include <Eigen/Dense>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);
void drawBones(Shader& shader, Joint * rootJoint, glm::mat4 model);
// settings
const unsigned int SCR_WIDTH = 2560;
const unsigned int SCR_HEIGHT = 1920;

// camera
glm::vec3 cameraLoc = glm::vec3(0.0f, 50.0f, 100.0f);
Camera camera(cameraLoc);
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

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
double limitFPS = 0;

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
    
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "BVHParser", NULL, NULL);
    
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
    
    BoneRig Rig;
    Rig.setHierarchy();
    Rig.setBoneVAOs();
    
    // build and compile shaders
    Shader shader("AnimationVertexShader.txt", "AnimationFragmentShader.txt");
    
    shader.use();
    // render loop
    // -----------
    
    limitFPS = 0.008342;
    lastFrame = glfwGetTime();
    timer = lastFrame;
    VISITED = new unsigned int[24];
    
    //Test an end effector
    glm::vec3 target= glm::vec3(15.0f, 20.0f, 35.0f);
    //Target Orientation
    float theta = 30.0f;
    glm::vec3 v_unit = glm::vec3(0.0f, 0.0f, 1.0f);
    glm::quat ori_d = glm::quat(glm::cos(glm::radians(theta/2)), v_unit * glm::sin(glm::radians(theta/2)));
    
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
            
            if (mouseMode == false){
                Rig.solveIK(target, ori_d);
            }
            else {
                //Do the clicking calculations here
                //Screen does not move, but only the mouse does.
            }
            if (resetMatrices == true) {
                Rig.resetMatrices();
                resetMatrices = false;
                frameIndex = 0;
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
            std::cout << "FPS: " << FPSShower << " Updated: " << update << " times" << std::endl;
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
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS)
        resetMatrices = true;
    if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS)
        mouseMode = true;
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
    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }
    
    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
    
    lastX = xpos;
    lastY = ypos;
    
    camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
}
