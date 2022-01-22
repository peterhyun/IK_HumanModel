//
//  Joint.h
//  ComputerAnimation
//
//  Created by Jeehoon Hyun on 27/10/2018.
//  Copyright © 2018 Jeehoon Hyun. All rights reserved.
//

#ifndef Joint_h
#define Joint_h
//The unit node of the rig

#include "Shader.h"
#include "Camera.h"
#include <GLFW/glfw3.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <cstring>

const int MAX = 3;

class Joint {
private:
    char * JointName;
    int JointID;
    //Rotation needed from parent
    glm::mat4 R;
    //Same Rotation, quaternion version
    glm::quat R_quat;
    glm::mat4 T;
    
    Joint * childJoint[MAX];
    Joint * parentJoint;
    
    int nChildren = 0;
public:
    float x, y, z;    //THIS Should be translated for intial T
    int * channelOrder;
    unsigned int * VAOs;
    
    Joint (int JointID=0, float x=0, float y=0, float z=0) {
        this->JointID = JointID;
        this->R = glm::mat4(1.0f);
        
        this->R_quat = glm::quat(1.0,0.0,0.0,0.0);
        
        this->T = glm::mat4(1.0f);
        this->x = x;
        this->y = y;
        this->z = z;
    }
    void setJointID(int JointID){
        this->JointID = JointID;
    }
    
    void setFloats(float x, float y, float z){
        this->x = x;
        this->y = y;
        this->z = z;
    }
    
    void setName(const char * JointName) {
        size_t length = strlen(JointName) + 1;
        this->JointName = new char[length];
        strcpy(this->JointName, JointName);
    }
    
    void setVAOs();
    
    void setChild(Joint * child) {
        childJoint[nChildren] = child;
        nChildren++;
    }

    void setParent(Joint* parent) {
        parentJoint = parent;
    }
    
    void setR(glm::mat4 R) {
        this->R = R;
    }
    
    void setR_quat(glm::quat R_quat){
        this->R_quat = R_quat;
    }
    
    void setT(glm::mat4 T) {
        this->T = T;
    }
    
    void setT() {
        glm::mat4 temp = glm::mat4(1.0f);
        this->T = glm::translate(temp,glm::vec3(x,y,z));
    }
    
    glm::mat4 returnR() {
        return R;
    }
    
    glm::quat returnR_quat(){
        return R_quat;
    }
    
    glm::mat4 returnT() {
        return T;
    }
    
    const char * returnJointName() {
        return JointName;
    }
    int returnJointID() {
        return JointID;
    }
    int returnnChildren() {
        return nChildren;
    }

    Joint* returnParent() {
        return parentJoint;
    }
    Joint * i_thChild(int index) {
        return childJoint[index];
    }
    unsigned int returnVAOs(int i) {
        if (i >= nChildren) {
            std::cout << "ERROR. THAT VAO DOES NOT EXIST" << std::endl;
            return NULL;
        }
        else {
            //std::cout <<"For "<<JointName<<", " << i<<"_th VAO is about to be returned"<< std::endl;
            return VAOs[i];
        }
    }
};

void Joint::setVAOs() {
    VAOs = new unsigned int[nChildren];
    //std::cout << "Making VAOs, for " << JointName << "nChildren is " << nChildren << std::endl;
    unsigned int * VBOs = new unsigned int[nChildren];
    glm::vec3 parentPos = glm::vec3(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < nChildren; i++) {
        float a = childJoint[i]->x;
        float b = childJoint[i]->y;
        float c = childJoint[i]->z;
        glm::vec3 childPos = glm::vec3(a, b, c);
        glm::vec3 parent2Child = childPos - parentPos;
        glm::vec3 cross1 = normalize(glm::cross(parent2Child, glm::vec3(0.0f, 0.1f, -1.0f)));    //0.0f, 0.1f, -1.0f leads to NaN of foot and toes cause cross product becomes zero.
        glm::vec3 cross2 = normalize(glm::cross(parent2Child, cross1));
        glm::vec3 minus1 = glm::vec3(-cross1.x, -cross1.y, -cross1.z);
        glm::vec3 minus2 = glm::vec3(-cross2.x, -cross2.y, -cross2.z);
        float thick = 2.0f;
        float vertices[] = {
            //Back Face
            parentPos.x + thick * cross1.x + thick * minus2.x, parentPos.y + thick * cross1.y + thick * minus2.y, parentPos.z + thick * cross1.z + thick * minus2.z,
            childPos.x + thick * minus1.x + thick * minus2.x, childPos.y + thick * minus1.y + thick * minus2.y, childPos.z + thick * minus1.z + thick * minus2.z,
            parentPos.x + thick * minus1.x + thick * minus2.x, parentPos.y + thick * minus1.y + thick * minus2.y, parentPos.z + thick * minus1.z + thick * minus2.z,
            childPos.x + thick * minus1.x + thick * minus2.x, childPos.y + thick * minus1.y + thick * minus2.y, childPos.z + thick * minus1.z + thick * minus2.z,
            parentPos.x + thick * cross1.x + thick * minus2.x, parentPos.y + thick * cross1.y + thick * minus2.y, parentPos.z + thick * cross1.z + thick * minus2.z,
            childPos.x + thick * cross1.x + thick * minus2.x, childPos.y + thick * cross1.y + thick * minus2.y, childPos.z + thick * cross1.z + thick * minus2.z,
            
            //Front face
            parentPos.x + thick * cross1.x + thick * cross2.x, parentPos.y + thick * cross1.y + thick * cross2.y, parentPos.z + thick * cross1.z + thick * cross2.z,
            parentPos.x + thick * minus1.x + thick * cross2.x, parentPos.y + thick * minus1.y + thick * cross2.y, parentPos.z + thick * minus1.z + thick * cross2.z,
            childPos.x + thick * minus1.x + thick * cross2.x, childPos.y + thick * minus1.y + thick * cross2.y, childPos.z + thick * minus1.z + thick * cross2.z,
            childPos.x + thick * minus1.x + thick * cross2.x, childPos.y + thick * minus1.y + thick * cross2.y, childPos.z + thick * minus1.z + thick * cross2.z,
            childPos.x + thick * cross1.x + thick * cross2.x, childPos.y + thick * cross1.y + thick * cross2.y, childPos.z + thick * cross1.z + thick * cross2.z,
            parentPos.x + thick * cross1.x + thick * cross2.x, parentPos.y + thick * cross1.y + thick * cross2.y, parentPos.z + thick * cross1.z + thick * cross2.z,
            
            //Left face
            childPos.x + thick * cross1.x + thick * cross2.x, childPos.y + thick * cross1.y + thick * cross2.y, childPos.z + thick * cross1.z + thick * cross2.z,
            childPos.x + thick * cross1.x + thick * minus2.x, childPos.y + thick * cross1.y + thick * minus2.y, childPos.z + thick * cross1.z + thick * minus2.z,
            parentPos.x + thick * cross1.x + thick * minus2.x, parentPos.y + thick * cross1.y + thick * minus2.y, parentPos.z + thick * cross1.z + thick * minus2.z,
            parentPos.x + thick * cross1.x + thick * minus2.x, parentPos.y + thick * cross1.y + thick * minus2.y, parentPos.z + thick * cross1.z + thick * minus2.z,
            parentPos.x + thick * cross1.x + thick * cross2.x, parentPos.y + thick * cross1.y + thick * cross2.y, parentPos.z + thick * cross1.z + thick * cross2.z,
            childPos.x + thick * cross1.x + thick * cross2.x, childPos.y + thick * cross1.y + thick * cross2.y, childPos.z + thick * cross1.z + thick * cross2.z,
            
            //Right face
            childPos.x + thick * minus1.x + thick * cross2.x, childPos.y + thick * minus1.y + thick * cross2.y, childPos.z + thick * minus1.z + thick * cross2.z,
            parentPos.x + thick * minus1.x + thick * minus2.x, parentPos.y + thick * minus1.y + thick * minus2.y, parentPos.z + thick * minus1.z + thick * minus2.z,
            childPos.x + thick * minus1.x + thick * minus2.x, childPos.y + thick * minus1.y + thick * minus2.y, childPos.z + thick * minus1.z + thick * minus2.z,
            parentPos.x + thick * minus1.x + thick * minus2.x, parentPos.y + thick * minus1.y + thick * minus2.y, parentPos.z + thick * minus1.z + thick * minus2.z,
            childPos.x + thick * minus1.x + thick * cross2.x, childPos.y + thick * minus1.y + thick * cross2.y, childPos.z + thick * minus1.z + thick * cross2.z,
            parentPos.x + thick * minus1.x + thick * cross2.x, parentPos.y + thick * minus1.y + thick * cross2.y, parentPos.z + thick * minus1.z + thick * cross2.z,
            
            //Bottom face
            parentPos.x + thick * cross1.x + thick * minus2.x, parentPos.y + thick * cross1.y + thick * minus2.y, parentPos.z + thick * cross1.z + thick * minus2.z,
            parentPos.x + thick * minus1.x + thick * minus2.x, parentPos.y + thick * minus1.y + thick * minus2.y, parentPos.z + thick * minus1.z + thick * minus2.z,
            parentPos.x + thick * minus1.x + thick * cross2.x, parentPos.y + thick * minus1.y + thick * cross2.y, parentPos.z + thick * minus1.z + thick * cross2.z,
            parentPos.x + thick * minus1.x + thick * cross2.x, parentPos.y + thick * minus1.y + thick * cross2.y, parentPos.z + thick * minus1.z + thick * cross2.z,
            parentPos.x + thick * cross1.x + thick * cross2.x, parentPos.y + thick * cross1.y + thick * cross2.y, parentPos.z + thick * cross1.z + thick * cross2.z,
            parentPos.x + thick * cross1.x + thick * minus2.x, parentPos.y + thick * cross1.y + thick * minus2.y, parentPos.z + thick * cross1.z + thick * minus2.z,
            
            //Top face
            childPos.x + thick * cross1.x + thick * minus2.x, childPos.y + thick * cross1.y + thick * minus2.y, childPos.z + thick * cross1.z + thick * minus2.z,
            childPos.x + thick * minus1.x + thick * cross2.x, childPos.y + thick * minus1.y + thick * cross2.y, childPos.z + thick * minus1.z + thick * cross2.z,
            childPos.x + thick * minus1.x + thick * minus2.x, childPos.y + thick * minus1.y + thick * minus2.y, childPos.z + thick * minus1.z + thick * minus2.z,
            childPos.x + thick * minus1.x + thick * cross2.x, childPos.y + thick * minus1.y + thick * cross2.y, childPos.z + thick * minus1.z + thick * cross2.z,
            childPos.x + thick * cross1.x + thick * minus2.x, childPos.y + thick * cross1.y + thick * minus2.y, childPos.z + thick * cross1.z + thick * minus2.z,
            childPos.x + thick * cross1.x + thick * cross2.x, childPos.y + thick * cross1.y + thick * cross2.y, childPos.z + thick * cross1.z + thick * cross2.z
        };
        float normals[] = {
            minus2.x, minus2.y, minus2.z,
            minus2.x, minus2.y, minus2.z,
            minus2.x, minus2.y, minus2.z,
            minus2.x, minus2.y, minus2.z,
            minus2.x, minus2.y, minus2.z,
            minus2.x, minus2.y, minus2.z,
            
            cross2.x, cross2.y, cross2.z,
            cross2.x, cross2.y, cross2.z,
            cross2.x, cross2.y, cross2.z,
            cross2.x, cross2.y, cross2.z,
            cross2.x, cross2.y, cross2.z,
            cross2.x, cross2.y, cross2.z,
            
            cross1.x, cross1.y, cross1.z,
            cross1.x, cross1.y, cross1.z,
            cross1.x, cross1.y, cross1.z,
            cross1.x, cross1.y, cross1.z,
            cross1.x, cross1.y, cross1.z,
            cross1.x, cross1.y, cross1.z,
            
            minus1.x, minus1.y, minus1.z,
            minus1.x, minus1.y, minus1.z,
            minus1.x, minus1.y, minus1.z,
            minus1.x, minus1.y, minus1.z,
            minus1.x, minus1.y, minus1.z,
            minus1.x, minus1.y, minus1.z,
            
            -parent2Child.x, -parent2Child.y, -parent2Child.z,
            -parent2Child.x, -parent2Child.y, -parent2Child.z,
            -parent2Child.x, -parent2Child.y, -parent2Child.z,
            -parent2Child.x, -parent2Child.y, -parent2Child.z,
            -parent2Child.x, -parent2Child.y, -parent2Child.z,
            -parent2Child.x, -parent2Child.y, -parent2Child.z,
            
            parent2Child.x, parent2Child.y, parent2Child.z,
            parent2Child.x, parent2Child.y, parent2Child.z,
            parent2Child.x, parent2Child.y, parent2Child.z,
            parent2Child.x, parent2Child.y, parent2Child.z,
            parent2Child.x, parent2Child.y, parent2Child.z,
            parent2Child.x, parent2Child.y, parent2Child.z
        };
        glGenVertexArrays(1, &VAOs[i]);
        glGenBuffers(1, &VBOs[i]);
        glBindVertexArray(VAOs[i]);
        glBindBuffer(GL_ARRAY_BUFFER, VBOs[i]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 216, NULL, GL_STATIC_DRAW);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * 108, vertices);
        glBufferSubData(GL_ARRAY_BUFFER, sizeof(float) * 108, sizeof(float) * 108, normals);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)(sizeof(float)*108));
    }
}

#endif
