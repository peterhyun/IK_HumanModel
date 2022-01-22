//
//  Particle.h
//  ClothingSimulation
//
//  Created by Jeehoon Hyun on 29/03/2019.
//  Copyright © 2019 Jeehoon Hyun. All rights reserved.
//

#ifndef Particle_h
#define Particle_h

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <vector>

//This is the RENDER vertex data.
class VERTEX{
public:
    glm::vec3 position;
    glm::vec2 uv_coord;
    glm::vec3 normal;
};

//This is the physics vertex data.
class Particle{
public:
    glm::vec3 position;
    glm::vec3 velocity;
    glm::vec3 force;
    float mass;
    //This is for modifying the rendering data that shares the same 'position' value as the Particle.
    std::vector<VERTEX *> vertexptr;
};


#endif /* Particle_h */
