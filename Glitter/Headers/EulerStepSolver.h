//
//  EulerStepSolver.h
//  ClothingSimulation
//
//  Created by Jeehoon Hyun on 30/03/2019.
//  Copyright © 2019 Jeehoon Hyun. All rights reserved.
//

#ifndef EulerStepSolver_h
#define EulerStepSolver_h
#include "ParticleSystem.h"


#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

class EulerStepSolver{
private:
    unsigned int VAO, VBO, EBO;
    ParticleSystem * particleSystem;
    void formBuffer(){
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        
        glBufferData(GL_ARRAY_BUFFER, out_vertices.size() * sizeof(VERTEX), &out_vertices[0], GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, out_indices.size()*sizeof(unsigned int), &out_indices[0], GL_STATIC_DRAW);
        
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VERTEX), (void *)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(VERTEX), (void *)offsetof(VERTEX, uv_coord));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(VERTEX), (void *)offsetof(VERTEX, normal));
        
        glBindVertexArray(0);
    }
    //Modify render data(out_vertices) based on simulation data
    void modifyRenderData(){
        for(int i=0;i<particles.size();i++){
            for(int j=0;j<particles[i].vertexptr.size();j++){
                particles[i].vertexptr[j]->position = particles[i].position;
            }
        }
    }
    
    //Call this after our out_vertices is modified
    void modifyBuffer(){
        glBindVertexArray(VAO);
        glBufferData(GL_ARRAY_BUFFER, out_vertices.size() * sizeof(VERTEX), &out_vertices[0], GL_STATIC_DRAW);
        glBindVertexArray(0);
    }
    
    //Assign simulation data to render data
    void assignParticles(){
        for(int i=0;i<particles.size();i++){
            for(int j=0;j<out_vertices.size();j++){
                if(glm::distance(particles[i].position, out_vertices[j].position)==0.0){
                    particles[i].vertexptr.push_back(&out_vertices[j]);
                }
            }
        }
    }
    void scaleVector(std::vector<ParticleDimensionHolder>& temp1, float deltaT){
        for(int i=0;i<temp1.size();i++){
            //now this is delta x
            temp1[i].x_v = temp1[i].x_v * deltaT;
            //now this is delta v
            temp1[i].v_a = temp1[i].v_a * deltaT;
        }
    }
    //add temp1 and temp2 and store it in temp2
    void addVectors(std::vector<ParticleDimensionHolder>& temp1, std::vector<ParticleDimensionHolder>& temp2){
        for(int i=0;i<temp1.size();i++){
            temp2[i].x_v = temp2[i].x_v + temp1[i].x_v;
            temp2[i].v_a = temp2[i].v_a + temp1[i].v_a;
        }
    }
public:
    std::vector<Particle> particles;
    std::vector<VERTEX> out_vertices;
    std::vector<unsigned int> out_indices;
    std::vector<unsigned int> original_vertex_indices;
    EulerStepSolver(std::vector<Particle> particles, std::vector<VERTEX> out_vertices, std::vector<unsigned int> out_indices, std::vector<unsigned int> original_vertex_indices){
        this->particles = particles;
        this->out_vertices = out_vertices;
        this->out_indices = out_indices;
        this->original_vertex_indices = original_vertex_indices;
        assignParticles();
        formBuffer();
        particleSystem = new ParticleSystem(&(this->particles), &(this->original_vertex_indices));
    }
    //The most important step!
    void EulerStep(float deltaT){
        particleSystem->clearForce();
        particleSystem->computeForces();
        //std::cout << "particleSystem force of random particle" << glm::to_string((*particleSystem->particles)[90].force) << std::endl;
        std::vector<ParticleDimensionHolder> temp1 = particleSystem->getDerivative();
        scaleVector(temp1, deltaT);
        std::vector<ParticleDimensionHolder> temp2 = particleSystem->getState();
        addVectors(temp1, temp2);
        //particleSystem->setState(temp2);
        
        //collision check
        particleSystem->resolveCollision(temp2);
        
        //This is just the pseudo-code according to the pixar slides.
        //particleSystem->checkCollision();
        /* //These parts are not necessary for my program. 이건 솔직히 완전 비탄성 충돌일때만 해당하는 얘기지ㅋㅋㅋㅋ거의 가능한 게 아님.
        //particleSystem->revertSystemToCollisionMoment();
        //particleSystem->modifyV(); //For each particle that collided. slide C page 22
        //if it's in 'contact' where dot(N,v) < epsilon, then add contact force. also add frictional force too.
         */
        particleSystem->setState(temp2);
        
        modifyRenderData();
    }
    
    void draw(Shader ourShader){
        ourShader.setVec3("lightDirection", glm::vec3(0.2f, 1.0f, 0.3f));
        //Explodes at 0.002
        EulerStep(0.005);
        modifyBuffer();
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, out_indices.size(), GL_UNSIGNED_INT, 0);
    }
};

#endif /* EulerStepSolver_h */
