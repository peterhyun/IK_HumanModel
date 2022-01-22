//
//  ParticleSystem.hpp
//  ClothingSimulation
//
//  Created by Jeehoon Hyun on 29/03/2019.
//  Copyright © 2019 Jeehoon Hyun. All rights reserved.
//

#ifndef ParticleSystem_hpp
#define ParticleSystem_hpp

#include <vector>
#include <algorithm>
#include <iostream>

#include "Particle.h"
//Represents Connection between two indices
float k_s = 19.0;
float k_d = 0.82;
float airResistance = 0.32;

glm::vec3 sphereCenter = glm::vec3();

class Connection{
public:
    int index1;
    int index2;
    float restLength;
    Connection(int index1, int index2, float restLength){
        this->index1 = index1;
        this->index2 = index2;
        this->restLength = restLength;
    }
    bool operator==(Connection connection){
        return ((connection.index1 == this->index2) && (connection.index2 == this->index1)) || ((connection.index1 == this->index1) && (connection.index2 == this->index2));
    }
};

class ParticleDimensionHolder{
public:
    glm::vec3 x_v;
    glm::vec3 v_a;
};

class ParticleSystem{
public:
    //Get it from particles from OBJReader
    std::vector<Particle> * particles;
    std::vector<unsigned int> * original_vertex_indices;
    //Connects the particles to form multiple springs
    ParticleSystem(std::vector<Particle> * particles, std::vector<unsigned int> * original_vertex_indices){
        this->particles = particles;
        this->original_vertex_indices = original_vertex_indices;
        setSprings();
        for(int i=0;i<connections.size();i++){
            if(connections[i].index1==5){
                std::cout << "index "<< connections[i].index1 << " is connected to "<< connections[i].index2 << std::endl;
            }
            if(connections[i].index2==5){
                std::cout << "(reverse)index "<< connections[i].index2 << " is connected to "<< connections[i].index1 << std::endl;
            }
        }
        
        
    }
    void clearForce(){
        for(int i=0;i<particles->size();i++){
            (*particles)[i].force = glm::vec3(0.0);
        }
    }
    void computeForces(){
        computeGravity();
        computeDampedSpringForce();
        computeAirResistance();
        //Fix some parts to the original location so the clothing doesn't move.
        (*particles)[962].force = glm::vec3(0.0);
        (*particles)[963].force = glm::vec3(0.0);
        (*particles)[2571].force = glm::vec3(0.0);
        (*particles)[2570].force = glm::vec3(0.0);
    }
    std::vector<ParticleDimensionHolder> getDerivative(){
        std::vector<ParticleDimensionHolder> pdhVector;
        for(int i=0;i<particles->size();i++){
            ParticleDimensionHolder pdh;
            pdh.x_v = (*particles)[i].velocity;
            pdh.v_a = (*particles)[i].force/(*particles)[i].mass;
            pdhVector.push_back(pdh);
        }
        return pdhVector;
    }
    std::vector<ParticleDimensionHolder> getState(){
        std::vector<ParticleDimensionHolder> pdhVector;
        for(int i=0;i<particles->size();i++){
            ParticleDimensionHolder pdh;
            pdh.x_v = (*particles)[i].position;
            pdh.v_a = (*particles)[i].velocity;
            pdhVector.push_back(pdh);
        }
        return pdhVector;
    }
    void setState(std::vector<ParticleDimensionHolder>& pdhVector){
        for(int i=0;i<pdhVector.size();i++){
            (*particles)[i].position = pdhVector[i].x_v;
            (*particles)[i].velocity = pdhVector[i].v_a;
        }
    }
    
    void resolveCollision(std::vector<ParticleDimensionHolder>& pdhVector){
        //This will be the center of the sphere
        glm::vec3 COM = glm::vec3(0.716704, 76.082527, -0.905833);
        float radius = 18.5;
        for(int i=0;i<pdhVector.size();i++){
            glm::vec3 center2Particle = pdhVector[i].x_v - COM;
            float length = glm::length(center2Particle);
            glm::vec3 n_center2Particle = glm::normalize(center2Particle);
            if(length < radius){
                pdhVector[i].x_v = pdhVector[i].x_v + (float)(radius-length) * n_center2Particle;
                glm::vec3 v = pdhVector[i].v_a;
                glm::vec3 v_n = glm::dot(n_center2Particle, v) * n_center2Particle;
                glm::vec3 v_t = v-v_n;
                pdhVector[i].v_a = v_t - (0.4f*v_n);
            }
        }
    }
    
private:
    std::vector<Connection> connections;
    /*
    int closestParticle(int i){
        //implement this! Return closest particle. (except itself)
        int closestIndex = -1;
        int closestDistance = 100000;
        for(int j=0;j<2590;j++){
            if((j!=i) && glm::distance((*particles)[i].position, (*particles)[j].position) < closestDistance){
                closestIndex = j;
                closestDistance = glm::distance((*particles)[i].position, (*particles)[j].position);
            }
        }
        return closestIndex;
    }
    */
    void setSprings(){
        //There are 2590 particles. Each particle should have at least 1 spring attached to it to a nearby particle!
        for(int i=0;i<original_vertex_indices->size();i+=3){
            unsigned int first_vertex_index = (*original_vertex_indices)[i]-1;
            unsigned int second_vertex_index = (*original_vertex_indices)[i+1]-1;
            unsigned int third_vertex_index = (*original_vertex_indices)[i+2]-1;
            Connection connection1(first_vertex_index, second_vertex_index, glm::distance((*particles)[first_vertex_index].position, (*particles)[second_vertex_index].position));
            Connection connection2(second_vertex_index, third_vertex_index, glm::distance((*particles)[second_vertex_index].position, (*particles)[third_vertex_index].position));
            Connection connection3(third_vertex_index, first_vertex_index, glm::distance((*particles)[third_vertex_index].position, (*particles)[first_vertex_index].position));
            bool isRedundant1 = false;
            bool isRedundant2 = false;
            bool isRedundant3 = false;
            //Check redundancy among previous connections
            for(int j=0;j<connections.size();j++){
                if(connection1 == connections[j]){
                    //std::cout << "is redundant!" << std::endl;
                    isRedundant1 = true;
                }
                if(connection2 == connections[j]){
                    //std::cout << "is redundant!" << std::endl;
                    isRedundant2 = true;
                }
                if(connection3 == connections[j]){
                    //std::cout << "is redundant!" << std::endl;
                    isRedundant3 = true;
                }
                if(isRedundant1 && isRedundant2 && isRedundant3){
                    break;
                }
            }
            if(!isRedundant1){
                //std::cout << connection1.index1 << " and " << connection1.index2 << "connected." << std::endl;
                connections.push_back(connection1);
            }
            if(!isRedundant2){
                //std::cout << connection2.index1 << " and " << connection2.index2 << "connected." << std::endl;
                connections.push_back(connection2);
            }
            if(!isRedundant3){
                //std::cout << connection3.index1 << " and " << connection3.index2 << "connected." << std::endl;
                connections.push_back(connection3);
            }
            //We have to connect one more remember?
            connectDiagonal(i);
        }
    }
    void connectDiagonal(int i){
        unsigned int first_vertex_index = (*original_vertex_indices)[i]-1;
        unsigned int second_vertex_index = (*original_vertex_indices)[i+1]-1;
        unsigned int third_vertex_index = (*original_vertex_indices)[i+2]-1;
        //First for the first_vertex_index
        for(int j=i+3;j<original_vertex_indices->size();j+=3){
            unsigned int first_vertex_index_prime = (*original_vertex_indices)[j]-1;
            unsigned int second_vertex_index_prime = (*original_vertex_indices)[j+1]-1;
            unsigned int third_vertex_index_prime = (*original_vertex_indices)[j+2]-1;
            if((((second_vertex_index == second_vertex_index_prime) && (third_vertex_index == third_vertex_index_prime)) || ((second_vertex_index == third_vertex_index_prime) && (third_vertex_index == second_vertex_index_prime))) && (first_vertex_index != first_vertex_index_prime)){
                Connection connection(first_vertex_index, first_vertex_index_prime, glm::distance((*particles)[first_vertex_index].position, (*particles)[first_vertex_index_prime].position));
                std::cout << connection.index1 << " and " << connection.index2 << " connected." << std::endl;
                bool isRedundant = false;
                for(int j=0;j<connections.size();j++){
                    if(connection == connections[j]){
                        std::cout << "is redundant!" << std::endl;
                        isRedundant = true;
                        break;
                    }
                }
                if(!isRedundant){
                    connections.push_back(connection);
                }
                
            }
            if((((second_vertex_index == first_vertex_index_prime) && (third_vertex_index == third_vertex_index_prime)) || ((second_vertex_index == third_vertex_index_prime) && (third_vertex_index == first_vertex_index_prime))) && (first_vertex_index != second_vertex_index_prime)){
                Connection connection(first_vertex_index, second_vertex_index_prime, glm::distance((*particles)[first_vertex_index].position, (*particles)[second_vertex_index_prime].position));
                std::cout << connection.index1 << " and " << connection.index2 << " connected." << std::endl;
                bool isRedundant = false;
                for(int j=0;j<connections.size();j++){
                    if(connection == connections[j]){
                        std::cout << "is redundant!" << std::endl;
                        isRedundant = true;
                        break;
                    }
                }
                if(!isRedundant){
                    connections.push_back(connection);
                }
            }
            if((((second_vertex_index == first_vertex_index_prime) && (third_vertex_index == second_vertex_index_prime)) || ((second_vertex_index == second_vertex_index_prime) && (third_vertex_index == first_vertex_index_prime))) && (first_vertex_index != third_vertex_index_prime)){
                Connection connection(first_vertex_index, third_vertex_index_prime, glm::distance((*particles)[first_vertex_index].position, (*particles)[third_vertex_index_prime].position));
                std::cout << connection.index1 << " and " << connection.index2 << " connected." << std::endl;
                bool isRedundant = false;
                for(int j=0;j<connections.size();j++){
                    if(connection == connections[j]){
                        std::cout << "is redundant!" << std::endl;
                        isRedundant = true;
                        break;
                    }
                }
                if(!isRedundant){
                    connections.push_back(connection);
                }
            }
            if((((first_vertex_index == second_vertex_index_prime) && (third_vertex_index == third_vertex_index_prime)) || ((first_vertex_index == third_vertex_index_prime) && (third_vertex_index == second_vertex_index_prime))) && (second_vertex_index != first_vertex_index_prime)){
                Connection connection(second_vertex_index, first_vertex_index_prime, glm::distance((*particles)[second_vertex_index].position, (*particles)[first_vertex_index_prime].position));
                std::cout << connection.index1 << " and " << connection.index2 << " connected." << std::endl;
                bool isRedundant = false;
                for(int j=0;j<connections.size();j++){
                    if(connection == connections[j]){
                        std::cout << "is redundant!" << std::endl;
                        isRedundant = true;
                        break;
                    }
                }
                if(!isRedundant){
                    connections.push_back(connection);
                }
            }
            if((((first_vertex_index == first_vertex_index_prime) && (third_vertex_index == third_vertex_index_prime)) || ((first_vertex_index == third_vertex_index_prime) && (third_vertex_index == first_vertex_index_prime))) && (second_vertex_index != second_vertex_index_prime)){
                Connection connection(second_vertex_index, second_vertex_index_prime, glm::distance((*particles)[second_vertex_index].position, (*particles)[second_vertex_index_prime].position));
                std::cout << connection.index1 << " and " << connection.index2 << " connected." << std::endl;
                bool isRedundant = false;
                for(int j=0;j<connections.size();j++){
                    if(connection == connections[j]){
                        std::cout << "is redundant!" << std::endl;
                        isRedundant = true;
                        break;
                    }
                }
                if(!isRedundant){
                    connections.push_back(connection);
                }
            }
            if((((first_vertex_index == first_vertex_index_prime) && (third_vertex_index == second_vertex_index_prime)) || ((first_vertex_index == second_vertex_index_prime) && (third_vertex_index == first_vertex_index_prime))) && (second_vertex_index != third_vertex_index_prime)){
                Connection connection(second_vertex_index, third_vertex_index_prime, glm::distance((*particles)[second_vertex_index].position, (*particles)[third_vertex_index_prime].position));
                std::cout << connection.index1 << " and " << connection.index2 << " connected." << std::endl;
                bool isRedundant = false;
                for(int j=0;j<connections.size();j++){
                    if(connection == connections[j]){
                        std::cout << "is redundant!" << std::endl;
                        isRedundant = true;
                        break;
                    }
                }
                if(!isRedundant){
                    connections.push_back(connection);
                }
            }
            if((((first_vertex_index == second_vertex_index_prime) && (second_vertex_index == third_vertex_index_prime)) || ((first_vertex_index == third_vertex_index_prime) && (second_vertex_index == second_vertex_index_prime))) && (third_vertex_index != first_vertex_index_prime)){
                Connection connection(third_vertex_index, first_vertex_index_prime, glm::distance((*particles)[third_vertex_index].position, (*particles)[first_vertex_index_prime].position));
                std::cout << connection.index1 << " and " << connection.index2 << " connected." << std::endl;
                bool isRedundant = false;
                for(int j=0;j<connections.size();j++){
                    if(connection == connections[j]){
                        std::cout << "is redundant!" << std::endl;
                        isRedundant = true;
                        break;
                    }
                }
                if(!isRedundant){
                    connections.push_back(connection);
                }
            }
            if((((first_vertex_index == first_vertex_index_prime) && (second_vertex_index == third_vertex_index_prime)) || ((first_vertex_index == third_vertex_index_prime) && (second_vertex_index == first_vertex_index_prime))) && (third_vertex_index != second_vertex_index_prime)){
                Connection connection(third_vertex_index, second_vertex_index_prime, glm::distance((*particles)[third_vertex_index].position, (*particles)[second_vertex_index_prime].position));
                std::cout << connection.index1 << " and " << connection.index2 << " connected." << std::endl;
                bool isRedundant = false;
                for(int j=0;j<connections.size();j++){
                    if(connection == connections[j]){
                        std::cout << "is redundant!" << std::endl;
                        isRedundant = true;
                        break;
                    }
                }
                if(!isRedundant){
                    connections.push_back(connection);
                }
            }
            if((((first_vertex_index == first_vertex_index_prime) && (second_vertex_index == second_vertex_index_prime)) || ((first_vertex_index == second_vertex_index_prime) && (second_vertex_index == first_vertex_index_prime))) && (third_vertex_index != third_vertex_index_prime)){
                Connection connection(third_vertex_index, third_vertex_index_prime, glm::distance((*particles)[third_vertex_index].position, (*particles)[third_vertex_index_prime].position));
                std::cout << connection.index1 << " and " << connection.index2 << " connected." << std::endl;
                bool isRedundant = false;
                for(int j=0;j<connections.size();j++){
                    if(connection == connections[j]){
                        std::cout << "is redundant!" << std::endl;
                        isRedundant = true;
                        break;
                    }
                }
                if(!isRedundant){
                    connections.push_back(connection);
                }
            }
        }
    }
    
    void computeGravity(){
        for(int i=0;i<particles->size();i++){
            (*particles)[i].force += glm::vec3(0.0, -9.8*(*particles)[i].mass, 0.0);
        }
    }
    void computeDampedSpringForce(){
        glm::vec3 l = glm::vec3(0.0);
        float l_size = 0.0;
        glm::vec3 l_dot = glm::vec3(0.0);
        float restlength = 0.0;
        for(int i=0;i<connections.size();i++){
            //Implement this!
            l = (*particles)[connections[i].index1].position - (*particles)[connections[i].index2].position;
            l_size = glm::length(l);
            l_dot = (*particles)[connections[i].index1].velocity - (*particles)[connections[i].index2].velocity;
            restlength = connections[i].restLength;
            glm::vec3 f = -(k_s * (l_size - restlength) + k_d/l_size * (glm::dot(l_dot,l))) * (l/l_size);
            //std::cout << f1.x << " "<<f1.y << " " << f1.z << std::endl;
            (*particles)[connections[i].index1].force += f;
            (*particles)[connections[i].index2].force += -f;
        }
    }
    void computeAirResistance(){
        for(int i=0;i<particles->size();i++){
            (*particles)[i].force -= airResistance * (*particles)[i].velocity;
        }
    }
};

#endif /* ParticleSystem_hpp */
