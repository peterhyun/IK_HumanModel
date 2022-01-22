//
//  OBJReader.h
//  ClothingSimulation
//
//  Created by Jeehoon Hyun on 27/03/2019.
//  Copyright © 2019 Jeehoon Hyun. All rights reserved.
//

#ifndef OBJReader_h
#define OBJReader_h

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>

//For debugging
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

#include "Particle.h"


class OBJReader{
private:
    std::fstream FILE;
    std::vector<glm::vec3> temp_vertices;
    std::vector<glm::vec2> temp_uvs;
    std::vector<glm::vec3> temp_normals;
    std::vector<unsigned int> vertexIndices, uvIndices, normalIndices;
    std::vector<Particle> particles;
    std::vector<VERTEX> out_vertices;
    std::vector<unsigned int> out_indices;
    void read(){
        std::string line;
        std::string firstWord;
        while(!FILE.eof()){
            std::getline(FILE, line);
            std::istringstream iss(line);
            iss>>firstWord;
            if(firstWord.compare(std::string("v"))==0){
                glm::vec3 vertex;
                iss >> vertex.x >> vertex.y >> vertex.z;
                temp_vertices.push_back(vertex);
                Particle particle;
                particle.position = vertex;
                particle.mass = 0.1;
                particle.force = glm::vec3(0.0);
                particle.velocity = glm::vec3(0.0);
                particles.push_back(particle);
            }
            else if(firstWord.compare(std::string("vt"))==0){
                glm::vec2 uv;
                iss >> uv.x >> uv.y;
                temp_uvs.push_back(uv);
            }
            else if(firstWord.compare(std::string("vn"))==0){
                glm::vec3 normal;
                iss >> normal.x >> normal.y >> normal.z;
                temp_normals.push_back(normal);
            }
            else if(firstWord.compare(std::string("f"))==0){
                unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
                std::string bigChunk;
                for(int i=0;i<3;i++){
                    iss >> bigChunk;
                    std::replace(bigChunk.begin(), bigChunk.end(), '/', ' ');
                    //std::cout << "bigChunk[" << i << "] is " << bigChunk[i] << std::endl;
                    std::stringstream divider(bigChunk);
                    divider >> vertexIndex[i] >> uvIndex[i] >> normalIndex[i];
                }
                vertexIndices.push_back(vertexIndex[0]);
                vertexIndices.push_back(vertexIndex[1]);
                vertexIndices.push_back(vertexIndex[2]);
                uvIndices.push_back(uvIndex[0]);
                uvIndices.push_back(uvIndex[1]);
                uvIndices.push_back(uvIndex[2]);
                normalIndices.push_back(normalIndex[0]);
                normalIndices.push_back(normalIndex[1]);
                normalIndices.push_back(normalIndex[2]);
            }
        }
    }
    int getVertexIndex(VERTEX vertex){
        for(int i=0;i<out_vertices.size();i++){
            if((glm::distance(vertex.position, out_vertices[i].position)<0.001) && (glm::distance(vertex.uv_coord, out_vertices[i].uv_coord)<0.001) && (glm::distance(vertex.normal, out_vertices[i].normal)<0.001)){
                return i;
            }
        }
        return out_vertices.size();
    }
    void reorderData(){
        for(int i=0;i<vertexIndices.size();i++){
            VERTEX vertex;
            int vi = vertexIndices[i]-1;
            int uvi = uvIndices[i]-1;
            int ni = normalIndices[i]-1;
            vertex.position = temp_vertices[vi];
            vertex.uv_coord = temp_uvs[uvi];
            vertex.normal = temp_normals[ni];
            int index = getVertexIndex(vertex);
            if(index==out_vertices.size()){
                out_vertices.push_back(vertex);
            }
            out_indices.push_back(index);
        }
    }
    
public:
    
    OBJReader(const char * path){
        FILE.open(path);
        if(FILE.bad()){
            std::cout << "Impossible to open the file !\n" << std::endl;
        }
        read();
        reorderData();
        //std::cout << temp_vertices.size() << std::endl;
        std::cout << "out_vertices.size() is " << out_vertices.size() << std::endl;
        std::cout << "out_indices.size() is " << out_indices.size() << std::endl;
        std::cout << "particles.size() is " << particles.size() << std::endl;
        /*
        std::cout << vertexIndices.back() << std::endl;
        std::cout << uvIndices.back() << std::endl;
        std::cout << normalIndices.back() << std::endl;
        std::cout << glm::to_string(temp_vertices.back()) << std::endl;
        std::cout << glm::to_string(temp_uvs.back()) << std::endl;
        std::cout << glm::to_string(temp_normals.back()) << std::endl;
        std::cout << vertexIndices.size() << std::endl;
        std::cout << uvIndices.size() << std::endl;
        std::cout << normalIndices.size() << std::endl;
        */
        
    }
    
    std::vector<Particle> getParticles(){
        return particles;
    }
    
    std::vector<VERTEX> getVertices(){
        return out_vertices;
    }
    
    std::vector<unsigned int> getIndices(){
        return out_indices;
    }
    
    std::vector<unsigned int> getOriginalVertexIndices(){
        return vertexIndices;
    }
};

#endif /* OBJReader_h */
