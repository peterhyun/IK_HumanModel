#ifndef BoneRig_H
#define BoneRig_H

#define GLM_ENABLE_EXPERIMENTAL

#include "Joint.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <glm/gtx/string_cast.hpp>

#define Xposition 1
#define Yposition 2
#define Zposition 3
#define Zrotation 4
#define Xrotation 5
#define Yrotation 6

class BoneRig
{
public:
    //pelvis
    Joint * Joints;
    void setHierarchy();
    void setBoneVAOs();
    int * VISITED;
    void clearVISITED();
    void resetMatrices();
    bool solveIKArm(const glm::vec3& target,const glm::quat& ori_d);
    bool solveIKUpperBody(const glm::vec3& target, const glm::quat& ori_d);
    //reset the data, clear local variables.
    void clearTarget();
    
private:
    void sub_setBoneVAOs(Joint * rootJoint);
    void sub_resetMatrices(Joint * rootJoint);
    glm::quat oldori_d = glm::quat(1.0f,0.0f,0.0f,0.0f);
    glm::vec3 oldtarget = glm::vec3(1.0f);
    glm::vec3 getEndEffectorP();
    glm::quat getEndEffectorQ();
    void setOrientationArm(Eigen::VectorXf& theta_d);
    void setOrientationUpperBody(Eigen::VectorXf& theta_d);
    Eigen::MatrixXf getJacobianArm(const glm::vec3& endEffector);
    Eigen::MatrixXf getJacobianUpperBody(const glm::vec3& endEffector);
    Eigen::VectorXf getx_dot(float deltaTime, const glm::vec3& errorP, const glm::quat& endEffectorOri, const glm::quat& ori_d);
    glm::mat4 getJointGlobal2LocalT(int index);
    glm::mat4 sub_getJointGlobal2LocalT(Joint * root, glm::mat4 T);
    
    /* //For future implementation! IK for the leg!
    void setOrientationLeftLeg(Eigen::VectorXf& theta_d);
    void setOrientationRightLeg(Eigen::VectorXf& theta_d);
    Eigen::MatrixXf getJacobianLeftLeg(const glm::vec3& endEffector);
    Eigen::MatrixXf getJacobianRightLeg(const glm::vec3& endEffector);
    glm::vec3 getEndEffectorLeftFootP();
    glm::quat getEndEffectorLeftFootQ();
    glm::vec3 getEndEffectorRightFootP();
    glm::quat getEndEffectorRightFootQ();
    */
    
};

//Get Joint[index]'s global position
glm::mat4 BoneRig::getJointGlobal2LocalT(int index){
    glm::mat4 temp = glm::mat4(1.0f);
    glm::mat4 T = sub_getJointGlobal2LocalT(&Joints[index], temp);
    return T;
}

glm::mat4 BoneRig::sub_getJointGlobal2LocalT(Joint * root, glm::mat4 T){
    if(root==nullptr){
        //std::cout << "root is a nullptr" << std::endl;
        return T;
    }
    //std::cout << "rootID is "<<root->returnJointID()<< std::endl;
    T = root->returnT() * glm::toMat4(root->returnR_quat())*T;
    return sub_getJointGlobal2LocalT(root->returnParent(), T);
}

void BoneRig::setHierarchy() {
    Joints = new Joint[24]; //Joints[0] is the root
    //pelvis
    //root = new Joint(0);
    Joints[0].setJointID(0);
    Joints[0].setName("pelvis");
    //Without this, error happens when using getJointGlobal2LocalT
    Joints[0].setParent(nullptr);
    
    //lfemur
    Joints[0].setChild(&Joints[1]);
    Joints[1].setParent(&Joints[0]);
    Joints[1].setJointID(1);
    Joints[1].setFloats(11.446f, 0.0f, 0.0f);
    //root->i_thChild(0)->setName("lfemur");
    Joints[1].setName("lfemur");
    
    
    //ltibia
    Joints[1].setChild(&Joints[2]);
    Joints[2].setParent(&Joints[1]);
    Joints[2].setJointID(2);
    Joints[2].setFloats(0.0f, -42.0304f, 0.0f);
    Joints[2].setName("ltibia");
    
    //lfoot
    Joints[2].setChild(&Joints[3]);
    Joints[3].setParent(&Joints[2]);
    Joints[3].setJointID(3);
    Joints[3].setFloats(0.0f, -39.9302f, 0.0f);
    Joints[3].setName("lfoot");
    
    //ltoes
    Joints[3].setChild(&Joints[4]);
    Joints[4].setParent(&Joints[3]);
    Joints[4].setJointID(4);
    Joints[4].setFloats(0.0f, 0.0f, 11.7758f);
    Joints[4].setName("ltoes");
    
    //end site
    Joints[4].setChild(&Joints[5]);
    Joints[5].setParent(&Joints[4]);
    Joints[5].setJointID(5);
    Joints[5].setFloats(0.0f, 0.0f, 5.0f);
    Joints[5].setName("End");
    
    //rfemur
    Joints[0].setChild(&Joints[6]);
    Joints[6].setParent(&Joints[0]);
    Joints[6].setJointID(6);
    Joints[6].setFloats(-11.446f, 0.0f, 0.0f);
    Joints[6].setName("rfemur");
    
    //rtibia
    Joints[6].setChild(&Joints[7]);
    Joints[7].setParent(&Joints[6]);
    Joints[7].setJointID(7);
    Joints[7].setFloats(0.0f, -42.0304f, 0.0f);
    Joints[7].setName("rtibia");
    
    //rfoot
    Joints[7].setChild(&Joints[8]);
    Joints[8].setParent(&Joints[7]);
    Joints[8].setJointID(8);
    Joints[8].setFloats(0.0f, -39.9302f, 0.0f);
    Joints[8].setName("rfoot");
    
    //rtoes
    Joints[8].setChild(&Joints[9]);
    Joints[9].setParent(&Joints[8]);
    Joints[9].setJointID(9);
    Joints[9].setFloats(0.0f, 0.0f, 11.7758f);
    Joints[9].setName("rtoes");
    
    //end site
    Joints[9].setChild(&Joints[10]);
    Joints[10].setParent(&Joints[9]);
    Joints[10].setJointID(10);
    Joints[10].setFloats(0.0f, 0.0f, 5.0f);
    Joints[10].setName("End");
    
    //thorax
    Joints[0].setChild(&Joints[11]);
    Joints[11].setParent(&Joints[0]);
    Joints[11].setJointID(11);
    Joints[11].setFloats(0.0f, 11.2556f, -5.0f);
    Joints[11].setName("thorax");
    
    //head
    Joints[11].setChild(&Joints[12]);
    Joints[12].setParent(&Joints[11]);
    Joints[12].setJointID(12);
    Joints[12].setFloats(0.0f, 43.7993f, 0.0f);
    Joints[12].setName("head");
    
    //end site
    Joints[12].setChild(&Joints[13]);
    Joints[13].setParent(&Joints[12]);
    Joints[13].setJointID(13);
    Joints[13].setFloats(0.0f, 5.0f, 0.0f);
    Joints[13].setName("End");
    
    //lclavicle
    Joints[11].setChild(&Joints[14]);
    Joints[14].setParent(&Joints[11]);
    Joints[14].setJointID(14);
    Joints[14].setFloats(0.0f, 39.863f, 0.0f);
    Joints[14].setName("lclavicle");
    
    //lhumerous
    Joints[14].setChild(&Joints[15]);
    Joints[15].setParent(&Joints[14]);
    Joints[15].setJointID(15);
    Joints[15].setFloats(17.3806f, 4.9583f, 0.0f);
    Joints[15].setName("lhumerous");
    
    //lradius
    Joints[15].setChild(&Joints[16]);
    Joints[16].setParent(&Joints[15]);
    Joints[16].setJointID(16);
    Joints[16].setFloats(0.0f, -27.5613f, 0.0f);
    Joints[16].setName("lradius");
    
    //lhand
    Joints[16].setChild(&Joints[17]);
    Joints[17].setParent(&Joints[16]);
    Joints[17].setJointID(17);
    Joints[17].setFloats(0.0f, -26.933f, 0.0f);
    Joints[17].setName("lhand");
    
    //lend
    Joints[17].setChild(&Joints[18]);
    Joints[18].setParent(&Joints[17]);
    Joints[18].setJointID(18);
    Joints[18].setFloats(0.0f, -5.0f, 0.0f);
    Joints[18].setName("End");
    
    //rclavicle
    Joints[11].setChild(&Joints[19]);
    Joints[19].setParent(&Joints[11]);
    Joints[19].setJointID(19);
    Joints[19].setFloats(0.0f, 39.863f, 0.0f);
    Joints[19].setName("rclavicle");
    
    //rhumerous
    Joints[19].setChild(&Joints[20]);
    Joints[20].setParent(&Joints[19]);
    Joints[20].setJointID(20);
    Joints[20].setFloats(-17.3806f, 4.9583f, 0.0f);
    Joints[20].setName("rhumerous");
    
    //rradius
    Joints[20].setChild(&Joints[21]);
    Joints[21].setParent(&Joints[20]);
    Joints[21].setJointID(21);
    Joints[21].setFloats(0.0f, -27.5613f, 0.0f);
    Joints[21].setName("rradius");
    
    //rhand
    Joints[21].setChild(&Joints[22]);
    Joints[22].setParent(&Joints[21]);
    Joints[22].setJointID(22);
    Joints[22].setFloats(0.0f, -29.933f, 0.0f);
    Joints[22].setName("rhand");
    
    //rend
    Joints[22].setChild(&Joints[23]);
    Joints[23].setParent(&Joints[22]);
    Joints[23].setJointID(23);
    Joints[23].setFloats(0.0f, -5.0f, 0.0f);
    Joints[23].setName("End");
    
    resetMatrices();
}

void BoneRig::clearVISITED() {
    VISITED = new int[24];
    for (int i = 0; i < 24; i++) {
        VISITED[i] = 0;
    }
}

//now set the vertex data of each bones
void BoneRig::setBoneVAOs() {
    clearVISITED();
    sub_setBoneVAOs(&Joints[0]);
}

void BoneRig::sub_setBoneVAOs(Joint * rootJoint){
    rootJoint->setVAOs();
    VISITED[rootJoint->returnJointID()] = 1;
    for (int i = 0; i < rootJoint->returnnChildren(); i++) {
        if(!VISITED[rootJoint->i_thChild(i)->returnJointID()])
            sub_setBoneVAOs(rootJoint->i_thChild(i));
    }
}

void BoneRig::resetMatrices() {
    //std::cout << "resetMatrices called!" << std::endl;
    clearVISITED();
    sub_resetMatrices(&Joints[0]);
}

void BoneRig::sub_resetMatrices(Joint * rootJoint){
    rootJoint->setR_quat(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    rootJoint->setT();
    VISITED[rootJoint->returnJointID()] = 1;
    for (int i = 0; i < rootJoint->returnnChildren(); i++) {
        if (!VISITED[rootJoint->i_thChild(i)->returnJointID()])
            sub_resetMatrices(rootJoint->i_thChild(i));
    }
}

void BoneRig::clearTarget(){
    //std::cout << "clear() called!" <<std::endl;
    clearVISITED();
    oldori_d = glm::quat(1.0f,0.0f,0.0f,0.0f);
    oldtarget = glm::vec3(1.0f);
}

//target is 7*1, including the quaternion
bool BoneRig::solveIKArm(const glm::vec3& target, const glm::quat& ori_d) {
    /*
    if(oldori_d == ori_d && oldtarget == target){
        return true;
    }
    */
    resetMatrices();
    //std::cout << "Different target!" << std::endl;
    
    oldori_d = ori_d;
    oldtarget = target;
    
    float error = 0.0f;
    Eigen::VectorXf deltaQ(5);
    Eigen::VectorXf Q_desired(5);
    Q_desired << 0,0,0,0,0;
    glm::vec3 endEffectorP;
    //glm::vec3 endEffector = glm::vec3(0.0f,0.0f,0.0f);
    glm::quat endEffectorOri;
    glm::vec3 errorP;
    float errorQ;
    Eigen::MatrixXf J(6,5);
    float deltaTime = 0.01;
    Eigen::MatrixXf J_plus(5,6);
    
    Eigen::VectorXf oldQ_desired(6);
    float old_error;
    
    int MaxIter = 2000;
    int iter = 0;
    
    while(true){
        endEffectorP = getEndEffectorP();
        endEffectorOri = getEndEffectorQ();
        
        //std::cout << "Current endEffectorP is "<< glm::to_string(endEffectorP) <<std::endl;
        errorP = target - endEffectorP;
        
        //std::cout << "errorP size is " << glm::length(errorP) << std::endl;
        
        Eigen::VectorXf x_dot(6);
        x_dot = getx_dot(deltaTime, errorP, endEffectorOri, ori_d);
        
        J = getJacobianArm(endEffectorP);
        //std::cout << "Jacobian is "<< J << std::endl;
        
        J_plus = ((J.transpose() * J + 0.1*0.1*Eigen::Matrix<float,5,5>::Identity()).inverse()) * J.transpose();
        deltaQ = J_plus * x_dot;
        //std::cout << "deltaQ is "<< deltaQ << std::endl;
        
        oldQ_desired = Q_desired;
        Q_desired += deltaQ;
        
        setOrientationArm(Q_desired);
        
        glm::quat qd= glm::inverse(ori_d) * endEffectorOri;
        
        errorQ = glm::length(glm::log(qd));
        
        //std::cout << "errorQ size is " << errorQ << std::endl;
        
        //NOW DEBUGGING!!!
        old_error = error;
        error = glm::length(errorP) * glm::length(errorP) + errorQ * errorQ;
        
        //Make sure it didn't get worse. I'm not sure if I need this though...
        
        
        /*
        if((error > old_error) && iter>0){
            //std::cout << "old_error is "<< old_error << std::endl;
            //std::cout << "error is "<< error << std::endl;
            std::cout << "Moving to wrong direction. Ending as previous state" <<std::endl;
            setOrientationArm(oldQ_desired);
            return false;
        }
         */
        
        //std::cout << "error is "<< error << std::endl;
        
        iter++;
        
        if(iter>= MaxIter){
            std::cout << "You could not solve this you loser "<<std::endl;
            return false;
        }
        
        if(error < 0.1){
            std::cout << "You made it at iter " << iter << std::endl;
            return true;
        }
    }
    return false;
}

bool BoneRig::solveIKUpperBody(const glm::vec3& target, const glm::quat& ori_d){
    /*
    if(oldori_d == ori_d && oldtarget == target)
        return true;
    */
    //std::cout << "Different target for solveIKUpperBody()!" << std::endl;
    resetMatrices();

    oldori_d = ori_d;
    oldtarget = target;
    
    
    
    float error = 0.0f;
    Eigen::VectorXf deltaQ(8);
    Eigen::VectorXf Q_desired(8);
    Q_desired << 0,0,0,0,0,0,0,0;
    glm::vec3 endEffectorP;
    //glm::vec3 endEffector = glm::vec3(0.0f,0.0f,0.0f);
    glm::quat endEffectorOri;
    glm::vec3 errorP;
    float errorQ;
    Eigen::MatrixXf J(6,8);
    float deltaTime = 0.01;
    Eigen::MatrixXf J_plus(8,6);
    
    Eigen::VectorXf oldQ_desired(8);
    float old_error;
    
    int MaxIter = 2000;
    int iter = 0;
    
    while(true){
        endEffectorP = getEndEffectorP();
        endEffectorOri = getEndEffectorQ();
        
        //std::cout << "Current endEffectorP is "<< glm::to_string(endEffectorP) <<std::endl;
        errorP = target - endEffectorP;
        
        //std::cout << "errorP size is " << glm::length(errorP) << std::endl;
        
        Eigen::VectorXf x_dot(6);
        x_dot = getx_dot(deltaTime, errorP, endEffectorOri, ori_d);
        
        J = getJacobianUpperBody(endEffectorP);
        //std::cout << "Jacobian is "<< J << std::endl;
        
        J_plus = J.transpose() * ((J * J.transpose() + 0.1*0.1*Eigen::Matrix<float,6,6>::Identity()).inverse());
        deltaQ = J_plus * x_dot;
        //std::cout << "deltaQ is "<< deltaQ << std::endl;
        
        oldQ_desired = Q_desired;
        Q_desired += deltaQ;
        
        setOrientationUpperBody(Q_desired);
        
        glm::quat qd= glm::inverse(ori_d) * endEffectorOri;
        
        errorQ = glm::length(glm::log(qd));
        
        //std::cout << "errorQ size is " << errorQ << std::endl;
        
        //NOW DEBUGGING!!!
        old_error = error;
        error = glm::length(errorP) * glm::length(errorP) + errorQ * errorQ;
        
        //Make sure it didn't get worse. I'm not sure if I need this though...
        /*
         if((error > old_error) && iter>0){
         //std::cout << "old_error is "<< old_error << std::endl;
         //std::cout << "error is "<< error << std::endl;
         std::cout << "Moving to wrong direction" <<std::endl;
         setOrientationUpperBody(oldQ_desired);
         break;
         }
         */
        //std::cout << "error is "<< error << std::endl;
        
        iter++;
        
        if(iter>= MaxIter){
            std::cout << "You could not solve this you loser "<<std::endl;
            return false;
        }
        
        if(error < 0.1){
            std::cout << "You made it at iter " << iter << std::endl;
            return true;
        }
    }
    return false;
}

Eigen::VectorXf BoneRig::getx_dot(float deltaTime, const glm::vec3& errorP, const glm::quat& endEffectorOri, const glm::quat& ori_d){
    Eigen::VectorXf x_dot(6);
    //xyz eulers
    glm::vec3 currentEuler = glm::eulerAngles(endEffectorOri);
    glm::vec3 desiredEuler = glm::eulerAngles(ori_d);
    
    x_dot << errorP.x * deltaTime, errorP.y * deltaTime, errorP.z * deltaTime, (desiredEuler-currentEuler).x * deltaTime, (desiredEuler-currentEuler).y * deltaTime, (desiredEuler-currentEuler).z * deltaTime;
    return x_dot;
}

Eigen::MatrixXf BoneRig::getJacobianArm(const glm::vec3& endEffector){
    Eigen::MatrixXf J(6,5);
    glm::vec4 xAxis = glm::vec4(1.0f,0.0f,0.0f,0.0f);
    glm::vec4 yAxis = glm::vec4(0.0f,1.0f,0.0f,0.0f);
    glm::vec4 zAxis = glm::vec4(0.0f,0.0f,1.0f,0.0f);
    glm::mat4 temp = getJointGlobal2LocalT(15);
    glm::vec3 worldXAxis = glm::vec3(temp * xAxis);
    glm::vec3 worldYAxis = glm::vec3(temp * yAxis);
    glm::vec3 worldZAxis = glm::vec3(temp * zAxis);
    glm::vec3 worldPos = glm::vec3(temp * glm::vec4(0.0f,0.0f,0.0f, 1.0f));
    glm::vec3 p = endEffector - worldPos;
    glm::vec3 upperPart = glm::cross(worldXAxis,p);
    J(0,0) = upperPart.x;
    J(1,0) = upperPart.y;
    J(2,0) = upperPart.z;
    J(3,0) = worldXAxis.x;
    J(4,0) = worldXAxis.y;
    J(5,0) = worldXAxis.z;
    upperPart = glm::cross(worldYAxis,p);
    J(0,1) = upperPart.x;
    J(1,1) = upperPart.y;
    J(2,1) = upperPart.z;
    J(3,1) = worldYAxis.x;
    J(4,1) = worldYAxis.y;
    J(5,1) = worldYAxis.z;
    upperPart = glm::cross(worldZAxis,p);
    J(0,2) = upperPart.x;
    J(1,2) = upperPart.y;
    J(2,2) = upperPart.z;
    J(3,2) = worldZAxis.x;
    J(4,2) = worldZAxis.y;
    J(5,2) = worldZAxis.z;
    temp = temp * Joints[16].returnT() * glm::toMat4(Joints[16].returnR_quat());
    worldXAxis = glm::vec3(temp * xAxis);
    worldPos = glm::vec3(temp * glm::vec4(0.0f,0.0f,0.0f, 1.0f));
    p = endEffector - worldPos;
    upperPart = glm::cross(worldXAxis,p);
    J(0,3) = upperPart.x;
    J(1,3) = upperPart.y;
    J(2,3) = upperPart.z;
    J(3,3) = worldXAxis.x;
    J(4,3) = worldXAxis.y;
    J(5,3) = worldXAxis.z;
    temp = temp * Joints[17].returnT() * glm::toMat4(Joints[17].returnR_quat());
    worldXAxis = glm::vec3(temp * xAxis);
    worldPos = glm::vec3(temp * glm::vec4(0.0f,0.0f,0.0f, 1.0f));
    p = endEffector - worldPos;
    upperPart = glm::cross(worldXAxis,p);
    J(0,4) = upperPart.x;
    J(1,4) = upperPart.y;
    J(2,4) = upperPart.z;
    J(3,4) = worldXAxis.x;
    J(4,4) = worldXAxis.y;
    J(5,4) = worldXAxis.z;
    return J;
}

Eigen::MatrixXf BoneRig::getJacobianUpperBody(const glm::vec3& endEffector){
    Eigen::MatrixXf J(6,8);
    glm::vec4 xAxis = glm::vec4(1.0f,0.0f,0.0f,0.0f);
    glm::vec4 yAxis = glm::vec4(0.0f,1.0f,0.0f,0.0f);
    glm::vec4 zAxis = glm::vec4(0.0f,0.0f,1.0f,0.0f);
    glm::mat4 temp = getJointGlobal2LocalT(11);
    glm::vec3 worldXAxis = glm::vec3(temp * xAxis);
    glm::vec3 worldYAxis = glm::vec3(temp * yAxis);
    glm::vec3 worldZAxis = glm::vec3(temp * zAxis);
    glm::vec3 worldPos = glm::vec3(temp * glm::vec4(0.0f,0.0f,0.0f, 1.0f));
    glm::vec3 p = endEffector - worldPos;
    glm::vec3 upperPart = glm::cross(worldXAxis,p);
    J(0,0) = 1.0*upperPart.x;
    J(1,0) = 1.0*upperPart.y;
    J(2,0) = 1.0*upperPart.z;
    J(3,0) = 1.0*worldXAxis.x;
    J(4,0) = 1.0*worldXAxis.y;
    J(5,0) = 1.0*worldXAxis.z;
    upperPart = glm::cross(worldYAxis,p);
    J(0,1) = 1.0*upperPart.x;
    J(1,1) = 1.0*upperPart.y;
    J(2,1) = 1.0*upperPart.z;
    J(3,1) = 1.0*worldYAxis.x;
    J(4,1) = 1.0*worldYAxis.y;
    J(5,1) = 1.0*worldYAxis.z;
    upperPart = glm::cross(worldZAxis,p);
    J(0,2) = 1.0*upperPart.x;
    J(1,2) = 1.0*upperPart.y;
    J(2,2) = 1.0*upperPart.z;
    J(3,2) = 1.0*worldZAxis.x;
    J(4,2) = 1.0*worldZAxis.y;
    J(5,2) = 1.0*worldZAxis.z;
    temp = temp * Joints[14].returnT() * glm::toMat4(Joints[14].returnR_quat()) * Joints[15].returnT() * glm::toMat4(Joints[15].returnR_quat());
    worldXAxis = glm::vec3(temp * xAxis);
    worldYAxis = glm::vec3(temp * yAxis);
    worldZAxis = glm::vec3(temp * zAxis);
    worldPos = glm::vec3(temp * glm::vec4(0.0f,0.0f,0.0f, 1.0f));
    p = endEffector - worldPos;
    upperPart = glm::cross(worldXAxis,p);
    J(0,3) = 1.1*upperPart.x;
    J(1,3) = 1.1*upperPart.y;
    J(2,3) = 1.1*upperPart.z;
    J(3,3) = 1.1*worldXAxis.x;
    J(4,3) = 1.1*worldXAxis.y;
    J(5,3) = 1.1*worldXAxis.z;
    upperPart = glm::cross(worldYAxis,p);
    J(0,4) = 1.1*upperPart.x;
    J(1,4) = 1.1*upperPart.y;
    J(2,4) = 1.1*upperPart.z;
    J(3,4) = 1.1*worldYAxis.x;
    J(4,4) = 1.1*worldYAxis.y;
    J(5,4) = 1.1*worldYAxis.z;
    upperPart = glm::cross(worldZAxis,p);
    J(0,5) = 1.1*upperPart.x;
    J(1,5) = 1.1*upperPart.y;
    J(2,5) = 1.1*upperPart.z;
    J(3,5) = 1.1*worldZAxis.x;
    J(4,5) = 1.1*worldZAxis.y;
    J(5,5) = 1.1*worldZAxis.z;
    temp = temp * Joints[16].returnT() * glm::toMat4(Joints[16].returnR_quat());
    worldXAxis = glm::vec3(temp * xAxis);
    worldPos = glm::vec3(temp * glm::vec4(0.0f,0.0f,0.0f, 1.0f));
    p = endEffector - worldPos;
    upperPart = glm::cross(worldXAxis,p);
    J(0,6) = 1.2*upperPart.x;
    J(1,6) = 1.2*upperPart.y;
    J(2,6) = 1.2*upperPart.z;
    J(3,6) = 1.2*worldXAxis.x;
    J(4,6) = 1.2*worldXAxis.y;
    J(5,6) = 1.2*worldXAxis.z;
    temp = temp * Joints[17].returnT() * glm::toMat4(Joints[17].returnR_quat());
    worldXAxis = glm::vec3(temp * xAxis);
    worldPos = glm::vec3(temp * glm::vec4(0.0f,0.0f,0.0f, 1.0f));
    p = endEffector - worldPos;
    upperPart = glm::cross(worldXAxis,p);
    J(0,7) = 1.2*upperPart.x;
    J(1,7) = 1.2*upperPart.y;
    J(2,7) = 1.2*upperPart.z;
    J(3,7) = 1.2*worldXAxis.x;
    J(4,7) = 1.2*worldXAxis.y;
    J(5,7) = 1.2*worldXAxis.z;
    return J;
}

void BoneRig::setOrientationArm(Eigen::VectorXf& theta_d){
    Joints[15].setR_quat(glm::angleAxis(theta_d(0), glm::vec3(1.0f,0.0f,0.0f)));
    Joints[15].setR_quat(Joints[15].returnR_quat() * glm::angleAxis(theta_d(1), glm::vec3(0.0f, 1.0f, 0.0f)));
    Joints[15].setR_quat(Joints[15].returnR_quat() * glm::angleAxis(theta_d(2), glm::vec3(0.0f, 0.0f, 1.0f)));
    //Implement Joint limits
    if(theta_d(3)>0){
        //std::cout << "Joint limit 0 activated"<<std::endl;
        theta_d(3) = 0;
    }
    
    Joints[16].setR_quat(glm::angleAxis(theta_d(3), glm::vec3(1.0f, 0.0f, 0.0f)));
    Joints[17].setR_quat(glm::angleAxis(theta_d(4), glm::vec3(1.0f, 0.0f, 0.0f)));

    //std::cout << "orientation arm set" << std::endl;

    return;
}

void BoneRig::setOrientationUpperBody(Eigen::VectorXf& theta_d){
    if(theta_d(0)<glm::radians(-70.0f)){
        //std::cout << "Joint limit -70.0f";
        theta_d(0) = glm::radians(-70.0f);
    }
    Joints[11].setR_quat(glm::angleAxis(theta_d(0), glm::vec3(1.0f,0.0f,0.0f)));
    if(theta_d(1)>glm::radians(90.0f)){
        //std::cout << "Joint[11]_y limit 90.0f";
        theta_d(1) = glm::radians(90.0f);
    }
    if(theta_d(1)<glm::radians(-90.0f)){
        //std::cout << "Joint[11]_y limit -90.0f";
        theta_d(1) = glm::radians(-90.0f);
    }
    Joints[11].setR_quat(Joints[11].returnR_quat() * glm::angleAxis(theta_d(1), glm::vec3(0.0f, 1.0f, 0.0f)));
    if(theta_d(2)>glm::radians(90.0f)){
        //std::cout << "Joint limit_z 90.0f";
        theta_d(2) = glm::radians(90.0f);
    }
    if(theta_d(2)<glm::radians(-90.0f)){
        //std::cout << "Joint limit_z -90.0f";
        theta_d(2) = glm::radians(-90.0f);
    }
    Joints[11].setR_quat(Joints[11].returnR_quat() * glm::angleAxis(theta_d(2), glm::vec3(0.0f, 0.0f, 1.0f)));
    Joints[15].setR_quat(glm::angleAxis(theta_d(3), glm::vec3(1.0f,0.0f,0.0f)));
    Joints[15].setR_quat(Joints[15].returnR_quat() * glm::angleAxis(theta_d(4), glm::vec3(0.0f, 1.0f, 0.0f)));
    Joints[15].setR_quat(Joints[15].returnR_quat() * glm::angleAxis(theta_d(5), glm::vec3(0.0f, 0.0f, 1.0f)));
    
    if(theta_d(6)>0){
        //std::cout << "Joint limit 0 activated: theta_d(6) is "<<theta_d(6)<<std::endl;
        theta_d(6) = 0;
    }
    
    Joints[16].setR_quat(glm::angleAxis(theta_d(6), glm::vec3(1.0f, 0.0f, 0.0f)));
    Joints[17].setR_quat(glm::angleAxis(theta_d(7), glm::vec3(1.0f, 0.0f, 0.0f)));
    
    //std::cout << "orientation upper body set" << std::endl;

    return;
}

//Correct method!
glm::vec3 BoneRig::getEndEffectorP(){
    glm::vec3 endEffector;
    glm::mat4 model = glm::mat4(1.0f);
    glm::vec4 origin = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    model = getJointGlobal2LocalT(18);
    glm::vec4 temp = model * origin;
    endEffector = glm::vec3(temp);
    return endEffector;
}

glm::quat BoneRig::getEndEffectorQ(){
    glm::quat model = Joints[0].returnR_quat();
    model = model * Joints[11].returnR_quat();
    model = model * Joints[14].returnR_quat();
    model = model * Joints[15].returnR_quat();
    model = model * Joints[16].returnR_quat();
    model = model * Joints[17].returnR_quat();
    model = model * Joints[18].returnR_quat();
    return model;
}

/*

void BoneRig::setOrientationLeftLeg(Eigen::VectorXf& theta_d){
    Joints[1].setR_quat(glm::angleAxis(theta_d(0), glm::vec3(1.0f,0.0f,0.0f)));
    Joints[1].setR_quat(Joints[1].returnR_quat() * glm::angleAxis(theta_d(1), glm::vec3(0.0f, 1.0f, 0.0f)));
    Joints[1].setR_quat(Joints[1].returnR_quat() * glm::angleAxis(theta_d(2), glm::vec3(0.0f, 0.0f, 1.0f)));
    
    if(theta_d(3)<0)
        theta_d(3)=0;
    Joints[2].setR_quat(glm::angleAxis(theta_d(3), glm::vec3(1.0f,0.0f,0.0f)));

    //std::cout << "orientation left leg set" << std::endl;
    return;
}

void BoneRig::setOrientationRightLeg(Eigen::VectorXf& theta_d){
    Joints[6].setR_quat(glm::angleAxis(theta_d(0), glm::vec3(1.0f,0.0f,0.0f)));
    Joints[6].setR_quat(Joints[6].returnR_quat() * glm::angleAxis(theta_d(1), glm::vec3(0.0f, 1.0f, 0.0f)));
    Joints[6].setR_quat(Joints[6].returnR_quat() * glm::angleAxis(theta_d(2), glm::vec3(0.0f, 0.0f, 1.0f)));
    
    if(theta_d(3)<0)
        theta_d(3)=0;
    Joints[7].setR_quat(glm::angleAxis(theta_d(3), glm::vec3(1.0f,0.0f,0.0f)));

    //std::cout << "orientation right leg set" << std::endl;
}

glm::vec3 BoneRig::getEndEffectorLeftFootP(){
    glm::mat4 toFoot = getJointGlobal2LocalT(3);
    return glm::vec3(toFoot * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
}

glm::vec3 BoneRig::getEndEffectorRightFootP(){
    glm::mat4 toFoot = getJointGlobal2LocalT(8);
    return glm::vec3(toFoot * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
}

glm::quat BoneRig::getEndEffectorLeftFootQ(){
    glm::quat model = Joints[0].returnR_quat();
    model = model * Joints[1].returnR_quat();
    model = model * Joints[2].returnR_quat();
    model = model * Joints[3].returnR_quat();
    return model;
}

glm::quat BoneRig::getEndEffectorRightFootQ(){
    glm::quat model = Joints[0].returnR_quat();
    model = model * Joints[6].returnR_quat();
    model = model * Joints[7].returnR_quat();
    model = model * Joints[8].returnR_quat();
    return model;
}

Eigen::MatrixXf BoneRig::getJacobianLeftLeg(const glm::vec3& endEffector){
    Eigen::MatrixXf J(6,4);
    glm::vec4 xAxis = glm::vec4(1.0f,0.0f,0.0f,0.0f);
    glm::vec4 yAxis = glm::vec4(0.0f,1.0f,0.0f,0.0f);
    glm::vec4 zAxis = glm::vec4(0.0f,0.0f,1.0f,0.0f);
    glm::mat4 temp = getJointGlobal2LocalT(1);
    glm::vec3 worldXAxis = glm::vec3(temp * xAxis);
    glm::vec3 worldYAxis = glm::vec3(temp * yAxis);
    glm::vec3 worldZAxis = glm::vec3(temp * zAxis);
    glm::vec3 worldPos = glm::vec3(temp * glm::vec4(0.0f,0.0f,0.0f, 1.0f));
    glm::vec3 p = endEffector - worldPos;
    glm::vec3 upperPart = glm::cross(worldXAxis,p);
    J(0,0) = 1.0*upperPart.x;
    J(1,0) = 1.0*upperPart.y;
    J(2,0) = 1.0*upperPart.z;
    J(3,0) = 1.0*worldXAxis.x;
    J(4,0) = 1.0*worldXAxis.y;
    J(5,0) = 1.0*worldXAxis.z;
    upperPart = glm::cross(worldYAxis,p);
    J(0,1) = 1.0*upperPart.x;
    J(1,1) = 1.0*upperPart.y;
    J(2,1) = 1.0*upperPart.z;
    J(3,1) = 1.0*worldYAxis.x;
    J(4,1) = 1.0*worldYAxis.y;
    J(5,1) = 1.0*worldYAxis.z;
    upperPart = glm::cross(worldZAxis,p);
    J(0,2) = 1.0*upperPart.x;
    J(1,2) = 1.0*upperPart.y;
    J(2,2) = 1.0*upperPart.z;
    J(3,2) = 1.0*worldZAxis.x;
    J(4,2) = 1.0*worldZAxis.y;
    J(5,2) = 1.0*worldZAxis.z;
    temp = temp * Joints[2].returnT() * glm::toMat4(Joints[2].returnR_quat());
    worldXAxis = glm::vec3(temp * xAxis);
    worldPos = glm::vec3(temp * glm::vec4(0.0f,0.0f,0.0f, 1.0f));
    p = endEffector - worldPos;
    upperPart = glm::cross(worldXAxis,p);
    J(0,3) = 1.0*upperPart.x;
    J(1,3) = 1.0*upperPart.y;
    J(2,3) = 1.0*upperPart.z;
    J(3,3) = 1.0*worldXAxis.x;
    J(4,3) = 1.0*worldXAxis.y;
    J(5,3) = 1.0*worldXAxis.z;
    return J;
}

Eigen::MatrixXf BoneRig::getJacobianRightLeg(const glm::vec3& endEffector){
    Eigen::MatrixXf J(6,4);
    glm::vec4 xAxis = glm::vec4(1.0f,0.0f,0.0f,0.0f);
    glm::vec4 yAxis = glm::vec4(0.0f,1.0f,0.0f,0.0f);
    glm::vec4 zAxis = glm::vec4(0.0f,0.0f,1.0f,0.0f);
    glm::mat4 temp = getJointGlobal2LocalT(6);
    glm::vec3 worldXAxis = glm::vec3(temp * xAxis);
    glm::vec3 worldYAxis = glm::vec3(temp * yAxis);
    glm::vec3 worldZAxis = glm::vec3(temp * zAxis);
    glm::vec3 worldPos = glm::vec3(temp * glm::vec4(0.0f,0.0f,0.0f, 1.0f));
    glm::vec3 p = endEffector - worldPos;
    glm::vec3 upperPart = glm::cross(worldXAxis,p);
    J(0,0) = 1.0*upperPart.x;
    J(1,0) = 1.0*upperPart.y;
    J(2,0) = 1.0*upperPart.z;
    J(3,0) = 1.0*worldXAxis.x;
    J(4,0) = 1.0*worldXAxis.y;
    J(5,0) = 1.0*worldXAxis.z;
    upperPart = glm::cross(worldYAxis,p);
    J(0,1) = 1.0*upperPart.x;
    J(1,1) = 1.0*upperPart.y;
    J(2,1) = 1.0*upperPart.z;
    J(3,1) = 1.0*worldYAxis.x;
    J(4,1) = 1.0*worldYAxis.y;
    J(5,1) = 1.0*worldYAxis.z;
    upperPart = glm::cross(worldZAxis,p);
    J(0,2) = 1.0*upperPart.x;
    J(1,2) = 1.0*upperPart.y;
    J(2,2) = 1.0*upperPart.z;
    J(3,2) = 1.0*worldZAxis.x;
    J(4,2) = 1.0*worldZAxis.y;
    J(5,2) = 1.0*worldZAxis.z;
    temp = temp * Joints[7].returnT() * glm::toMat4(Joints[7].returnR_quat());
    worldXAxis = glm::vec3(temp * xAxis);
    worldPos = glm::vec3(temp * glm::vec4(0.0f,0.0f,0.0f, 1.0f));
    p = endEffector - worldPos;
    upperPart = glm::cross(worldXAxis,p);
    J(0,3) = 1.0*upperPart.x;
    J(1,3) = 1.0*upperPart.y;
    J(2,3) = 1.0*upperPart.z;
    J(3,3) = 1.0*worldXAxis.x;
    J(4,3) = 1.0*worldXAxis.y;
    J(5,3) = 1.0*worldXAxis.z;
    return J;
}

*/

#endif
