#ifndef BoneRig_H
#define BoneRig_H

#include "Bone.h"

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
    Bone * root;
    void setHierarchy();
    void setBoneVAOs();
    void sub_setBoneVAOs(Bone * root);
    int * VISITED;
    int totalChannels = 0;
    void clearVISITED();
    void resetMatrices();
    void sub_resetMatrices(Bone * root);
};

void BoneRig::setHierarchy() {
    //pelvis
    root = new Bone(0);
    root->setName("pelvis");
    //lfemur
    root->setChild(new Bone(1, 11.446f,0.0f,0.0f));
    root->i_thChild(0)->setName("lfemur");
    //ltibia
    root->i_thChild(0)->setChild(new Bone(2, 0.0f, -42.0304f, 0.0f));
    root->i_thChild(0)->i_thChild(0)->setName("ltibia");
    //lfoot
    root->i_thChild(0)->i_thChild(0)->setChild(new Bone(3, 0.0f, -39.9302f, 0.0f));
    root->i_thChild(0)->i_thChild(0)->i_thChild(0)->setName("lfoot");
    //ltoes
    root->i_thChild(0)->i_thChild(0)->i_thChild(0)->setChild(new Bone(4, 0.0f, 0.0f, 11.7758f));
    root->i_thChild(0)->i_thChild(0)->i_thChild(0)->i_thChild(0)->setName("ltoes");
    //end site
    root->i_thChild(0)->i_thChild(0)->i_thChild(0)->i_thChild(0)->setChild(new Bone(5, 0.0f, 0.0f, 5.0f));
    root->i_thChild(0)->i_thChild(0)->i_thChild(0)->i_thChild(0)->i_thChild(0)->setName("End");
    //rfemur
    root->setChild(new Bone(6, -11.446f, 0.0f, 0.0f));
    root->i_thChild(1)->setName("rfemur");
    //rtibia
    root->i_thChild(1)->setChild(new Bone(7, 0.0f, -42.0304f, 0.0f));
    root->i_thChild(1)->i_thChild(0)->setName("rtibia");
    //rfoot
    root->i_thChild(1)->i_thChild(0)->setChild(new Bone(8, 0.0f, -39.9302f, 0.0f));
    root->i_thChild(1)->i_thChild(0)->i_thChild(0)->setName("rfoot");
    //rtoes
    root->i_thChild(1)->i_thChild(0)->i_thChild(0)->setChild(new Bone(9, 0.0f, 0.0f, 11.7758f));
    root->i_thChild(1)->i_thChild(0)->i_thChild(0)->i_thChild(0)->setName("rtoes");
    //end site
    root->i_thChild(1)->i_thChild(0)->i_thChild(0)->i_thChild(0)->setChild(new Bone(10, 0.0f, 0.0f, 5.0f));
    root->i_thChild(1)->i_thChild(0)->i_thChild(0)->i_thChild(0)->i_thChild(0)->setName("End");
    //thorax
    root->setChild(new Bone(11, 0.0f, 11.2556f, -5.0f));
    root->i_thChild(2)->setName("thorax");
    //head
    root->i_thChild(2)->setChild(new Bone(12, 0.0f, 43.7993f, 0.0f));
    root->i_thChild(2)->i_thChild(0)->setName("head");
    //end site
    root->i_thChild(2)->i_thChild(0)->setChild(new Bone(13, 0.0f, 5.0f, 0.0f));
    root->i_thChild(2)->i_thChild(0)->i_thChild(0)->setName("End");
    //lclavicle
    root->i_thChild(2)->setChild(new Bone(14, 0.0f, 39.863f, 0.0f));
    root->i_thChild(2)->i_thChild(1)->setName("lclavicle");
    //lhumerous
    root->i_thChild(2)->i_thChild(1)->setChild(new Bone(15, 17.380600f, 4.9583f, 0.0f));
    root->i_thChild(2)->i_thChild(1)->i_thChild(0)->setName("lhumerous");
    //lradius
    root->i_thChild(2)->i_thChild(1)->i_thChild(0)->setChild(new Bone(16, 0.0f, -27.5613f, 0.0f));
    root->i_thChild(2)->i_thChild(1)->i_thChild(0)->i_thChild(0)->setName("lradius");
    //lhand
    root->i_thChild(2)->i_thChild(1)->i_thChild(0)->i_thChild(0)->setChild(new Bone(17, 0.0f, -26.933f, 0.0f));
    root->i_thChild(2)->i_thChild(1)->i_thChild(0)->i_thChild(0)->i_thChild(0)->setName("lhand");
    //lend
    root->i_thChild(2)->i_thChild(1)->i_thChild(0)->i_thChild(0)->i_thChild(0)->setChild(new Bone(18, 0.0f, -5.0f, 0.0f));
    root->i_thChild(2)->i_thChild(1)->i_thChild(0)->i_thChild(0)->i_thChild(0)->i_thChild(0)->setName("End");
    //rclavicle
    root->i_thChild(2)->setChild(new Bone(19, 0.0f, 39.863f, 0.0f));
    root->i_thChild(2)->i_thChild(2)->setName("rclavicle");
    //rhumerous
    root->i_thChild(2)->i_thChild(2)->setChild(new Bone(20, -17.3806f, 4.9583f, 0.0f));
    root->i_thChild(2)->i_thChild(2)->i_thChild(0)->setName("rhumerous");
    //rradius
    root->i_thChild(2)->i_thChild(2)->i_thChild(0)->setChild(new Bone(21, 0.0f, -27.5613f, 0.0f));
    root->i_thChild(2)->i_thChild(2)->i_thChild(0)->i_thChild(0)->setName("rradius");
    //rhand
    root->i_thChild(2)->i_thChild(2)->i_thChild(0)->i_thChild(0)->setChild(new Bone(22, 0.0f, -29.933f, 0.0f));
    root->i_thChild(2)->i_thChild(2)->i_thChild(0)->i_thChild(0)->i_thChild(0)->setName("rhand");
    //rend
    root->i_thChild(2)->i_thChild(2)->i_thChild(0)->i_thChild(0)->i_thChild(0)->setChild(new Bone(23, 0.0f, -5.0f, 0.0f));
    root->i_thChild(2)->i_thChild(2)->i_thChild(0)->i_thChild(0)->i_thChild(0)->i_thChild(0)->setName("End");
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
    sub_setBoneVAOs(root);
}

void BoneRig::sub_setBoneVAOs(Bone * root){
    root->setVAOs();
    VISITED[root->returnBoneID()] = 1;
    for (int i = 0; i < root->returnnChildren(); i++) {
        if(!VISITED[root->i_thChild(i)->returnBoneID()])
            sub_setBoneVAOs(root->i_thChild(i));
    }
}

void BoneRig::resetMatrices() {
    clearVISITED();
    sub_resetMatrices(root);
}

void BoneRig::sub_resetMatrices(Bone * root){
    root->setR(glm::mat4(1.0f));
    root->setT();
    VISITED[root->returnBoneID()] = 1;
    for (int i = 0; i < root->returnnChildren(); i++) {
        if (!VISITED[root->i_thChild(i)->returnBoneID()])
            sub_resetMatrices(root->i_thChild(i));
    }
}
#endif
