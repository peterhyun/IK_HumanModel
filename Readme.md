# IK_HumanModel
> October 2019 Jeehon Hyun

This is an IK-solver(using Jacobian matrix and damped-least-squared pseudo-inverse) for the shoulder-to-arm part and the pelvis-to-arm part of the human model.  
The shoulder's position is fixed and it is a ball-and-socket joint. The rest of the joints(elbow, hand) are just revolute joints.

## Demo
https://www.youtube.com/watch?v=n8UnMyvP6Uk&feature=youtu.be

## Key bindings  
R: Reset model back to default  
M: Mouse Mode(Fix the screen and make mouse visible)  
N: Exit Mouse Mode  
1: IK with just the arm  
2: IK with the whole upper part of body  
C: Change hand position and orientation  
Esc: Exit  

For more details regarding Jacobian-based IK solvers, refer to the ```IK Report.pdf``` file.

## How to build and run.
1. Download this repository and go to AnimationFramework/Build directory.
```
git clone --recursive https://github.com/peterhyun/IK_HumanModel.git
cd IK_HumanModel
cd Build
```

If you forgot to use the ```--recursive``` flag while cloning this repository, use the following line to update the submodules:  
```git submodule update --init```

2. Make a project/solution file or makefile depending on your platform. I used Microsoft Windows, Visual Studio 2019.
```
# UNIX Makefile
cmake ..

# Mac OSX
cmake -G "Xcode" ..

# Microsoft Windows
cmake -G "Visual Studio 16 2019" ..
...
```
3. Build the project on your platform accordingly.

While building in the Visual Studio 2019 IDE, ```Treat Warning As Errors``` had to be unticked in the project file property settings for building the Assimp library.
![DoNotTreatWarningAsErrors](screenshots/DoNotTreatWarningsAsErrors.jpg)

4. Go to the directory ```Build/Glitter/Debug``` and now you can see the ```Glitter.exe``` file. Run it with no extra arguments, and enjoy!

## Directory Structure & Explanation
.  
├── Build/  
├── Glitter/  
│   ├── Headers/  
│   │   ├── BoneRig.h  
│   │   ├── Camera.h  
│   │   ├── Joint.h  
│   │   └── Shader.h  
│   ├── Shaders/  
│   │   ├── AnimationVertexShader.vs  
│   │   └── AnimationFragmentShader.fs  
│   ├── Sources/  
│   │   └── main.cpp  
│   └── Vendor/  
├── screenshots/  
├── CMakeLists.txt  
├── IK Report.pdf  
└── Readme.md  

The directory tree above shows the files and folders relevant to this project. All the code that I wrote are in the ```./Glitter/Headers/```, ```./Glitter/Shaders/```, ```./Gliter/Sources/``` directory. Each ```Joint``` class represents a joint and the connected single bone. There are 24 bones/joints of the human model and the ```BoneRig``` class manages a tree structure of 24 ```Joint```s. The root node represents the pelvis bone and the leaf nodes are the end sites of the human body. ```BoneRig``` also solves the two IK problem(left hand being the end site and the joints being the whole left arm or the whole upper boddy) via the ```BoneRig::solveIKUpperBoddy``` function and the ```BoneRig::solveIKArm``` function. These functions utilize the Jacobian matrices representing the series of joints depending on the type of IK, which are calculated in ```BoneRig::getJacobianUpperBody```, and ```BoneRig::getJacobianArm```.

The vertex shader and fragment shader are simple glsl code used to render the green model on screen, so I won't go over it here.

## Acknowledgement
The basic OpenGL Setup boilerplate of this code is from Kevin Fung's Glitter repository: https://github.com/Polytonic/Glitter
With this boilerplate, you can test this code in Windows, Linux, or Mac environment. The only dependency it requires is CMake.

-------------------------------------------------------------------------------------------------------------------
## MIT License of the Glitter Boilerplate
>The MIT License (MIT)

>Copyright (c) 2015 Kevin Fung

>Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

>The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

>THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
