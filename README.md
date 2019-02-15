# IK_HumanModel

This is an IK-solver(using Jacobian matrix and damped-least-squared pseudo-inverse) for the shoulder-to arm part of the human model. The shoulder's position is fixed, while the shoulder joint is a ball-and-socket joint. The rest of the joints(elbow, hand) are just revolute joints.  
Here is a demo video.  
https://www.youtube.com/watch?v=n8UnMyvP6Uk&feature=youtu.be

Key bindings:  
R: Reset model back to default  
M: Mouse Mode(Fix the screen and make mouse visible)  
N: Exit Mouse Mode  
1: IK with just the arm  
2: IK with the whole upper part of body  
C: Change hand position and orientation  
Esc: Exit  

For more details, read the IK Report.pdf file.
