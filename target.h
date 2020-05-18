#ifndef TARGET_H
#define TARGET_H


#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>


#include <vector>

//// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
//enum Target_Movement {
//    FORWARD,
//    BACKWARD,
//    LEFT,
//    RIGHT
//};
//
//// Default camera values
//const float YAW = -90.0f;
//const float PITCH = 0.0f;

#ifndef MOUSESPEED
#define MOUSESPEED
const float SPEED = 2.5f;
const float SENSITIVITY = 0.02f;
#endif

// An abstract camera class that processes input and calculates the corresponding Euler Angles, Vectors and Matrices for use in OpenGL
class Target
{
public:

    // Target attributes
    float rotationAngle;
    glm::vec3 rotationAxis;
    glm::vec3 translationVector;
    
    // Target options
    float MovementSpeed;
    float MouseSensitivity;

    // Constructor with vectors
    Target(glm::mat4 transformation = glm::mat4(1.0f), float angle = 0.0f , glm::vec3 axis = glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3 translation = glm::vec3(0.0f)) : MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY)
    {
        TransformationMatrix = transformation;
        rotationAngle = angle;
        rotationAxis = axis;
        translationVector = translation;
    }

    // Returns the view matrix calculated using Euler Angles and the LookAt Matrix
    glm::mat4 GetTransformationMatrix()
    {
        return TransformationMatrix;
    }


    glm::mat4 setTransformationMatrix(glm::mat4 trans)
    {
        TransformationMatrix = trans;
    }

    glm::mat4 setTransformationMatrix(glm::vec3 axis, float angle, glm::vec3 translation)
    {
        rotationAngle = angle;
        rotationAxis = axis;
        translationVector = translation;

        setupTransformationMatrix();
    }
    //// Processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
    //void ProcessKeyboard(Target_Movement direction, float deltaTime)
    //{
    //    float velocity = MovementSpeed * deltaTime;
    //    if (direction == FORWARD)
    //        Position += Front * velocity;
    //    if (direction == BACKWARD)
    //        Position -= Front * velocity;
    //    if (direction == LEFT)
    //        Position -= Right * velocity;
    //    if (direction == RIGHT)
    //        Position += Right * velocity;
    //}

    // Processes input received from a mouse input system. Expects the offset value in both the x and y direction.
    void ProcessMouseMovement(float xoffset, float yoffset, bool targetTranslate = false)
    {
        if (!targetTranslate) {
            float xvec = xoffset, yvec = yoffset;
            glm::vec3 mousedirection = glm::normalize(glm::vec3(xvec, -yvec, 0.0f));
            glm::vec3 tempvec = glm::vec3(0.0f, 0.0f, -1.0f);
            glm::vec3 rotation_axis = glm::cross(mousedirection, tempvec);

            float rotation_angle = sqrt(pow(xoffset, 2) + pow(yoffset, 2)) * MouseSensitivity;

            rotationAngle = rotation_angle;
            rotationAxis = rotation_axis;
            translationVector = glm::vec3(0.0f);
        }
        else {

            rotationAngle = 0.0f;
            rotationAxis = glm::vec3(0.0f, 1.0f, 0.0f);
            translationVector = glm::vec3(-xoffset * SENSITIVITY, yoffset * SENSITIVITY, 0);

        }

        // Update 
        updateTargetTransformation();
    }


    void outputTransformationMatrix() {
        for (int i = 0; i < 4; i++) {
            std::cout << TransformationMatrix[i].x << TransformationMatrix[i].y << TransformationMatrix[i].z << TransformationMatrix[i].w << std::endl;
        }
        
    }
    //void ProcessMouseMovement(float xoffset, float yoffset)
    //{

    //    float xvec = xoffset, yvec = yoffset;
    //    glm::vec3 mousedirection = glm::normalize(glm::vec3(xvec, yvec, 0.0f));
    //    glm::vec3 tempvec = glm::vec3(0.0f, 0.0f, -1.0f);
    //    glm::vec3 rotation_axis = glm::cross(mousedirection, tempvec);

    //    float rotation_angle = sqrt(pow(xoffset, 2) + pow(yoffset, 2)) * MouseSensitivity;

    //    // Update Front, Right and Up Vectors using the updated Euler angles
    //}
    

private:
    glm::mat4 TransformationMatrix;

    // Calculates the front vector from the Camera's (updated) Euler Angles
    void updateTargetTransformation()
    {
        TransformationMatrix = glm::rotate(TransformationMatrix, rotationAngle, rotationAxis);
        TransformationMatrix = glm::translate(TransformationMatrix, translationVector);
    }

    void setupTransformationMatrix()
    {
        glm::mat4 trans = glm::mat4(1.0f);
        trans = glm::rotate(trans, rotationAngle, rotationAxis);
        TransformationMatrix = glm::translate(trans, translationVector);
    }
};
#endif