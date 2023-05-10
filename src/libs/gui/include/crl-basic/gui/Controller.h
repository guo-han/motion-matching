#pragma once
#include "crl-basic/gui/glUtils.h"

namespace crl::mocap {
    class Controller {
        public:
        Controller()
        {   
            speed = 0;  // initialize speed of character;
            // intialize vectors -- to be done
            speed_add_unit = 0.01;
        };
        void run(GLFWwindow *window)
        {
            manipulatefromWSAD(window);
        }
        void updateNewSkeletionInfo(const std::vector<crl::P3D>& markerPos, const std::vector<crl::V3D>& markerVel, const std::vector<crl::Quaternion> & marker)
        {
            matchedPos = markerPos; //  const to non const may error?
            matchedVel = markerVel;
            matchedQuat = marker; 
        }
        public:
        std::vector<crl::P3D> matchedPos;
        std::vector<crl::V3D> matchedVel;
        std::vector<crl::Quaternion> matchedQuat; // facing direction
        Eigen::VectorXd prevFeat;
        double speed;
        double speed_add_unit;
        double dt;
        Eigen::Vector3d movingDir;
        double max_speed = 5;
        double dir_angle = 0;
        double angle_change_unit;
        void manipulatefromWSAD(GLFWwindow *window)
        {
            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            {
                speed += speed_add_unit * dt;
                std::cout << "w pressed" << std::endl;
            }
            else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            {
                speed -= speed_add_unit * dt;
                std::cout << "s pressed" << std::endl;
            }
            else if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            {
                dir_angle -= angle_change_unit * dt;
                std::cout << "a pressed" << std::endl;
            }
            else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            {
                dir_angle += angle_change_unit* dt;
                std::cout << "d pressed" << std::endl;
            }
        }
        // getstate
    };
}