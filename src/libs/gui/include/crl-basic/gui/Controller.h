#pragma once
#include "crl-basic/gui/glUtils.h"

namespace crl::mocap {
    class Controller {
        public:
        void run(GLFWwindow *window)
        {
            manipulatefromWSAD(window);
        }
        private:
        void manipulatefromWSAD(GLFWwindow *window)
        {
            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            {
                std::cout << "w pressed" << std::endl;
            }
            else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            {
                std::cout << "s pressed" << std::endl;
            }
            else if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            {
                std::cout << "a pressed" << std::endl;
            }
            else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            {
                std::cout << "d pressed" << std::endl;
            }
        }
    };
}