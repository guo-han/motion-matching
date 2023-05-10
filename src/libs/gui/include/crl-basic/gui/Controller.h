#pragma once
#include "crl-basic/gui/glUtils.h"
#include <crl-basic/gui/camera.h>

namespace crl::mocap {
    // Game object class
    struct GameObject{
        Eigen::Vector3d position = Eigen::Vector3d(0,0,0);
        Eigen::Vector3d velocity = Eigen::Vector3d(0,0,0);
        Eigen::MatrixXd trajectory = Eigen::MatrixXd::Zero(60,3);
    };

    class Controller {
        public:
        Controller()
        {   
            speed = 0;  // initialize speed of character;
            // intialize vectors -- to be done
            speed_add_unit = 0.01;
        };
        GameObject gameObject;

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

        void updateTrajectory(crl::gui::TrackingCamera &camera, GameObject &object){
            // camera facing direction
            Eigen::Vector2d camera_dir(camera.direction.x, camera.direction.z);
            camera_dir.normalized();

            // set move direction based keyboard input
            Eigen::Vector2d key_dir(0,0);
            if (KEY_W) key_dir[0] += 1;
            if (KEY_A) key_dir[1] += 1;
            if (KEY_S) key_dir[0] -= 1;
            if (KEY_D) key_dir[1] -= 1;
            key_dir.normalized();
            Eigen::Matrix2d rot_matrix;
            rot_matrix << key_dir[0], -key_dir[1],
                          key_dir[1], key_dir[0];
            // Desired velocity and direction from keyboard input
            Eigen::Vector2d desiredDir = rot_matrix * camera_dir;
            Eigen::Vector2d desiredVel = desiredDir * 10.0f; // scale to maximum speed

            // System state
            Eigen::Vector2d obj_pos(object.position[0], object.position[2]);
            Eigen::Vector2d obj_vel(object.velocity[0], object.velocity[2]);

            // Time step and spring damper parameters
            float dt = 0.1f;
            float k = 0.1f;
            float b = 0.1f;


            // Loop
            for (int i=0; i < object.trajectory.rows(); i++){

                // Calculate current force based on spring-damper system equation
                Eigen::Vector2d x = obj_pos - desiredDir;
                Eigen::Vector2d v = obj_vel - desiredVel;
                Eigen::Vector2d force = (x * (-k)) + (v * (-b));

                // Update velocity and position using Euler integration
                obj_vel = obj_vel + (force * dt);
                obj_pos = obj_pos + (obj_vel * dt);

                if (i == 0){
                    // Apply position and velocity to game object
                    object.position = Eigen::Vector3d(obj_pos[0], 0, obj_pos[1]);
                    object.velocity = Eigen::Vector3d(obj_vel[0], 0, obj_vel[1]);
                }
                object.trajectory.row(i) = Eigen::Vector3d(obj_pos[0], 0, obj_pos[1]);
            }
            camera.target.x =  object.position[0];
            camera.target.z =  object.position[2];
        }

        private:
        V3D key_dir;
        bool KEY_W = false, KEY_A = false, KEY_S = false, KEY_D = false;

        void manipulatefromWSAD(GLFWwindow *window)
        {
            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            {
                speed += speed_add_unit * dt;
                std::cout << "w pressed" << std::endl;
                KEY_W = true;
            }
            else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            {
                speed -= speed_add_unit * dt;
                std::cout << "s pressed" << std::endl;
                KEY_S = true;
            }
            else if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            {
                dir_angle -= angle_change_unit * dt;
                std::cout << "a pressed" << std::endl;
                KEY_A = true;
            }
            else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            {
                dir_angle += angle_change_unit* dt;
                std::cout << "d pressed" << std::endl;
                KEY_D = true;
            }
        }
        // getstate
    };
}