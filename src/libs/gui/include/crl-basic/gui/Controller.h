#pragma once
#include "crl-basic/gui/glUtils.h"
#include <crl-basic/gui/camera.h>
#include "mocap/MocapSkeletonState.h"

namespace crl::mocap {
    // Game object class
    struct GameObject{
        crl::P3D position = crl::P3D(0,0,0);
        crl::V3D velocity = crl::V3D(0,0,0);
        crl::V3D angularVel = crl::V3D(0,0,0);
        Eigen::MatrixXd trajectoryPos = Eigen::MatrixXd::Zero(60,2);
        Eigen::MatrixXd trajectoryDir = Eigen::MatrixXd::Zero(60,2);
    };

    class Controller {
        public:
        Controller()
        {   
            speed = 0;  // initialize speed of character;
            // intialize vectors -- to be done
            speed_add_unit = 0.01;
        };

        void run(GLFWwindow *window, const crl::P3D& hippos, const crl::V3D& hipvel, crl::gui::TrackingCamera &camera, Eigen::VectorXd& futureTrajInfo)
        {
            manipulatefromWSAD(window);
            object = GameObject();
            object.position = crl::P3D(hippos.x, hippos.y, hippos.z);
            object.velocity = crl::V3D(hipvel.x(), hipvel.y(), hipvel.z());
            updateTrajectory(camera);
            obtainTrajectoryInfo(futureTrajInfo);
        }

        // void updateNewSkeletionInfo(const std::vector<crl::P3D>& markerPos, const std::vector<crl::V3D>& markerVel, const std::vector<crl::Quaternion> & marker)
        // {
        //     matchedPos = markerPos; //  const to non const may error?
        //     matchedVel = markerVel;
        //     matchedQuat = marker; 
        // }

        void getUpdatedState(MocapSkeletonState& state)
        {
            state.setRootPosition(object.position);
            state.setRootVelocity(object.velocity);

            Eigen::Quaternion<double> quaternion;
            quaternion.w() = 0;
            quaternion.vec() = object.velocity.normalized();
            state.setRootOrientation(quaternion);
        }

        void obtainTrajectoryInfo(Eigen::VectorXd& futureTrajInfo)
        {
            futureTrajInfo.setZero(12);
            futureTrajInfo << object.trajectoryPos(19,0), object.trajectoryPos(19,1), 
                              object.trajectoryPos(39,0), object.trajectoryPos(39,1),
                              object.trajectoryPos(59,0), object.trajectoryPos(59,1),
                              object.trajectoryDir(19,0), object.trajectoryDir(19,1),
                              object.trajectoryDir(39,0), object.trajectoryDir(39,1),
                              object.trajectoryDir(59,0), object.trajectoryDir(59,1); //要不要改成10 20 30
            
        }

        void drawTrajectory(const crl::gui::Shader &shader){
            Eigen::VectorXd traj_info;
            obtainTrajectoryInfo(traj_info);

            crl::P3D pose20 = crl::P3D(traj_info[0],0,traj_info[1]);
            crl::P3D pose40 = crl::P3D(traj_info[2],0,traj_info[3]);
            crl::P3D pose60 = crl::P3D(traj_info[4],0,traj_info[5]);
            crl::V3D vel20 = crl::V3D(traj_info[6],0,traj_info[7]).normalized() *0.5;
            crl::V3D vel40 = crl::V3D(traj_info[8],0,traj_info[9]).normalized() *0.5;
            crl::V3D vel60 = crl::V3D(traj_info[10],0,traj_info[11]).normalized() *0.5;
            // draw pose
            crl::gui::drawSphere(pose20, 0.05, shader, Eigen::Vector3d(1, 0, 0), 1.0);
            crl::gui::drawSphere(pose40, 0.05, shader, Eigen::Vector3d(1, 0, 0), 1.0);
            crl::gui::drawSphere(pose60, 0.05, shader, Eigen::Vector3d(1, 0, 0), 1.0);

            crl::gui::drawArrow3d(pose20, vel20, 0.03, shader, Eigen::Vector3d(1, 0, 0), 1.0);
            crl::gui::drawArrow3d(pose40, vel40, 0.03, shader, Eigen::Vector3d(1, 0, 0), 1.0);
            crl::gui::drawArrow3d(pose60, vel60, 0.03, shader, Eigen::Vector3d(1, 0, 0), 1.0);
        }

        Eigen::VectorXd prevFeat;
        double speed;
        double speed_add_unit;
        double dt;
        Eigen::Vector3d movingDir;
        double max_speed = 5;
        double dir_angle = 0;
        double angle_change_unit;
        GameObject object;

        void updateTrajectory(crl::gui::TrackingCamera &camera){
            // camera facing direction
            glm::vec3 orientation = camera.getOrientation();
            Eigen::Vector2d camera_dir(orientation.x, orientation.z);
            camera_dir.normalized();

            // set move direction based keyboard input
            Eigen::Vector2d key_dir(0,0);
            if (KEY_W) key_dir[0] += 1;
            if (KEY_A) key_dir[1] -= 1;
            if (KEY_S) key_dir[0] -= 1;
            if (KEY_D) key_dir[1] += 1;
            key_dir.normalized();
            Eigen::Matrix2d rot_matrix;
            rot_matrix << key_dir[0], -key_dir[1],
                          key_dir[1], key_dir[0];
            // Desired velocity and direction from keyboard input
            Eigen::Vector2d desiredDir = rot_matrix * camera_dir;
            if (key_dir.norm() == 0) desiredDir << object.velocity[0], object.velocity[2];
            Eigen::Vector2d desiredVel = desiredDir * 5.0f; // scale to maximum speed

            // System state
            Eigen::Vector2d obj_pos(object.position[0], object.position[2]);
            Eigen::Vector2d obj_vel(object.velocity[0], object.velocity[2]);

            // Time step and spring damper parameters
            float dt = 0.1f;
            float k = 0.1f;
            float b = 0.1f;


            // Loop over future 60 frames
            for (int i=0; i < object.trajectoryPos.rows(); i++){

                // Calculate current force based on spring-damper system equation
                Eigen::Vector2d x = obj_pos - desiredDir;
                Eigen::Vector2d v = obj_vel - desiredVel;
                Eigen::Vector2d force = (x * (-k)) + (v * (-b));

                // Update velocity and position using Euler integration
                obj_vel = obj_vel + (force * dt);
                obj_pos = obj_pos + (obj_vel * dt);

                if (i == 0){
                    // Apply position and velocity to game object
                    object.position = crl::P3D(obj_pos[0], 0, obj_pos[1]);
                    object.velocity = crl::V3D(obj_vel[0], 0, obj_vel[1]);
                    object.angularVel = crl::V3D(0,1,0) * acos((obj_pos.normalized()).dot(obj_vel.normalized()));
                }
                object.trajectoryPos.row(i) = Eigen::Vector2d(obj_pos[0], obj_pos[1]);
                object.trajectoryDir.row(i) = Eigen::Vector2d(obj_vel[0], obj_vel[1]).normalized();
            }
            camera.target.x =  object.position[0];
            camera.target.z =  object.position[2];
        }

        private:
        V3D key_dir;
        bool KEY_W = false, KEY_A = false, KEY_S = false, KEY_D = false;
        void manipulatefromWSAD(GLFWwindow *window)
        {
            KEY_W = false;
            KEY_A = false;
            KEY_S = false;
            KEY_D = false;
            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            {
                std::cout << "w pressed" << std::endl;
                KEY_W = true;
            }
            if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            {
                std::cout << "s pressed" << std::endl;
                KEY_S = true;
            }
            if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            {
                std::cout << "a pressed" << std::endl;
                KEY_A = true;
            }
            if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            {
                std::cout << "d pressed" << std::endl;
                KEY_D = true;
            }
        }
    };
}