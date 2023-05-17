#pragma once
#include <crl-basic/gui/glUtils.h>
#include <crl-basic/gui/camera.h>
#include "mocap/MocapSkeletonState.h"

namespace crl::mocap {
    // Game object class
    struct GameObject{
        crl::P3D position = crl::P3D(0,0,0);
        crl::V3D velocity = crl::V3D(0,0,0);
        Eigen::Vector3d originalPosition = Eigen::Vector3d(0,0,0);
        Eigen::Vector3d originalVelocity = Eigen::Vector3d(0,0,0);
        crl::Quaternion orientation;
        Eigen::MatrixXd trajectoryPos = Eigen::MatrixXd::Zero(60,3);
        Eigen::MatrixXd trajectoryDir = Eigen::MatrixXd::Zero(60,3);
        crl::V3D angularVel = crl::V3D(0,0,0);
    };

    class Controller {
        public:

        void run(GLFWwindow *window, const crl::P3D& hippos, const crl::V3D& hipvel, const crl::Quaternion& hipquat, crl::gui::TrackingCamera &camera, Eigen::VectorXd& futureTrajInfo, bool &inputGet)
        {
            manipulatefromWSAD(window);
            object.position = crl::P3D(hippos.x, hippos.y, hippos.z);
            if(!KEY_W && !KEY_A && !KEY_S && !KEY_D){
                // object.velocity = crl::V3D(0, 0, 0);
                object.velocity = crl::V3D(hipvel.x(), hipvel.y(), hipvel.z());
                inputGet = false;
            }
            else{
               object.velocity = crl::V3D(hipvel.x(), hipvel.y(), hipvel.z());
               inputGet = true;
            }
            object.originalPosition = Eigen::Vector3d(hippos.x, 0, hippos.z);
            object.originalVelocity = Eigen::Vector3d(hipvel.x(), 0, hipvel.z());
            object.orientation = hipquat;
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
            crl::P3D root_p = state.getRootPosition();
            crl::V3D root_v = state.getRootVelocity();
            Eigen::Quaternion root_o = state.getRootOrientation();

            state.setRootPosition(crl::P3D(object.position[0], root_p[1], object.position[2]));
            state.setRootVelocity(crl::V3D(object.velocity[0], root_v[1], object.velocity[2]));
            // TODO: set quaternion according to local orientation
            // Eigen::Quaternion<double> quaternion;
            // quaternion.w() = 0;
            // quaternion.vec() = object.velocity.normalized();
            // state.setRootOrientation(quaternion);
        }

        void obtainTrajectoryInfo(Eigen::VectorXd& futureTrajInfo)
        {
            crl::Quaternion localOrientationRotT = object.orientation.inverse();
            Eigen::Vector3d trajDir20 = object.trajectoryDir.row(19);
            Eigen::Vector3d trajDir40 = object.trajectoryDir.row(39);
            Eigen::Vector3d trajDir60 = object.trajectoryDir.row(59);
            Eigen::Vector3d trajDir20local = localOrientationRotT * trajDir20;
            Eigen::Vector3d trajDir40local = localOrientationRotT * trajDir40;
            Eigen::Vector3d trajDir60local = localOrientationRotT * trajDir60;
            Eigen::Vector3d trajPos20 = object.trajectoryPos.row(19);
            Eigen::Vector3d trajPos40 = object.trajectoryPos.row(39);
            Eigen::Vector3d trajPos60 = object.trajectoryPos.row(59);
            Eigen::Vector3d trajPos20local = localOrientationRotT * (trajPos20 - object.originalPosition);
            Eigen::Vector3d trajPos40local = localOrientationRotT * (trajPos40 - object.originalPosition);
            Eigen::Vector3d trajPos60local = localOrientationRotT * (trajPos60 - object.originalPosition);
            futureTrajInfo.setZero(12);
            futureTrajInfo << trajPos20local(0), trajPos20local(2),
                              trajPos40local(0), trajPos40local(2),
                              trajPos60local(0), trajPos60local(2),
                              trajDir20local(0), trajDir20local(2),
                              trajDir40local(0), trajDir40local(2),
                              trajDir60local(0), trajDir60local(2); //要不要改成10 20 30
        }

        void drawTrajectory(const crl::gui::Shader &shader){
            crl::P3D pose20 = crl::P3D(object.trajectoryPos(19,0), 0, object.trajectoryPos(19,2));
            crl::P3D pose40 = crl::P3D(object.trajectoryPos(39,0), 0, object.trajectoryPos(39,2));
            crl::P3D pose60 = crl::P3D(object.trajectoryPos(59,0), 0, object.trajectoryPos(59,2));
            crl::V3D vel20 = crl::V3D(object.trajectoryDir.row(19));
            crl::V3D vel40 = crl::V3D(object.trajectoryDir.row(39));
            crl::V3D vel60 = crl::V3D(object.trajectoryDir.row(59));
            // draw pose
            crl::gui::drawSphere(pose20, 0.05, shader, Eigen::Vector3d(1, 0, 0), 1.0);
            crl::gui::drawSphere(pose40, 0.05, shader, Eigen::Vector3d(1, 0, 0), 1.0);
            crl::gui::drawSphere(pose60, 0.05, shader, Eigen::Vector3d(1, 0, 0), 1.0);

            crl::gui::drawArrow3d(pose20, vel20, 0.03, shader, Eigen::Vector3d(1, 0, 0), 1.0);
            crl::gui::drawArrow3d(pose40, vel40, 0.03, shader, Eigen::Vector3d(1, 0, 0), 1.0);
            crl::gui::drawArrow3d(pose60, vel60, 0.03, shader, Eigen::Vector3d(1, 0, 0), 1.0);
        }

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
            // Eigen::Vector3d desiredDir(desiredDir2d[0], 0, desiredDir2d[1]);
            // Eigen::Vector3d desiredVel(desiredVel2d[0], 0, desiredVel2d[1]); // scale to maximum speed
            Eigen::Vector2d desiredVel = desiredDir * 3.3f; // scale to maximum speed

            // System state
            Eigen::Vector2d obj_pos(object.position[0], object.position[2]);
            Eigen::Vector2d obj_vel(object.velocity[0], object.velocity[2]);
            // Eigen::Vector3d obj_pos = object.position;
            // Eigen::Vector3d obj_vel = object.velocity;
            // Time step and spring damper parameters
            float dt = 0.0166f;
            float k = 20.0f;
            float b = 40.0f;

            // Loop over future 60 frames
            for (int i=0; i < object.trajectoryPos.rows(); i++){

                // Calculate current force based on spring-damper system equation
                Eigen::Vector2d x = - desiredDir;
                Eigen::Vector2d v = obj_vel - desiredVel;
                Eigen::Vector2d force = (x * (-k)) + (v * (-b));
                // Eigen::Vector3d x = obj_pos - desiredDir;
                // Eigen::Vector3d v = obj_vel - desiredVel;
                // Eigen::Vector3d force = (x * (-k)) + (v * (-b));

                // Update velocity and position using Euler integration
                obj_vel = obj_vel + (force * dt);
                obj_pos = obj_pos + (obj_vel * dt);

                if (i == 0){
                    // Apply position and velocity to game object
                    object.position = crl::P3D(obj_pos[0], 0, obj_pos[1]);
                    object.velocity = crl::V3D(obj_vel[0], 0, obj_vel[1]);
                    // object.position = obj_pos;
                    // object.velocity = obj_vel;
                    object.angularVel = crl::V3D(0,1,0) * acos((obj_pos.normalized()).dot(obj_vel.normalized()));
                }
                object.trajectoryPos.row(i) = Eigen::Vector3d(obj_pos[0], 0, obj_pos[1]);
                object.trajectoryDir.row(i) = Eigen::Vector3d(obj_vel[0], 0, obj_vel[1]).normalized();
                // object.trajectoryPos.row(i) = obj_pos;
                // object.trajectoryDir.row(i) = obj_vel.normalized();
            }
            camera.target.x =  object.position[0];
            camera.target.z =  object.position[2];
        }

        private:
        V3D key_dir;
        GameObject object;
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