#pragma once

#include <crl-basic/gui/Controller.h>
#include <crl-basic/gui/application.h>
#include <crl-basic/utils/timer.h>
#include <imgui_widgets/imfilebrowser.h>

#include "mocap/MocapClip.h"
#include "mocap/MocapClipUtils.h"
#include "mocap/MocapMarkers.h"
#include "mocap/MocapSkeletonState.h"
#include "mocap/PlotUtils.h"
#include "mocap/TimelineUtils.h"

namespace mocapApp {

class App : public crl::gui::ShadowApplication {
public:
    App()
        : crl::gui::ShadowApplication("Mocap App"),
          fileDialog(ImGuiFileBrowserFlags_SelectDirectory),
          baseHeightPlot("Root Height", "[sec]", "[m]"),
          baseSpeedPlot("Root Speed", "[sec]", "[m/s or rad/s]"),
          feetHeightPlot("Feet Height", "[sec]", "[m]"),
          feetVelocityPlot("Feet Velocity", "[sec]", "[m/s]"),
          feetAccelerationPlot("Feet Acceleration", "[sec]", "[m/s/s]"),
          swingTimeline(footSteps) {
        fileDialog.SetPwd(fs::path(CRL_MOCAP_DATA_FOLDER));
        fileDialog.SetTitle("Mocap Directory");
    }

    ~App() override {}

    void run() override {
        float tmpEntireLoopTimeRunningAverage = 0.0f;
        float tmpProcessTimeRunningAverage = 0.0f;
        int runningAverageStepCount = 0;

        crl::Timer FPSDisplayTimer, processTimer, FPSTimer;
        glfwSwapInterval(0);  //Disable waiting for framerate of glfw window

        while (!glfwWindowShouldClose(window)) {
            if (FPSDisplayTimer.timeEllapsed() > 0.33) {
                // controller get input every 0.33s?
                // controller.getInputfromWSAD(window);
                FPSDisplayTimer.restart();
                if (runningAverageStepCount > 0) {
                    averageFPS = 1.0 / (tmpEntireLoopTimeRunningAverage / runningAverageStepCount);  // calculate FPS
                    averagePercentTimeSpentProcessing =
                        tmpProcessTimeRunningAverage / tmpEntireLoopTimeRunningAverage;  // calculate time for function process() or process_motion_matching()
                } else {
                    averageFPS = -1;
                    averagePercentTimeSpentProcessing = -1;
                }
                tmpEntireLoopTimeRunningAverage = 0;
                tmpProcessTimeRunningAverage = 0;
                runningAverageStepCount = 0;
            }
            runningAverageStepCount++;

            tmpEntireLoopTimeRunningAverage += FPSTimer.timeEllapsed();
            FPSTimer.restart();

            processTimer.restart();
            if (!useSeparateProcessThread && processIsRunning)
                process_motion_matching();
            tmpProcessTimeRunningAverage += processTimer.timeEllapsed();

            draw();

            // glfw: swap buffers and poll IO events (keys pressed/released, mouse
            // moved etc.)
#ifdef SINGLE_BUFFER
            glFlush();
#else
            glfwSwapBuffers(window);
#endif
            glfwPollEvents();

            if (limitFramerate)
                while (FPSTimer.timeEllapsed() < (1.0 / (double)targetFramerate)) {
#ifndef WIN32
                    using namespace std::chrono_literals;
                    std::this_thread::sleep_for(1ms);  //Sleep for a bit (too inaccurate to be used on windows)
#endif                                                 // WIN32
                }

            if (screenIsRecording) {
                char filename[1000];
                sprintf(filename, "%s_%04d.png", screenshotPath.c_str(), screenShotCounter);
                screenshot(filename);
                screenShotCounter++;
            }
            if (currentMatchClip != -1 && processIsRunning)
            {
                motion_matching_end_process();
            }
            
        }

        // glfw: terminate, clearing all previously allocated GLFW resources.
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        crl::gui::rendering::DestroyContext();
        ImPlot::DestroyContext();
        ImGui::DestroyContext();
        glfwDestroyWindow(window);
        glfwTerminate();
    }

    void process() override {
        if (selectedBvhClipIdx == -1 && selectedC3dClipIdx == -1)
            return;

        if (selectedBvhClipIdx > -1) {
            auto &clip = bvhClips[selectedBvhClipIdx];
            if (auto *skel = clip->getModel()) {
                auto state = clip->getState(frameIdx);
                skel->setState(&state);
                if (followCharacter) {
                    camera.target.x = (float)clip->getModel()->root->state.pos.x;
                    camera.target.z = (float)clip->getModel()->root->state.pos.z;
                }
                light.target.x() = (float)clip->getModel()->root->state.pos.x;
                light.target.z() = (float)clip->getModel()->root->state.pos.z;
            }
            if (++frameIdx >= clip->getFrameCount())
                frameIdx = 0;
        }

        if (selectedC3dClipIdx > -1) {
            auto &clip = c3dClips[selectedC3dClipIdx];
            if (auto *model = clip->getModel()) {
                auto state = clip->getState(frameIdx);
                model->setState(&state);
                camera.target.x = (float)clip->getModel()->getMarkerByName("S_1")->state.pos.x;
                camera.target.z = (float)clip->getModel()->getMarkerByName("S_1")->state.pos.z;
            }
            if (++frameIdx >= clip->getFrameCount())
                frameIdx = 0;
        }
    }

    void updateQueryVector(const Eigen::VectorXd& trajInfo)
    {
        currentQueryVector = DBMatching.row(currentMatchIdx);
        currentQueryVector.block(0, 0, 12, 1) << trajInfo;
    }

    void process_motion_matching() {    // 位置与编译错误
        // Only for bvh
        // No data input
        if (currentMatchFrame == -1 && currentMatchClip == -1)
            return;
        
        if (currentMatchClip > -1) {
            auto &clip = bvhClips[currentMatchClip];
            if (auto *skel = clip->getModel()) {
                // controller get state state type
                auto state = clip->getState(currentMatchFrame);
                // TODO: need to update the hip position and velocity here
                skel->setState(&state);
                // TODO: check the settings of camera target position, viewing angle.
                if (followCharacter) {
                    camera.target.x = (float)clip->getModel()->root->state.pos.x;
                    camera.target.z = (float)clip->getModel()->root->state.pos.z;
                }
                light.target.x() = (float)clip->getModel()->root->state.pos.x;
                light.target.z() = (float)clip->getModel()->root->state.pos.z;
                // auto updatedState = controller.getUpdatedState(state);   
            }
        }
    }
    
    void motion_matching_end_process()
    {
        nextMatchFrame = currentMatchFrame + 1;
        nextMatchClip = currentMatchClip;
        currentContinousFramesPlayed++;
        auto &clip = bvhClips[currentMatchClip];
        if (nextMatchFrame >= clip->getFrameCount() || currentContinousFramesPlayed >= maxContFramePlayed)
        {
            if (auto *skel = clip->getModel()) {
                crl::V3D markervel;
                crl::P3D markerpos;
                crl::Quaternion markerQuat;
                std::string marker = "Hips";
                if (const auto joint = skel -> getMarkerByName(marker.c_str())) {
                    markerpos = joint->state.pos;
                    markervel = joint->state.velocity;
                    markerQuat = joint -> state.orientation;
                } 
                Eigen::VectorXd trajInfo;
                controller.run(window, markerpos, markervel, markerQuat, camera, trajInfo);
                updateQueryVector(trajInfo);
                MMSearchforIndex();
                currentContinousFramesPlayed = 0;
            }
            else
            {
                std::cout << "getModelError" << std::endl;
            }
        }
        currentMatchFrame = nextMatchFrame;
    }
    
    void restart() override {
        frameIdx = 0;
    }

    void prepareToDraw() override {
        // if (selectedBvhClipIdx > -1) {
        //     bvhClips[selectedBvhClipIdx]->getModel()->showCoordFrame = showCoordinateFrames;
        // }
        // if (selectedC3dClipIdx > -1) {
        //     c3dClips[selectedC3dClipIdx]->getModel()->showCoordFrame = showCoordinateFrames;
        // }
        // only bvh
        if (currentMatchClip > -1) {
            bvhClips[currentMatchClip]->getModel()->showCoordFrame = showCoordinateFrames;
        }
    }

    void drawShadowCastingObjects(const crl::gui::Shader &shader) override {
        // if (selectedBvhClipIdx > -1) {
        //     bvhClips[selectedBvhClipIdx]->draw(shader, frameIdx);
        // }
        // if (selectedC3dClipIdx > -1) {
        //     c3dClips[selectedC3dClipIdx]->draw(shader, frameIdx);
        //     auto state = c3dClips[selectedC3dClipIdx]->getState(frameIdx);
        //     c3dClips[selectedC3dClipIdx]->getModel()->setState(&state);
        //     for (const auto &linkName : linkNames) {
        //         const auto *startMarker = c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(linkName.first.c_str());
        //         const auto *endMarker = c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(linkName.second.c_str());
        //         if (startMarker && endMarker)
        //             crl::gui::drawCapsule(startMarker->state.pos, endMarker->state.pos, 0.01, shader, crl::V3D(0.5, 0.5, 0.5));
        //     }
        // }
        if (currentMatchClip > -1) {
            bvhClips[currentMatchClip]->draw(shader, currentMatchFrame);
        }
    }

    void drawObjectsWithoutShadows(const crl::gui::Shader &shader) override {
        // if (selectedBvhClipIdx > -1)
        //     bvhClips[selectedBvhClipIdx]->draw(shader, frameIdx);
        // else if (bvhClips.size() != 0){
        //     bvhClips[0]->draw(shader, 0);
        //     selectedBvhClipIdx = 0;
        //     processBVHClip();
        // }
        // if (selectedC3dClipIdx > -1) {
        //     c3dClips[selectedC3dClipIdx]->draw(shader, frameIdx);

        //     // draw skeleton
        //     if (selectedC3dClipIdx > -1) {
        //         c3dClips[selectedC3dClipIdx]->draw(shader, frameIdx);
        //         auto state = c3dClips[selectedC3dClipIdx]->getState(frameIdx);
        //         c3dClips[selectedC3dClipIdx]->getModel()->setState(&state);
        //         for (const auto &linkName : linkNames) {
        //             const auto *startMarker = c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(linkName.first.c_str());
        //             const auto *endMarker = c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(linkName.second.c_str());
        //             if (startMarker && endMarker)
        //                 crl::gui::drawCapsule(startMarker->state.pos, endMarker->state.pos, 0.01, shader, crl::V3D(0.5, 0.5, 0.5));
        //         }

        //         if (showVirtualLimbs)
        //             for (const auto &linkName : virtualLinkNames) {
        //                 const auto *startMarker = c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(linkName.first.c_str());
        //                 const auto *endMarker = c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(linkName.second.c_str());
        //                 if (startMarker && endMarker)
        //                     crl::gui::drawCapsule(startMarker->state.pos, endMarker->state.pos, 0.01, shader, crl::V3D(0.5, 0.5, 0.5), 0.2);
        //             }
        //     }

        //     // draw foot contact
        //     if (showContactState)
        //         for (const auto &footMarkerName : footMarkerNames) {
        //             bool isSwing = false;
        //             for (const auto &swing : footSteps[footMarkerName]) {
        //                 if (swing.first <= frameIdx * c3dClips[selectedC3dClipIdx]->getFrameTimeStep() &&
        //                     frameIdx * c3dClips[selectedC3dClipIdx]->getFrameTimeStep() <= swing.second)
        //                     isSwing = true;
        //             }
        //             if (isSwing)
        //                 crl::gui::drawSphere(c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(footMarkerName.c_str())->state.pos,  //
        //                                      0.05, shader, crl::V3D(1, 0, 1), 0.5);
        //             else
        //                 crl::gui::drawSphere(c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(footMarkerName.c_str())->state.pos,  //
        //                                      0.05, shader, crl::V3D(0, 1, 0), 0.5);
        //         }
        //     for (const auto &footMarkerName : footMarkerNames) {
        //         bool isSwing = false;
        //         for (const auto &swing : footSteps[footMarkerName]) {
        //             if (swing.first <= frameIdx * c3dClips[selectedC3dClipIdx]->getFrameTimeStep() &&
        //                 frameIdx * c3dClips[selectedC3dClipIdx]->getFrameTimeStep() <= swing.second)
        //                 isSwing = true;
        //         }
        //         if (isSwing)
        //             crl::gui::drawSphere(c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(footMarkerName.c_str())->state.pos,  //
        //                                  0.05, shader, crl::V3D(1, 0, 1), 0.5);
        //         else
        //             crl::gui::drawSphere(c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(footMarkerName.c_str())->state.pos,  //
        //                                  0.05, shader, crl::V3D(0, 1, 0), 0.5);
        //     }

        //     // selected marker (for debugging)
        //     if (selectedMarkerIdx > -1)
        //         crl::gui::drawSphere(c3dClips[selectedC3dClipIdx]->getModel()->getMarker(selectedMarkerIdx)->state.pos, 0.05, shader, crl::V3D(1, 0, 0), 0.5);
        // }
        if (currentMatchClip > -1){
            bvhClips[currentMatchClip]->draw(shader, currentMatchFrame);
            controller.drawTrajectory(shader);
        }
        else if (bvhClips.size() != 0) {
            bvhClips[0]->draw(shader, 0);
            currentMatchClip = 0;
            currentMatchFrame = 0;
            currentMatchIdx = 0;
            processBVHClip();
            controller.drawTrajectory(shader);
        }
    }

    void drawPlots() {
        if (selectedBvhClipIdx == -1 && selectedC3dClipIdx == -1)
            return;
        double t = 0;
        if (selectedBvhClipIdx > -1)
            t = frameIdx * bvhClips[selectedBvhClipIdx]->getFrameTimeStep();
        if (selectedC3dClipIdx > -1)
            t = frameIdx * c3dClips[selectedC3dClipIdx]->getFrameTimeStep();

        ImGui::SetNextWindowPos(ImVec2(this->width - 720, 20), ImGuiCond_Once);
        ImGui::SetNextWindowSize(ImVec2(720, this->height - 20 - this->consoleHeight), ImGuiCond_Once);
        ImGui::Begin("Plot");
        ImGui::Text("Frame time = %lf", t);
        baseHeightPlot.draw(t);
        baseSpeedPlot.draw(t);
        feetHeightPlot.draw(t);
        feetVelocityPlot.draw(t);
        feetAccelerationPlot.draw(t);
        ImGui::End();
        swingTimeline.draw(t);
    }

    void drawImGui() override {
        crl::gui::ShadowApplication::drawImGui();

        ImGui::Begin("Main Menu");
        ImGui::Checkbox("Follow Character", &followCharacter);
        if (ImGui::CollapsingHeader("Mocap Data", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Import")) {
                crl::Logger::consolePrint("Loading mocap data...\n");
                fileDialog.Open();
            }
            fileDialog.Display();
            if (fileDialog.HasSelected()) {
                loadMocapFile(fileDialog.GetSelected());
                fileDialog.ClearSelected();
            }
            ImGui::SameLine();
            if (ImGui::Button("Clear")) {
                crl::Logger::consolePrint("Clear loaded mocap data...\n");
                bvhClips.clear();
                c3dClips.clear();
                selectedBvhClipIdx = -1;
                selectedC3dClipIdx = -1;
            }
            if (ImGui::ListBoxHeader("BVH\nMocap Clips##MocapClips", 10)) {
                for (int i = 0; i < bvhClips.size(); i++) {
                    bool isSelected = (selectedBvhClipIdx == (int)i);
                    if (ImGui::Selectable(bvhClips[i]->getName().c_str(), isSelected)) {
                        selectedBvhClipIdx = i;
                        selectedC3dClipIdx = -1;
                        selectedMarkerIdx = -1;
                        frameIdx = 0;
                        processBVHClip();
                    }
                }

                ImGui::ListBoxFooter();
            }
            if (ImGui::ListBoxHeader("C3D\nMocap Clips##MocapClips", 10)) {
                for (int i = 0; i < c3dClips.size(); i++) {
                    bool isSelected = (selectedC3dClipIdx == (int)i);
                    if (ImGui::Selectable(c3dClips[i]->getName().c_str(), isSelected)) {
                        selectedBvhClipIdx = -1;
                        selectedC3dClipIdx = i;
                        selectedMarkerIdx = -1;
                        frameIdx = 0;
                        processC3DClip();
                    }
                }
                ImGui::ListBoxFooter();
            }
            if (selectedC3dClipIdx > -1) {
                if (ImGui::ListBoxHeader("Mocap\nMarkers##MocapClips", 10)) {
                    for (int i = 0; i < c3dClips[selectedC3dClipIdx]->getModel()->getMarkerCount(); i++) {
                        bool isSelected = (selectedMarkerIdx == (int)i);
                        if (ImGui::Selectable(c3dClips[selectedC3dClipIdx]->getModel()->getMarker(i)->name.c_str(), isSelected)) {
                            selectedMarkerIdx = i;
                        }
                    }
                    ImGui::ListBoxFooter();
                }
            }
        }

        if (ImGui::CollapsingHeader("Post Processing", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::SliderDouble("Foot Height Threshold", &footHeightThreshold, 0.0, 0.2, "%.2f")) {
                processBVHClip();
                processC3DClip();
            }
            if (ImGui::SliderDouble("Foot Velocity Threshold", &footVelocityThreshold, 0.0, 2.0, "%.2f")) {
                processBVHClip();
                processC3DClip();
            }
        }

        if (ImGui::CollapsingHeader("Draw")) {
            ImGui::Checkbox("Show Coordinate Frames", &showCoordinateFrames);
            ImGui::Checkbox("Show Virtual Limb", &showVirtualLimbs);
            ImGui::Checkbox("Show Contact State", &showContactState);
        }

        ImGui::End();
        drawPlots();
    }

    bool mouseButtonPressed(int button, int mods) override {
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            crl::P3D rayOrigin;
            crl::V3D rayDirection;
            crl::Ray mouseRay(rayOrigin, rayDirection);
            camera.getRayFromScreenCoordinates(mouseState.lastMouseX, mouseState.lastMouseY, rayOrigin, rayDirection);
            if (selectedC3dClipIdx > -1) {
                crl::P3D selectedPoint;
                crl::mocap::MocapMarker *marker = c3dClips[selectedC3dClipIdx]->getModel()->getFirstJointHitByRay(mouseRay, selectedPoint);
                if (marker)
                    crl::Logger::consolePrint("Selected marker: %s\n", marker->name.c_str());
            }
        }
        return true;
    }

    bool drop(int count, const char **fileNames) override {
        for (uint i = 0; i < count; i++)
            loadMocapFile(fs::path(fileNames[i]));
        return true;
    }

    void loadMocapFile(const fs::path &path) {
        int cnt = 0;
        if (is_directory(path)) {
            for (const auto &entry : fs::directory_iterator(path)) {
                if (loadSingleMocapFile(entry.path()))
                    cnt++;
            }
        } else {
            if (loadSingleMocapFile(path))
                cnt++;
        }

        // sort by name
        std::sort(bvhClips.begin(),
                  bvhClips.end(),  //
                  [](const auto &a, const auto &b) { return a->getName() < b->getName(); });
        std::sort(c3dClips.begin(),
                  c3dClips.end(),  //
                  [](const auto &a, const auto &b) { return a->getName() < b->getName(); });

        // reset indices
        selectedBvhClipIdx = -1;
        selectedC3dClipIdx = -1;
        frameIdx = 0;
        processAllBVHClip();
        crl::Logger::consolePrint("Imported %d clips.\n", cnt);
    }

    // void transferSkeletonInformationToController() {
    //     // call after get the motion matching result ?
    //     // auto state = bvhClips[currentMatchClip]->getState(currentMatchFrame);
    //     auto *sk = bvhClips[currentMatchClip]->getModel();
    //     sk->setState(&bvhClips[currentMatchClip]->getState(currentMatchFrame));
    //     std::vector<crl::V3D> markervel;
    //     std::vector<crl::Quaternion> markerQuat;
    //     std::vector<crl::P3D> markerpos;
    //     for (int i = 0; i < DBmarkerNames.size(); i++) {
    //         const auto &name = DBmarkerNames[i];
    //         if (const auto joint = sk->getMarkerByName(name.c_str())) {
    //             crl::P3D eepos = joint->state.pos;
    //             crl::V3D eevel = joint->state.velocity;
    //             crl::Quaternion eequat = joint->state.orientation;
    //             markervel.push_back(eevel);
    //             markerpos.push_back(eepos);
    //             markerQuat.push_back(eequat);
    //         }
    //     }
    //     controller.updateNewSkeletionInfo(markerpos, markervel, markerQuat);  // 可能要改orz
    // }

private:
    bool loadSingleMocapFile(const fs::path &path) {
        try {
            if (path.extension() == ".bvh")
                bvhClips.push_back(std::make_unique<crl::mocap::BVHClip>(path));
            else if (path.extension() == ".c3d")
                c3dClips.push_back(std::make_unique<crl::mocap::C3DClip>(path));
        } catch (...) {
            crl::Logger::consolePrint("Failed to load %s file.\n", path.c_str());
            return false;
        }
        return true;
    }

    void processBVHClip() {
        // if (selectedBvhClipIdx == -1)
        //     return;

        // footSteps.clear();
        // // footMarkerNames = {"LeftHand", "LeftFoot", "RightHand", "RightFoot"};
        // footMarkerNames = {"LeftToe", "RightToe"};
        // linkNames = {};
        // virtualLinkNames = {};

        // // initialize plots
        // baseHeightPlot.clearAll();
        // baseSpeedPlot.clearAll();
        // feetHeightPlot.clearAll();
        // feetVelocityPlot.clearAll();
        // feetAccelerationPlot.clearAll();
        // baseHeightPlot.addLineSpec({"h", [](const auto &d) { return (float)d.y; }});
        // baseSpeedPlot.addLineSpec({"forward", [](const auto &d) { return (float)d[0]; }});
        // baseSpeedPlot.addLineSpec({"sideways", [](const auto &d) { return (float)d[1]; }});
        // baseSpeedPlot.addLineSpec({"turning", [](const auto &d) { return (float)d[2]; }});

        // baseHeightPlot.drawVerticalGuide = true;
        // baseHeightPlot.setMaxSize(bvhClips[selectedBvhClipIdx]->getFrameCount());
        // baseSpeedPlot.drawVerticalGuide = true;
        // baseSpeedPlot.setMaxSize(bvhClips[selectedBvhClipIdx]->getFrameCount());
        // feetHeightPlot.drawVerticalGuide = true;
        // feetHeightPlot.setMaxSize(bvhClips[selectedBvhClipIdx]->getFrameCount());
        // feetVelocityPlot.drawVerticalGuide = true;
        // feetVelocityPlot.setMaxSize(bvhClips[selectedBvhClipIdx]->getFrameCount());
        // feetAccelerationPlot.drawVerticalGuide = true;
        // feetAccelerationPlot.setMaxSize(bvhClips[selectedBvhClipIdx]->getFrameCount());
        // for (uint i = 0; i < footMarkerNames.size(); i++) {
        //     feetHeightPlot.addLineSpec({footMarkerNames[i], [i](const auto &d) { return (float)d[i]; }});
        //     feetVelocityPlot.addLineSpec({footMarkerNames[i], [i](const auto &d) { return (float)d[i]; }});
        //     feetAccelerationPlot.addLineSpec({footMarkerNames[i] + " x", [i](const auto &d) { return (float)d[3 * i]; }});
        //     feetAccelerationPlot.addLineSpec({footMarkerNames[i] + " y", [i](const auto &d) { return (float)d[3 * i + 1]; }});
        //     feetAccelerationPlot.addLineSpec({footMarkerNames[i] + " z", [i](const auto &d) { return (float)d[3 * i + 2]; }});
        // }

        // // extracting foot velocity and position
        // auto *sk = bvhClips[selectedBvhClipIdx]->getModel();
        // double t = 0;
        // const auto &speed = crl::mocap::extractRootSpeedProfileFromBVHClip(bvhClips[selectedBvhClipIdx].get());

        // std::vector<crl::V3D> footVelocityBackup;
        // for (int i = 0; i < footMarkerNames.size(); i++)
        //     footVelocityBackup.push_back(crl::V3D());

        // for (int i = 0; i < bvhClips[selectedBvhClipIdx]->getFrameCount(); i++) {
        //     sk->setState(&bvhClips[selectedBvhClipIdx]->getState(i));

        //     baseHeightPlot.addData((float)t, sk->root->state.pos);
        //     baseSpeedPlot.addData((float)t, speed.evaluate_linear(t));
        //     crl::dVector footHeights(footMarkerNames.size());
        //     crl::dVector footVelocities(footMarkerNames.size());
        //     crl::dVector footAcceleration(footMarkerNames.size() * 3);
        //     for (int j = 0; j < footMarkerNames.size(); j++) {
        //         const auto &name = footMarkerNames[j];
        //         if (const auto joint = sk->getMarkerByName(name.c_str())) {
        //             crl::P3D eepos = joint->state.getWorldCoordinates(joint->endSites[0].endSiteOffset);
        //             crl::V3D eevel = joint->state.getVelocityForPoint_local(joint->endSites[0].endSiteOffset);
        //             footHeights[j] = eepos.y;
        //             footVelocities[j] = eevel.norm();

        //             if (i == 0) {
        //                 footAcceleration[3 * j] = 0;
        //                 footAcceleration[3 * j + 1] = 0;
        //                 footAcceleration[3 * j + 2] = 0;
        //             } else {
        //                 crl::V3D acc = (eevel - footVelocityBackup[j]) / bvhClips[selectedBvhClipIdx]->getFrameTimeStep();
        //                 footAcceleration[3 * j] = acc.x();
        //                 footAcceleration[3 * j + 1] = acc.y();
        //                 footAcceleration[3 * j + 2] = acc.z();
        //             }

        //             footVelocityBackup[j] = eevel;
        //         }
        //     }

        //     feetHeightPlot.addData((float)t, footHeights);
        //     feetVelocityPlot.addData((float)t, footVelocities);
        //     feetAccelerationPlot.addData((float)t, footAcceleration);
        //     t += bvhClips[selectedBvhClipIdx]->getFrameTimeStep();
        // }

        // // extracting swing sequences
        // for (const auto &name : footMarkerNames) {
        //     auto contactInfos = crl::mocap::extractFootContactStateFromBVHClip(  //
        //         bvhClips[selectedBvhClipIdx].get(), name, footHeightThreshold, footVelocityThreshold);
        //     footSteps[name] = crl::mocap::convertFootSwingSequenceFromFootContactStates(contactInfos);
        // }
        if (currentMatchClip == -1)
            return;

        footSteps.clear();
        // footMarkerNames = {"LeftHand", "LeftFoot", "RightHand", "RightFoot"};
        footMarkerNames = {"LeftToe", "RightToe"};
        linkNames = {};
        virtualLinkNames = {};

        // initialize plots
        baseHeightPlot.clearAll();
        baseSpeedPlot.clearAll();
        feetHeightPlot.clearAll();
        feetVelocityPlot.clearAll();
        feetAccelerationPlot.clearAll();
        baseHeightPlot.addLineSpec({"h", [](const auto &d) { return (float)d.y; }});
        baseSpeedPlot.addLineSpec({"forward", [](const auto &d) { return (float)d[0]; }});
        baseSpeedPlot.addLineSpec({"sideways", [](const auto &d) { return (float)d[1]; }});
        baseSpeedPlot.addLineSpec({"turning", [](const auto &d) { return (float)d[2]; }});

        baseHeightPlot.drawVerticalGuide = true;
        baseHeightPlot.setMaxSize(bvhClips[currentMatchClip]->getFrameCount());
        baseSpeedPlot.drawVerticalGuide = true;
        baseSpeedPlot.setMaxSize(bvhClips[currentMatchClip]->getFrameCount());
        feetHeightPlot.drawVerticalGuide = true;
        feetHeightPlot.setMaxSize(bvhClips[currentMatchClip]->getFrameCount());
        feetVelocityPlot.drawVerticalGuide = true;
        feetVelocityPlot.setMaxSize(bvhClips[currentMatchClip]->getFrameCount());
        feetAccelerationPlot.drawVerticalGuide = true;
        feetAccelerationPlot.setMaxSize(bvhClips[currentMatchClip]->getFrameCount());
        for (uint i = 0; i < footMarkerNames.size(); i++) {
            feetHeightPlot.addLineSpec({footMarkerNames[i], [i](const auto &d) { return (float)d[i]; }});
            feetVelocityPlot.addLineSpec({footMarkerNames[i], [i](const auto &d) { return (float)d[i]; }});
            feetAccelerationPlot.addLineSpec({footMarkerNames[i] + " x", [i](const auto &d) { return (float)d[3 * i]; }});
            feetAccelerationPlot.addLineSpec({footMarkerNames[i] + " y", [i](const auto &d) { return (float)d[3 * i + 1]; }});
            feetAccelerationPlot.addLineSpec({footMarkerNames[i] + " z", [i](const auto &d) { return (float)d[3 * i + 2]; }});
        }

        // extracting foot velocity and position
        auto *sk = bvhClips[currentMatchClip]->getModel();
        double t = 0;
        const auto &speed = crl::mocap::extractRootSpeedProfileFromBVHClip(bvhClips[currentMatchClip].get());

        std::vector<crl::V3D> footVelocityBackup;
        for (int i = 0; i < footMarkerNames.size(); i++)
            footVelocityBackup.push_back(crl::V3D());

        for (int i = 0; i < bvhClips[currentMatchClip]->getFrameCount(); i++) {
            sk->setState(&bvhClips[currentMatchClip]->getState(i));

            baseHeightPlot.addData((float)t, sk->root->state.pos);
            baseSpeedPlot.addData((float)t, speed.evaluate_linear(t));
            crl::dVector footHeights(footMarkerNames.size());
            crl::dVector footVelocities(footMarkerNames.size());
            crl::dVector footAcceleration(footMarkerNames.size() * 3);
            for (int j = 0; j < footMarkerNames.size(); j++) {
                const auto &name = footMarkerNames[j];
                if (const auto joint = sk->getMarkerByName(name.c_str())) {
                    crl::P3D eepos = joint->state.getWorldCoordinates(joint->endSites[0].endSiteOffset);
                    crl::V3D eevel = joint->state.getVelocityForPoint_local(joint->endSites[0].endSiteOffset);
                    footHeights[j] = eepos.y;
                    footVelocities[j] = eevel.norm();

                    if (i == 0) {
                        footAcceleration[3 * j] = 0;
                        footAcceleration[3 * j + 1] = 0;
                        footAcceleration[3 * j + 2] = 0;
                    } else {
                        crl::V3D acc = (eevel - footVelocityBackup[j]) / bvhClips[currentMatchClip]->getFrameTimeStep();
                        footAcceleration[3 * j] = acc.x();
                        footAcceleration[3 * j + 1] = acc.y();
                        footAcceleration[3 * j + 2] = acc.z();
                    }

                    footVelocityBackup[j] = eevel;
                }
            }

            feetHeightPlot.addData((float)t, footHeights);
            feetVelocityPlot.addData((float)t, footVelocities);
            feetAccelerationPlot.addData((float)t, footAcceleration);
            t += bvhClips[currentMatchClip]->getFrameTimeStep();
        }

        // extracting swing sequences
        for (const auto &name : footMarkerNames) {
            auto contactInfos = crl::mocap::extractFootContactStateFromBVHClip(  //
                bvhClips[currentMatchClip].get(), name, footHeightThreshold, footVelocityThreshold);
            footSteps[name] = crl::mocap::convertFootSwingSequenceFromFootContactStates(contactInfos);
        }
    }

    void processAllBVHClip() {
        DBtotalFrame = 0;
        for (uint bvhidx = 0; bvhidx < bvhClips.size(); bvhidx++) {
            DBtotalFrame += bvhClips[bvhidx]->getFrameCount();
        }
        std::vector<std::string> DBmarkerNames = {"LeftFoot", "RightFoot", "Hips"};
        DBMatching.setZero(DBtotalFrame, DBfeatVecDim);
        Feature_mean.setZero(DBfeatVecDim);
        Feature_std.setZero(DBfeatVecDim);

        int counter = 0;
        accumulatedFrameNum.setZero(bvhClips.size());
        for (uint bvhidx = 0; bvhidx < bvhClips.size(); bvhidx++) {
            int clipFrameNum = bvhClips[bvhidx]->getFrameCount();
            if (bvhidx == 0)
            {
                accumulatedFrameNum[bvhidx] = clipFrameNum;
            }
            else
            {
                accumulatedFrameNum[bvhidx] = accumulatedFrameNum[bvhidx - 1] + clipFrameNum;
            }

            Eigen::MatrixXd footJointPos;
            footJointPos.setZero(clipFrameNum, 6);
            Eigen::MatrixXd footJointVel;
            footJointVel.setZero(clipFrameNum, 6);
            Eigen::MatrixXd hipJointVel;
            hipJointVel.setZero(clipFrameNum, 3);
            Eigen::MatrixXd hipJointPos;
            hipJointPos.setZero(clipFrameNum, 3);
            Eigen::MatrixXd trajPoseonGround;
            trajPoseonGround.setZero(clipFrameNum, 3);
            std::vector<crl::Quaternion> hipQuaternions;

            auto *sk = bvhClips[bvhidx]->getModel();

            for (int i = 0; i < clipFrameNum; i++) {
                sk->setState(&bvhClips[bvhidx]->getState(i));

                for (int j = 0; j < DBmarkerNames.size(); j++) {
                    const auto &name = DBmarkerNames[j];
                    if (const auto joint = sk->getMarkerByName(name.c_str())) {
                        crl::P3D eepos = joint->state.pos;
                        crl::V3D eevel = joint->state.velocity;
                        if (j == 0)  // left foot
                        {
                            footJointPos(i, 0) = eepos.x;
                            footJointPos(i, 1) = eepos.y;
                            footJointPos(i, 2) = eepos.z;
                            footJointVel(i, 0) = eevel.x();
                            footJointVel(i, 1) = eevel.y();
                            footJointVel(i, 2) = eevel.z();
                        } else if (j == 1)  // right foot
                        {
                            footJointPos(i, 3) = eepos.x;
                            footJointPos(i, 4) = eepos.y;
                            footJointPos(i, 5) = eepos.z;
                            footJointVel(i, 3) = eevel.x();
                            footJointVel(i, 4) = eevel.y();
                            footJointVel(i, 5) = eevel.z();
                        } else if (j == 2)  // Hips
                        {
                            hipJointPos(i, 0) = eepos.x;
                            hipJointPos(i, 1) = eepos.y;
                            hipJointPos(i, 2) = eepos.z;
                            hipJointVel(i, 0) = eevel.x();
                            hipJointVel(i, 1) = eevel.y();
                            hipJointVel(i, 2) = eevel.z();
                            crl::Quaternion eequat = joint->state.orientation;
                            hipQuaternions.push_back(eequat);
                        }
                    }
                }
                trajPoseonGround(i, 0) = (footJointPos(i, 0) + footJointPos(i, 3)) / 2;
                trajPoseonGround(i, 1) = 0;
                trajPoseonGround(i, 2) = (footJointPos(i, 2) + footJointPos(i, 5)) / 2;
            }
            /* compute future traj orientation for every frame and save in DBMatching
            // assert(hipQuaternions.size() == clipFrameNum);

            // for (int i = 0; i < clipFrameNum; i++)
            // {
            //     crl::Quaternion localInverse = hipQuaternions[i].inverse();
            //     Eigen::Vector3d positiveDirection(1, 0, 0);
                

                // if (i + 10 < clipFrameNum)
                // {
                //     crl::Quaternion q10 = hipQuaternions[i + 10];
                //     crl::Quaternion q_relative_10 = q10 * localInverse;
                //     Eigen::Vector3d face_direction_10 = q_relative_10 * positiveDirection;
                //     face_direction_10(1) = 0;
                //     face_direction_10.normalize();
                //     DBMatching(counter + i, 6) = face_direction_10(0);
                //     DBMatching(counter + i, 7) = face_direction_10(2);
                // }
                // if (i + 20 < clipFrameNum)
                // {
                //     crl::Quaternion q20 = hipQuaternions[i + 20];
                //     crl::Quaternion q_relative_20 = q20 * localInverse;
                //     Eigen::Vector3d face_direction_20 = q_relative_20 * positiveDirection;
                //     face_direction_20(1) = 0;
                //     face_direction_20.normalize();
                //     DBMatching(counter + i, 8) = face_direction_20(0);
                //     DBMatching(counter + i, 9) = face_direction_20(2);
                // }
                // if (i + 30 < clipFrameNum)
                // {
                //     crl::Quaternion q30 = hipQuaternions[i + 30];
                //     crl::Quaternion q_relative_30 = q30 * localInverse;
                //     Eigen::Vector3d face_direction_30 = q_relative_30 * positiveDirection;
                //     face_direction_30(1) = 0;
                //     face_direction_30.normalize();
                //     DBMatching(counter + i, 10) = face_direction_30(0);
                //     DBMatching(counter + i, 11) = face_direction_30(2);
                // }
            // }*/

            // future trajectory positions
            for (int i = 0; i < clipFrameNum; i++)
            {
                crl::Quaternion localInverse = hipQuaternions[i].inverse();
                Eigen::Vector3d trajPos = trajPoseonGround.row(i);
                if (i + 10 < clipFrameNum)
                {
                    Eigen::Vector3d trajPos10 = trajPoseonGround.row(i + 10);
                    Eigen::Vector3d localRelativePos10 = localInverse * (trajPos10 - trajPos);
                    DBMatching(counter + i, 0) = localRelativePos10(0);
                    DBMatching(counter + i, 1) = localRelativePos10(2);
                }
                else
                {
                    break;
                }
                if (i + 20 < clipFrameNum)
                {
                    Eigen::Vector3d trajPos20 = trajPoseonGround.row(i + 20);
                    Eigen::Vector3d localRelativePos20 = localInverse * (trajPos20 - trajPos);
                    DBMatching(counter + i, 2) = localRelativePos20(0);
                    DBMatching(counter + i, 3) = localRelativePos20(2);
                }
                else
                {
                    continue;
                }
                if (i + 30 < clipFrameNum)
                {
                    Eigen::Vector3d trajPos30 = trajPoseonGround.row(i + 30);
                    Eigen::Vector3d localRelativePos30 = localInverse * (trajPos30 - trajPos);
                    DBMatching(counter + i, 4) = localRelativePos30(0);
                    DBMatching(counter + i, 5) = localRelativePos30(2);
                }
            }    

            // future trajectory orientations
            for (int i = 0; i < clipFrameNum; i++)
            {
                crl::Quaternion localInverse = hipQuaternions[i].inverse();
                if (i + 10 < clipFrameNum)
                {
                    Eigen::Vector3d hipVel = hipJointVel.row(i + 10);
                    Eigen::Vector3d localRelativeDir10 = localInverse * hipVel;
                    localRelativeDir10.normalize();
                    DBMatching(counter + i, 6) = localRelativeDir10(0);
                    DBMatching(counter + i, 7) = localRelativeDir10(2);
                }
                else
                {
                    break;
                }
                if (i + 20 < clipFrameNum)
                {
                    Eigen::Vector3d hipVel = hipJointVel.row(i + 20);
                    Eigen::Vector3d localRelativeDir20 = localInverse * hipVel;
                    localRelativeDir20.normalize();
                    DBMatching(counter + i, 8) = localRelativeDir20(0);
                    DBMatching(counter + i, 9) = localRelativeDir20(2);
                }
                else
                {
                    continue;
                }
                if (i + 30 < clipFrameNum)
                {
                    Eigen::Vector3d hipVel = hipJointVel.row(i + 30);
                    Eigen::Vector3d localRelativeDir30 = localInverse * hipVel;
                    localRelativeDir30.normalize();
                    DBMatching(counter + i, 10) = localRelativeDir30(0);
                    DBMatching(counter + i, 11) = localRelativeDir30(2);
                }
            }    

            // foot joint
            for (int i = 0; i < clipFrameNum; i++)
            {
                crl::Quaternion localInverse = hipQuaternions[i].inverse();
                Eigen::Vector3d leftFoot = footJointPos.row(i).head(3);
                Eigen::Vector3d rightFoot = footJointPos.row(i).tail(3);
                Eigen::Vector3d hipPos = hipJointPos.row(i);
                footJointPos.row(i).segment(0, 3) = localInverse * (leftFoot - hipPos);
                footJointPos.row(i).segment(3, 3) = localInverse * (rightFoot - hipPos);
            }
            // compute foot joint velocity and hip joint velocity local to character
            for (int i = 0; i < clipFrameNum; i++)
            {
                crl::Quaternion localInverse = hipQuaternions[i].inverse();
                Eigen::Vector3d localHipVel = localInverse * hipJointVel.row(i);
                Eigen::Vector3d localLeftFootVelocity = localInverse * footJointVel.row(i).head(3);
                Eigen::Vector3d localRightFootVelocity = localInverse * footJointVel.row(i).tail(3);
                hipJointVel.row(i) << localHipVel;
                footJointVel.row(i).segment(0, 3) = localLeftFootVelocity;
                footJointVel.row(i).segment(3, 3) = localRightFootVelocity;   
            }
        
            // defined for 30Hz, stores the future 10\20\30 frames
            DBMatching.block(counter, 12, clipFrameNum, 6) = footJointPos;
            DBMatching.block(counter, 18, clipFrameNum, 6) = footJointVel;
            DBMatching.block(counter, 24, clipFrameNum, 3) = hipJointVel;
            counter += clipFrameNum;
        }

        for (int i = 0; i < DBMatching.cols(); i++) {
            Eigen::VectorXd col = DBMatching.col(i);
            double mean = col.mean();                           // Compute mean
            double std = (col.array() - mean).square().mean();  // Compute variance
            std = std > 0 ? sqrt(std) : 0;                      // Compute standard deviation from variance

            DBMatching.col(i) = (DBMatching.col(i).array() - mean) / std;
            Feature_mean[i] = mean;
            Feature_std[i] = std;
        }
    }

    void MMSearchforIndex() { 
        Eigen::RowVectorXd normalizedQuery = (currentQueryVector.array() - Feature_mean.array()) / Feature_std.array();
        Eigen::VectorXd norms = (DBMatching.rowwise() - normalizedQuery).rowwise().norm();
        int index;
        double minnorm = norms.minCoeff(&index); 
        currentMatchIdx = index;
        MMSearchforClipandFrame();
    }

    void MMSearchforClipandFrame()
    {
        for (int i = 0; i < accumulatedFrameNum.size(); i++)
        {
            if (currentMatchIdx < accumulatedFrameNum[i])
            {
                currentMatchClip = i;
                currentMatchFrame = i > 0 ? currentMatchIdx - accumulatedFrameNum[i - 1] : currentMatchIdx;
                break;
            }
        }
    }
    // void find

    void processC3DClip() {
        if (selectedC3dClipIdx == -1)
            return;

        footSteps.clear();
        std::unique_ptr<crl::mocap::C3DClip> &clip = c3dClips[selectedC3dClipIdx];
        const std::vector<double> gaussianKernel = {0.031091376664711, 0.041904779550059, 0.053944239501404, 0.066326105267268, 0.077890021426103,
                                                    0.087364914695958, 0.093594471765148, 0.095768182258699, 0.093594471765148, 0.087364914695958,
                                                    0.077890021426103, 0.066326105267268, 0.053944239501404, 0.041904779550059, 0.031091376664711};

        const std::string rootName = "S_1";
        footMarkerNames = {"lf_fetlock", "lh_fetlock", "rf_fetlock", "rh_fetlock"};
        linkNames = {{"S_1", "Th_6"},
                     {"S_1", "S_6"},
                     {"S_1", "r_sacrum"},
                     {"S_1", "l_sacrum"},
                     {"l_sacrum", "l_hip"},
                     {"l_hip", "l_knee"},
                     {"l_knee", "l_tarsus"},
                     {"l_tarsus", "lh_fetlock"},
                     {"Th_6", "l_spina_ud"},
                     {"l_spina_ud", "l_spina_d"},
                     {"l_spina_d", "l_shoulder"},
                     {"l_shoulder", "l_elbow"},
                     {"l_elbow", "l_carpus"},
                     {"l_carpus", "lf_fetlock"}};
        virtualLinkNames = {{"Th_6", "rf_fetlock"}, {"r_sacrum", "rh_fetlock"}};

        double treadmill = 0;
        if (clip->getName().rfind("wa") != std::string::npos)
            treadmill = 1.1;
        if (clip->getName().rfind("tr") != std::string::npos)
            treadmill = 2.1;

        // initialize plots
        baseHeightPlot.clearAll();
        baseSpeedPlot.clearAll();
        feetHeightPlot.clearAll();
        feetVelocityPlot.clearAll();
        baseHeightPlot.addLineSpec({"h", [](const auto &d) { return (float)d.y; }});
        baseSpeedPlot.addLineSpec({"forward", [](const auto &d) { return (float)d[0]; }});
        baseSpeedPlot.addLineSpec({"sideways", [](const auto &d) { return (float)d[1]; }});
        baseSpeedPlot.addLineSpec({"turning", [](const auto &d) { return (float)d[2]; }});
        baseSpeedPlot.addLineSpec({"forward post", [](const auto &d) { return (float)d[3]; }});
        baseSpeedPlot.addLineSpec({"sideways post", [](const auto &d) { return (float)d[4]; }});
        baseSpeedPlot.addLineSpec({"turning post", [](const auto &d) { return (float)d[5]; }});

        baseHeightPlot.drawVerticalGuide = true;
        baseHeightPlot.setMaxSize(clip->getFrameCount());
        baseSpeedPlot.drawVerticalGuide = true;
        baseSpeedPlot.setMaxSize(clip->getFrameCount());
        feetHeightPlot.drawVerticalGuide = true;
        feetHeightPlot.setMaxSize(clip->getFrameCount());
        feetVelocityPlot.drawVerticalGuide = true;
        feetVelocityPlot.setMaxSize(clip->getFrameCount());
        for (uint i = 0; i < footMarkerNames.size(); i++) {
            feetHeightPlot.addLineSpec({footMarkerNames[i], [i](const auto &d) { return (float)d[i]; }});
            feetVelocityPlot.addLineSpec({footMarkerNames[i], [i](const auto &d) { return (float)d[i * 2]; }});
            feetVelocityPlot.addLineSpec({footMarkerNames[i] + " post", [i](const auto &d) { return (float)d[i * 2 + 1]; }});
        }

        auto *model = clip->getModel();
        double t = 0;

        // populate data
        std::map<std::string, std::vector<crl::V3D>> footVelocityMap;      // from data
        std::map<std::string, std::vector<crl::V3D>> footVelocityPostMap;  // after post-processing
        for (int i = 0; i < clip->getFrameCount(); i++) {
            model->setState(&clip->getState(i));

            // TODO: consider heading...
            crl::V3D rootVelocity = model->getMarkerByName(rootName.c_str())->state.velocity;
            crl::dVector velocityData(6);
            velocityData << rootVelocity.x(), rootVelocity.z(), 0, rootVelocity.x() + treadmill, rootVelocity.z(), 0;
            baseHeightPlot.addData((float)t, model->getMarkerByName(rootName.c_str())->state.pos);
            baseSpeedPlot.addData((float)t, velocityData);

            crl::dVector footHeights(footMarkerNames.size());
            for (int j = 0; j < footMarkerNames.size(); j++) {
                const auto &name = footMarkerNames[j];
                if (const auto joint = model->getMarkerByName(name.c_str())) {
                    crl::P3D eepos = joint->state.pos;
                    crl::V3D eevel = joint->state.velocity;
                    footHeights[j] = eepos.y;
                    footVelocityMap[name].push_back(eevel);
                }
            }

            feetHeightPlot.addData((float)t, footHeights);
            t += clip->getFrameTimeStep();
        }

        // filtering for denoise
        for (int i = 0; i < clip->getFrameCount(); i++) {
            for (const auto &footMarkerName : footMarkerNames) {
                crl::V3D ret(0, 0, 0);
                for (int offset = -7; offset <= 7; ++offset) {
                    int idx = i + offset;
                    if (idx >= 0 && idx < (int)clip->getFrameCount()) {
                        // Add the forward speed values multiplied by the corresponding kernel coefficient otherwise "add a zero" (i.e. assume zero padding)
                        ret += gaussianKernel[offset + 7] * footVelocityMap[footMarkerName][idx];
                    }
                }
                footVelocityPostMap[footMarkerName].push_back(ret + crl::V3D(treadmill, 0, 0));
            }
        }

        // populate foot velocity plot data (after filtering)
        t = 0;
        for (int i = 0; i < clip->getFrameCount(); i++) {
            crl::dVector footHeights(footMarkerNames.size());
            crl::dVector footVelocities(footMarkerNames.size() * 2);
            for (int j = 0; j < footMarkerNames.size(); j++) {
                const auto &name = footMarkerNames[j];
                crl::V3D eevel = footVelocityMap[name][i];
                crl::V3D eevelPost = footVelocityPostMap[name][i];
                footVelocities[j * 2] = eevel.norm();
                footVelocities[j * 2 + 1] = eevelPost.norm();
            }

            feetVelocityPlot.addData((float)t, footVelocities);
            t += clip->getFrameTimeStep();
        }

        // populate footstep
        for (const auto &footMarkerName : footMarkerNames) {
            std::vector<std::pair<double, bool>> contactYN;
            contactYN.reserve(clip->getFrameCount());
            auto *sk = clip->getModel();

            for (int i = 0; i < clip->getFrameCount(); i++) {
                sk->setState(&clip->getState(i));
                if (const auto marker = sk->getMarkerByName(footMarkerName.c_str())) {
                    crl::P3D eepos = marker->state.pos;
                    crl::V3D eevel = footVelocityPostMap[footMarkerName][i];
                    eevel.y() = 0;

                    if (eepos.y < footHeightThreshold && eevel.norm() < footVelocityThreshold)
                        contactYN.emplace_back(clip->getFrameTimeStep() * i, true);  // contact
                    else
                        contactYN.emplace_back(clip->getFrameTimeStep() * i, false);  // swing
                }
            }

            footSteps[footMarkerName] = crl::mocap::convertFootSwingSequenceFromFootContactStates(contactYN);
        }
    }

private:
    ImGui::FileBrowser fileDialog;
    std::vector<std::unique_ptr<crl::mocap::BVHClip>> bvhClips;
    std::vector<std::unique_ptr<crl::mocap::C3DClip>> c3dClips;
    int selectedBvhClipIdx = -1;
    int selectedC3dClipIdx = -1;
    int selectedMarkerIdx = -1;
    int frameIdx = 0;

    // post processing
    std::vector<std::string> footMarkerNames;
    std::vector<std::pair<std::string, std::string>> linkNames;
    std::vector<std::pair<std::string, std::string>> virtualLinkNames;
    crl::mocap::Timeline::TimelineData footSteps;
    double footVelocityThreshold = 0.8;
    double footHeightThreshold = 0.055;

    // plot and visualization
    bool followCharacter = true;
    bool showCoordinateFrames = false;
    bool showVirtualLimbs = true;
    bool showContactState = false;
    crl::mocap::PlotLine2D<crl::P3D> baseHeightPlot;
    crl::mocap::PlotLine2D<crl::dVector> baseSpeedPlot;
    crl::mocap::PlotLine2D<crl::dVector> feetHeightPlot;
    crl::mocap::PlotLine2D<crl::dVector> feetVelocityPlot;
    crl::mocap::PlotLine2D<crl::dVector> feetAccelerationPlot;
    crl::mocap::Timeline swingTimeline;

    int DBtotalFrame = 0;
    Eigen::MatrixXd DBMatching;
    Eigen::VectorXd Feature_mean;
    Eigen::VectorXd Feature_std;
    int DBfeatVecDim = 27;  // ?

    int currentMatchClip = -1;
    int currentMatchFrame = -1;
    int currentMatchIdx = 0;
    int nextMatchClip = -1;
    int nextMatchFrame = -1;
    int nextMatchIdx = 0;
    int currentContinousFramesPlayed = 0;
    int maxContFramePlayed = 5;
    std::vector<std::string> DBmarkerNames = {"LeftFoot", "RightFoot", "Hips"};
    Eigen::VectorXd accumulatedFrameNum;
    bool runMotionMatching;
    crl::mocap::Controller controller;
    Eigen::VectorXd currentQueryVector;
    bool firstInputGet = false;

    // int desiredFPS = 30;
    // double controllerOperateInterval;   // Time interval for controller take and process input
};
}  // namespace mocapApp