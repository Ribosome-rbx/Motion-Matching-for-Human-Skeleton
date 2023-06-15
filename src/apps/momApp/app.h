#pragma once

#include <crl-basic/gui/application.h>
#include <imgui_widgets/imfilebrowser.h>
#include <crl-basic/utils/logger.h>


#include "mom/MotionMatching.h"

namespace momApp {

class App : public crl::gui::ShadowApplication {
public:
    App() : crl::gui::ShadowApplication("MotionMatching App") {
        // load mann dataset
        std::string mocapPath = dataPath_ + "walk1_subject1.bvh";
        mocapSkeleton = new crl::mocap::MocapSkeleton(mocapPath.c_str());
        motionDatabase = new crl::mocap::MotionDatabase(dataPath_, false);
        motionJumpDatabase = new crl::mocap::MotionDatabase(dataJumpPath_, true);
        motionStopDatabase = new crl::mocap::MotionDatabase(dataStopPath_, false); 
        motionDanceDatabase = new crl::mocap::MotionDatabase(dataDancePath_, false); 
        motionMatching = new crl::mocap::MotionMatching(mocapSkeleton, motionDatabase, motionJumpDatabase, motionStopDatabase, motionDanceDatabase);
        motionMatching->queueSize = 60;
    }

    ~App() override {
        delete mocapSkeleton;
        delete motionDatabase;
        delete motionJumpDatabase;
        delete motionStopDatabase;
        delete motionDanceDatabase;
    }

    void process() override {
        motionMatching->paintTraj = hitPoints;
        static uint frame = 0;
        int max_frame = 30;
        if (motionMatching->KEY_J) max_frame += 20;
        // if (!motionMatching->isCartoonOn_)
        {
            if ((frame >= max_frame && !motionMatching->shouldStop_ && !motionMatching->isDance_) || NEW_INPUT || motionMatching->forceMotionMatching_) {   // || motionMatching->forceMotionMatching_
                crl::Logger::consolePrint("transition happens!");\
                motionMatching->forceMotionMatching_ = false;
                if (motionMatching->shouldStop_)
                {
                    motionMatching->shouldStop_ = false;
                    motionMatching->switchDatabase();
                }
                motionMatching->matchMotion(camera);
                frame = 0;
                NEW_INPUT = false;
                motionMatching->switchDatabase();
            }
        }
        

        motionMatching->advance();
        frame++;
        crl::P3D pos_(mocapSkeleton->root->state.pos.x, 0, mocapSkeleton->root->state.pos.z);
        crl::V3D vel_(crl::P3D(mocapSkeleton->root->state.velocity[0], 0, mocapSkeleton->root->state.velocity[2]));
        motionMatching->historyPos.push_back(pos_);
        motionMatching->historyVel.push_back(vel_.normalized());

        camera.target.x = mocapSkeleton->root->state.pos.x;
        camera.target.z = mocapSkeleton->root->state.pos.z;

        if (trackVelocity && !motionMatching->shouldStop_){
            motionMatching->cameraRotation(camera);
        }
        light.target.x() = mocapSkeleton->root->state.pos.x;
        light.target.z() = mocapSkeleton->root->state.pos.z;
    }

    void drawShadowCastingObjects(const crl::gui::Shader &shader) override {
        mocapSkeleton->draw(shader);
    }

    void drawObjectsWithoutShadows(const crl::gui::Shader &shader) override {
        mocapSkeleton->draw(shader);
        // motionMatching->drawDebugInfo(shader, camera);

        for (int i=0; i<hitPoints.size(); i++)
            crl::gui::drawSphere(hitPoints[i], 0.05, shader, crl::V3D(1, 0, 0), 0.5);

        // customized draw
        crl::P3D pos = mocapSkeleton->root->state.pos;
        crl::V3D vel = motionMatching->goalVel.normalized();
        if (!motionMatching->shouldStop_){
            drawSphere(pos + vel*0.5, 0.04, shader, crl::V3D(1, 0.55, 0), 0.5);
            drawArrow3d(pos + vel*0.5, vel*0.5, 0.02, shader, crl::V3D(1, 0.55, 0), 0.5);
        
            // drawArrow3d(mocapSkeleton->root->state.pos, motionMatching->cameraDir.normalized(), 0.02, shader, crl::V3D(0, 0.55, 1), 0.5);

            for (int i=0; i<motionMatching->PosTraj.getKnotCount(); i++){
                crl::P3D pos_i = crl::P3D() + motionMatching->PosTraj.evaluate_linear(i / 60.0);
                drawSphere(pos_i, 0.02, shader, crl::V3D(1, 0, 1), 0.5);
                if (i == motionMatching->PosTraj.getKnotCount() - 1){
                    crl::V3D vel_i = motionMatching->VelTraj.evaluate_linear(i / 60.0);
                    vel_i = vel_i.normalized();
                    drawArrow3d(pos_i, vel_i*0.15, 0.02, shader, crl::V3D(1, 0, 1), 0.5);
                }
            }
        }
        
        int start = motionMatching->historyPos.size() >= 120? motionMatching->historyPos.size()-120 : 0;
        for (int i=start; i<motionMatching->historyPos.size(); i++){
            crl::P3D pos_i = motionMatching->historyPos[i];
            drawSphere(pos_i, 0.02, shader, crl::V3D(0, 0.55, 1), 0.5);

            // if(i % 20 == 19){
            //     crl::V3D vel_i = (motionMatching->historyVel[i]).normalized();
            //     drawArrow3d(pos_i, vel_i, 0.02, shader, crl::V3D(0, 0, 1), 0.5);
            // }
        }
        pos[1] = 0;
        drawSphere(pos, 0.03, shader, crl::V3D(0, 1, 0.35), 0.5);
        if(!motionMatching->shouldStop_){
            crl::V3D character_vel = mocapSkeleton->root->state.velocity;
            character_vel[1] = 0;
            character_vel = character_vel.normalized();
            drawArrow3d(pos, character_vel*0.15, 0.02, shader, crl::V3D(0, 1, 0.35), 0.5);
        }
    }

    void drawImGui() override {
        crl::gui::ShadowApplication::drawImGui();

        ImGui::Begin("Main Menu");
        if (ImGui::CollapsingHeader("Motion Control options", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::InputDouble("Speed forward", &motionMatching->speedForward, 0.0f, 0.0f, "%.3f", ImGuiInputTextFlags_ReadOnly);
            ImGui::InputDouble("Speed turning", &motionMatching->turningSpeed, 0.0f, 0.0f, "%.3f", ImGuiInputTextFlags_ReadOnly);
            ImGui::Checkbox("Paint trajectory", &drawTrajMode);
            if (ImGui::Button("Clear Painting")) {
                hitPoints.clear();
            }
        }
        ImGui::End();
    }

    virtual bool keyPressed(int key, int mods) override {
        if (key == GLFW_KEY_SPACE) {
            processIsRunning = !processIsRunning;
        }
        if (key == GLFW_KEY_ENTER) {
            if (!processIsRunning)
                process();
        }
        if (key == GLFW_KEY_BACKSPACE) {
            screenIsRecording = !screenIsRecording;
        }
        if (key == GLFW_KEY_UP) {
            motionMatching->speedForward += 0.3;
        }
        if (key == GLFW_KEY_DOWN) {
            motionMatching->speedForward -= 0.3;
        }
        if (key == GLFW_KEY_LEFT) {
            motionMatching->turningSpeed += 0.3;
        }
        if (key == GLFW_KEY_RIGHT) {
            motionMatching->turningSpeed -= 0.3;
        }

        if (processIsRunning) {
            if (key == GLFW_KEY_P) {
                if (!motionMatching->KEY_P)
                {
                    NEW_INPUT = true;
                    motionMatching->isDance_ = !motionMatching->isDance_;
                    if (motionMatching->isDance_)
                        motionMatching->setPropertoDance();
                }
                motionMatching->KEY_P = true;
                motionMatching->switchDatabase();
            }
            if (!motionMatching->isDance_)
            {
                if (key == GLFW_KEY_W) {
                    if (motionMatching->speedForward < 0.01)
                        motionMatching->speedForward = 3.6;
                    if (!motionMatching->KEY_W) NEW_INPUT = true;
                    motionMatching->KEY_W = true;
                }
                if (key == GLFW_KEY_S) {
                    // motionMatching->speedForward = -3.0;
                    if (!motionMatching->KEY_S) NEW_INPUT = true;
                    motionMatching->KEY_S = true;
                }
                if (key == GLFW_KEY_A) {
                    // motionMatching->turningSpeed = 1.0;
                    if (!motionMatching->KEY_A) NEW_INPUT = true;
                    motionMatching->KEY_A = true;
                }
                if (key == GLFW_KEY_D) {
                    // motionMatching->turningSpeed = -1.0;
                    if (!motionMatching->KEY_D) NEW_INPUT = true;
                    motionMatching->KEY_D = true;
                }
                if (key == GLFW_KEY_J) {
                    // motionMatching->turningSpeed = -1.0;
                    if (!motionMatching->KEY_J) NEW_INPUT = true;
                    motionMatching->KEY_J = true;
                    motionMatching->switchDatabase();
                }
                if (key == GLFW_KEY_C) {
                    // motionMatching->turningSpeed = -1.0;
                    if (!motionMatching->KEY_C) NEW_INPUT = true;
                    motionMatching->KEY_C = true;
                    motionMatching->switchDatabase();
                }
                if (key == GLFW_KEY_LEFT_SHIFT) {
                    if (!motionMatching->KEY_J)
                        motionMatching->speedForward = 13.0;
                }
            }
        }
        return false;
    }

    virtual bool keyReleased(int key, int mods) override{
        if (processIsRunning) {
            if (key == GLFW_KEY_W) {
                if (motionMatching->KEY_W) NEW_INPUT = true;
                motionMatching->KEY_W = false;
            }
            if (key == GLFW_KEY_S) {
                if (motionMatching->KEY_S) NEW_INPUT = true;
                motionMatching->KEY_S = false;
            }
            if (key == GLFW_KEY_A) {
                if (motionMatching->KEY_A) NEW_INPUT = true;
                motionMatching->KEY_A = false;
            }
            if (key == GLFW_KEY_D) {
                if (motionMatching->KEY_D) NEW_INPUT = true;
                motionMatching->KEY_D = false;
            }
            if (key == GLFW_KEY_C) {
                // if (motionMatching->KEY_J) NEW_INPUT = true;
                motionMatching->KEY_C = false;
                motionMatching->switchDatabase();
            }
            if (key == GLFW_KEY_J) {
                // if (motionMatching->KEY_J) NEW_INPUT = true;
                motionMatching->KEY_J = false;
                motionMatching->switchDatabase();
            }
            if (key == GLFW_KEY_LEFT_SHIFT) {
                if (!motionMatching->KEY_J)
                    motionMatching->speedForward = 3.6;
            }
            if (key == GLFW_KEY_P) {
                motionMatching->KEY_P = false;
            }
        }
        return false;
    }

    bool mouseMove(double xpos, double ypos) override {
        if (drawTrajMode && mouseState.dragging) {
            crl::P3D rayOrigin;
            crl::V3D rayDirection;
            camera.getRayFromScreenCoordinates(xpos, ypos, rayOrigin, rayDirection);

            crl::P3D hit = crl::P3D(rayOrigin[0] - rayOrigin[1]/rayDirection[1] * rayDirection[0], 0, rayOrigin[2] - rayOrigin[1]/rayDirection[1] * rayDirection[2]);
            hitPoints.push_back(hit);
            return true;
        }

        return crl::gui::ShadowApplication::mouseMove(xpos, ypos);
    }

    bool mouseButtonPressed(int button, int mods) override {
        return true;
    }

    bool drop(int count, const char **fileNames) override {
        return true;
    }

public:
    std::string dataPath_ = MOTION_MATCHING_DEMO_DATA_FOLDER "/mocap/lafan1_mini/";
    std::string dataJumpPath_ = MOTION_MATCHING_DEMO_DATA_FOLDER "/mocap/lafan1_jump/";
    std::string dataStopPath_ = MOTION_MATCHING_DEMO_DATA_FOLDER "/mocap/lafan1_stop/";
    std::string dataDancePath_ = MOTION_MATCHING_DEMO_DATA_FOLDER "/mocap/lafan1_dance/";
    crl::mocap::MocapSkeleton *mocapSkeleton = nullptr;
    crl::mocap::MotionDatabase *motionDatabase = nullptr;
    crl::mocap::MotionDatabase *motionJumpDatabase = nullptr;
    crl::mocap::MotionDatabase *motionStopDatabase = nullptr;
    crl::mocap::MotionDatabase *motionDanceDatabase = nullptr;
    crl::mocap::MotionMatching *motionMatching = nullptr;
    std::vector<crl::P3D> hitPoints;
private:
    bool NEW_INPUT = false;
    bool drawTrajMode = false;
};

}  // namespace momApp