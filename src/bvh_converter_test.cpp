//
// Created by lasagnaphil on 19. 11. 11..
//

#define _DEBUG

#include <iostream>
#include <random>

#include "imguix/imgui_plot.h"

#include <InputManager.h>
#include <MotionClipData.h>
#include <MotionClipPlayer.h>
#include <PoseRenderBody.h>
#include <glm/gtx/string_cast.hpp>

#include "App.h"
#include "PhongRenderer.h"
#include "GizmosRenderer.h"
#include "TrackballCamera.h"
#include "c3d/PointMotionClip.h"

class ViewerApp : public App {
public:
    ViewerApp(int argc, char** argv) : App(false) {
        if (argc != 3 && argc != 4 && argc != 5) {
            fmt::print("Usage: {} <type> <clip> <ref>\n  <type>: vicon or ma\n", argv[0]);
            exit(EXIT_FAILURE);
        }
        else {
            if (strcmp(argv[1], "vicon") == 0)
                markerType = ga::MarkerType::Vicon;
            else if (strcmp(argv[1], "ma") == 0)
                markerType = ga::MarkerType::MotionAnalysis;
            pointMotionClipName = argv[2];
            if (argc >= 4) {
                refMotionClipName = argv[3];
            }
            else {
                refMotionClipName = pointMotionClipName;
            }
            if (argc >= 5) {
                outputClipName = argv[4];
            }
        }
    }

    void loadResources() override {
        pointMotionClip = ga::PointMotionClip::fromC3D(pointMotionClipName, markerType);
        pointMotionClip.rescale(0.001f);

        refMotionClip = ga::PointMotionClip::fromC3D(refMotionClipName, markerType);
        refMotionClip.rescale(0.001f);

        bvh = ga::convertToLowerBodyBVH(refMotionClip, pointMotionClip);
        if (!outputClipName.empty()) {
            bvh.saveToFile(outputClipName, EulOrdZYXs);
        }

        refMotionClip.invertYZ();
        pointMotionClip.invertYZ();

        motionClipPlayer = BVHMotionClipPlayer(&bvh);

        Ref<PhongMaterial> bodyMat = Resources::make<PhongMaterial>();
        bodyMat->ambient = {0.1f, 0.1f, 0.1f, 1.0f};
        bodyMat->specular = {0.7f, 0.7f, 0.7f, 1.0f};
        bodyMat->diffuse = {1.0f, 0.0f, 0.0f, 1.0f};
        bodyMat->shininess = 64.0f;
        bodyMat->texDiffuse = {};
        bodyMat->texSpecular = {};

        poseRenderBody = PoseRenderBody::createAsBoxes(bvh.poseTree, 0.05f, bodyMat);

        // TrackballCamera* trackballCamera = initCamera<TrackballCamera>();
        FlyCamera* flyCamera = dynamic_cast<FlyCamera*>(this->camera.get());
        flyCamera->movementSpeed = 2.0f;
        // flyCamera->far = 100000.0f;

        Ref<Transform> cameraTransform = flyCamera->transform;
        cameraTransform->move({0.0f, 1.0f, 0.0f});

        // cameraTransform->rotate(M_PI/4, {0.0f, 1.0f, 0.0f});
    }

    void processInput(SDL_Event& event) override {
    }

    void update(float dt) override {
        auto inputMgr = InputManager::get();

        if (!modifyJoints) {
            motionClipPlayer.update(dt);
        }

        if (fixCamera) {
            FlyCamera* flyCamera = dynamic_cast<FlyCamera*>(camera.get());
            glm::vec3 cameraPos = motionClipPlayer.getPoseState().v;
            cameraPos.z += 1.0f;
            cameraPos.y += 0.0f;
            flyCamera->transform->setPosition(cameraPos);
        }
    }

    void render() override {
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

        if (renderBody)
            renderMotionClip(phongRenderer, imRenderer, motionClipPlayer.getPoseState(), bvh.poseTree, poseRenderBody,
                    glm::mat4(1.0f), false);
        phongRenderer.render();

        if (renderJoints)
            pointMotionClip.renderJoints(imRenderer, motionClipPlayer.getFrameIdx());
        if (renderMarkers)
            pointMotionClip.renderMarkers(imRenderer, motionClipPlayer.getFrameIdx());

        refMotionClip.renderMarkers(imRenderer, refMotionClipIdx);

        imRenderer.drawAxisTriad(glm::mat4(1.0f), 0.1f, 1.0f, true);
        imRenderer.drawXZSquareGrid(-10.0f, 10.0f, 0.0f, 1.0f, colors::White, true);
        imRenderer.render();

        gizmosRenderer.render();

        drawImGui();
        motionClipPlayer.renderImGui();
    }

    void release() override {
    }

    void drawImGui() {
        ImGui::Begin("Motion Clip");
        ImGui::Checkbox("Fix Camera", &fixCamera);
        ImGui::Checkbox("Render Markers", &renderMarkers);
        ImGui::Checkbox("Render Joints", &renderJoints);
        ImGui::Checkbox("Render Body", &renderBody);

        ImGui::SliderInt("Ref frame idx", &refMotionClipIdx, 0, refMotionClip.frame_cnt-1);

        ImGui::End();
    }

private:
    ga::PointMotionClip pointMotionClip;
    ga::PointMotionClip refMotionClip;
    MotionClipData bvh;
    BVHMotionClipPlayer motionClipPlayer;
    int refMotionClipIdx = 0;

    bool renderMarkers = true;
    bool renderJoints = true;
    bool renderBody = true;
    bool modifyJoints = false;

    bool fixCamera = true;

    PoseRenderBody poseRenderBody;

    std::string pointMotionClipName;
    std::string refMotionClipName;
    std::string outputClipName;
    ga::MarkerType markerType;
};

int main(int argc, char** argv) {
    ViewerApp app(argc, argv);
    app.load();
    app.startMainLoop();
    app.release();

    return 0;
}
