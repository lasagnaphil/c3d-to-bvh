//
// Created by lasagnaphil on 19. 12. 31..
//

#define _DEBUG

#include <iostream>
#include <random>
#include "filesystem.h"

#include "imguix/imgui_plot.h"

#include <InputManager.h>
#include <MotionClipData.h>
#include <MotionClipPlayer.h>
#include <PoseRenderBody.h>

#include "App.h"
#include "PhongRenderer.h"
#include "GizmosRenderer.h"
#include "TrackballCamera.h"
#include "c3d/PointMotionClip.h"
#include "c3d/C3DMotionClipPlayer.h"

namespace fs = std::filesystem;

enum class FileType {
    BVH, C3D
};

class ViewerApp : public App {
public:
    ViewerApp(int argc, char** argv) : App(false) {
        if (argc != 2) {
            fmt::print("Filename: ");
            std::cin >> filename;
        }
        else {
            filename = argv[1];
        }
    }

    void loadResources() override {
        auto path = fs::path(filename);
        if (!fs::exists(path)) {
            fmt::print(stderr, "File {} does not exist!", path.string());
            exit(EXIT_FAILURE);
        }
        auto ext = path.extension().string();
        if (ext == ".bvh") {
            fileType = FileType::BVH;
            bvh = MotionClipData::loadFromFile(filename);
            motionClipPlayer = std::make_unique<BVHMotionClipPlayer>(&bvh);

            Ref<PhongMaterial> bodyMat = Resources::make<PhongMaterial>();
            bodyMat->ambient = {0.1f, 0.1f, 0.1f, 1.0f};
            bodyMat->specular = {0.7f, 0.7f, 0.7f, 1.0f};
            bodyMat->diffuse = {1.0f, 0.0f, 0.0f, 1.0f};
            bodyMat->shininess = 64.0f;
            bodyMat->texDiffuse = {};
            bodyMat->texSpecular = {};

            poseRenderBody = PoseRenderBody::createAsBoxes(bvh.poseTree, 0.05f, bodyMat);

            fmt::print("frameTime = {}\n", bvh.frameTime);
        }
        else if (ext == ".c3d" || ext == ".C3D") {
            fileType = FileType::C3D;
            c3d = ga::PointMotionClip::fromC3D(filename, ga::MarkerType::Vicon);
            c3d.rescale(0.001f);
            c3d.invertYZ();
            motionClipPlayer = std::make_unique<ga::C3DMotionClipPlayer>(&c3d);
        }

        // TrackballCamera* trackballCamera = initCamera<TrackballCamera>();
        FlyCamera* flyCamera = dynamic_cast<FlyCamera*>(this->camera.get());
        flyCamera->movementSpeed = 2.0f;
        // flyCamera->far = 100000.0f;

        Ref<Transform> cameraTransform = flyCamera->transform;
        cameraTransform->move({0.0f, 1.0f, 0.0f});
    }

    void processInput(SDL_Event& event) override {
    }

    void update(float dt) override {
        auto inputMgr = InputManager::get();

        if (!modifyJoints) {
            motionClipPlayer->update(dt);
        }

        if (fixCamera) {
            FlyCamera* flyCamera = dynamic_cast<FlyCamera*>(camera.get());
            glm::vec3 cameraPos;
            if (fileType == FileType::BVH) {
                cameraPos = dynamic_cast<BVHMotionClipPlayer*>(motionClipPlayer.get())->getPoseState().v;
            }
            else if (fileType == FileType::C3D) {
                cameraPos = dynamic_cast<ga::C3DMotionClipPlayer*>(motionClipPlayer.get())->getCurrentFrame()[0];
            }
            cameraPos.z += 1.0f;
            cameraPos.y += 0.0f;
            flyCamera->transform->setPosition(cameraPos);
        }
    }

    void render() override {
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

        if (fileType == FileType::BVH) {
            auto& poseState = dynamic_cast<BVHMotionClipPlayer*>(motionClipPlayer.get())->getPoseState();
            renderMotionClip(phongRenderer, imRenderer, poseState, bvh.poseTree, poseRenderBody,
                             glm::mat4(1.0f), false);
        }
        phongRenderer.render();

        if (fileType == FileType::C3D) {
            c3d.renderMarkers(imRenderer, motionClipPlayer->getFrameIdx());
        }
        imRenderer.drawAxisTriad(glm::mat4(1.0f), 0.1f, 1.0f, true);
        imRenderer.drawXZSquareGrid(-10.0f, 10.0f, 0.0f, 1.0f, colors::White, true);
        imRenderer.render();

        gizmosRenderer.render();

        drawImGui();
        motionClipPlayer->renderImGui();
    }

    void release() override {
    }

    void drawImGui() {
        ImGui::Begin("Motion Clip");
        ImGui::Checkbox("Fix Camera", &fixCamera);
        ImGui::End();
    }

private:
    MotionClipData bvh;
    std::unique_ptr<MotionClipPlayer> motionClipPlayer;

    ga::PointMotionClip c3d;

    bool modifyJoints = false;

    bool fixCamera = true;

    PoseRenderBody poseRenderBody;

    std::string filename;
    FileType fileType;
};

int main(int argc, char** argv) {
    ViewerApp app(argc, argv);
    app.load();
    app.startMainLoop();
    app.release();

    return 0;
}