//
// Created by lasagnaphil on 20. 1. 2..
//

#ifndef GAIT_ANALYSIS_C3DMOTIONCLIPPLAYER_H
#define GAIT_ANALYSIS_C3DMOTIONCLIPPLAYER_H

#include "PointMotionClip.h"

namespace ga {

struct C3DMotionClipPlayer : public MotionClipPlayer {
    PointMotionClip* clip;

    C3DMotionClipPlayer(PointMotionClip* clip = nullptr) : MotionClipPlayer(), clip(clip) {}

    void setData(PointMotionClip* clip) {
        this->clip = clip;
    }

    uint32_t getNumFrames() override {
        return clip->frame_cnt;
    }

    float getFrameTime() override {
        return clip->frame_dt;
    }

    std::string getName() override {
        assert(clip);
        return "C3D Player";
    }

    nonstd::span<glm::vec3> getCurrentFrame() {
        return clip->getMarkerFrameData(currentFrameIdx);
    }
};

}
#endif //GAIT_ANALYSIS_C3DMOTIONCLIPPLAYER_H
