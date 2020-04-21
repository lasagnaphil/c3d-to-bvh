//
// Created by lasagnaphil on 19. 11. 12..
//

#ifndef GAIT_ANALYSIS_POINTMOTIONCLIP_H
#define GAIT_ANALYSIS_POINTMOTIONCLIP_H

#include "../filesystem.h"
#include "PoseFK.h"
#include "DebugRenderer.h"
#include "MotionClipData.h"
#include <fmt/core.h>
#include "Colors.h"

namespace ga {
    enum ViconMarkerLabels : uint32_t {
        SACR, LASI, RASI, LTHI, RTHI, LKNE, RKNE, LTIB, RTIB, LANK, RANK, LTOE, RTOE
    };

    enum MAMarkerLabels : uint32_t {
        MA_R_ASIS,
        MA_L_ASIS,
        MA_V_Sacral,
        MA_R_Thigh,
        MA_R_Knee,
        MA_R_Shank,
        MA_R_Ankle,
        MA_R_Heel,
        MA_R_Toe,
        MA_L_Thigh,
        MA_L_Knee,
        MA_L_Shank,
        MA_L_Ankle,
        MA_L_Heel,
        MA_L_Toe,
    };

    enum JointLabels : uint32_t {
        Pelvic, L_Hip, L_Knee, L_Ankle, L_Toe, R_Hip, R_Knee, R_Ankle, R_Toe
    };

    enum MarkerType : uint32_t {
        Vicon, MotionAnalysis
    };

    struct PointMotionClip {
        std::string name;
        MarkerType markerType;
        int point_cnt;
        int frame_cnt;
        float frame_dt;

        std::vector<glm::vec3> markerData;
        std::vector<glm::vec3> jointData;
        std::vector<glm::vec3> vmarkerData;

        int midPhaseIdx = -1;

        static PointMotionClip fromCSV(const std::string& file_in);

        static PointMotionClip fromC3D(const std::string& filename, MarkerType markerType, bool* valid = nullptr);

        nonstd::span<glm::vec3> getMarkerFrameData(uint32_t f) {
            return nonstd::span<glm::vec3>(markerData.data() + f*point_cnt, markerData.data() + (f+1)*point_cnt);
        }

        glm::vec3 getMarkerPos(uint32_t f, uint32_t p) {
            return markerData[f*point_cnt + p];
        }

        void setMarkerPos(uint32_t f, uint32_t p, glm::vec3 v) {
            markerData[f*point_cnt + p] = v;
        }

        glm::vec3 getJointPos(uint32_t f, uint32_t p) {
            assert(!jointData.empty());
            return jointData[f*9 + p];
        }

        void setJointPos(uint32_t f, uint32_t p, glm::vec3 v) {
            assert(!jointData.empty());
            jointData[f*9 + p] = v;
        }

        glm::vec3 getVMarkerPos(uint32_t f, uint32_t p) {
            assert(!vmarkerData.empty());
            return vmarkerData[f*4 + p];
        }

        void setVMarkerPos(uint32_t f, uint32_t p, glm::vec3 v) {
            assert(!vmarkerData.empty());
            vmarkerData[f*point_cnt + p] = v;
        }

        void rescale(float scale);

        void strip();

        void fillInvalidPoints();

        void smooth();

        std::vector<PointMotionClip> splitCycles(float dxMinThreshold = 0.005f, float dxMaxThreshold = 0.02f);

        bool isRefClip(float threshold=0.1f);

        void invertYZ();

        void move(glm::vec3 offset);

        void setRootToOrigin();

        void renderMarkers(DebugRenderer& imRenderer, uint32_t frameIdx);

        void renderJoints(DebugRenderer& imRenderer, uint32_t frameIdx);

        void renderVMarkers(DebugRenderer& imRenderer, uint32_t frameIdx);

        bool checkValidity();
    };

    MotionClipData convertToLowerBodyBVH(PointMotionClip& ref, PointMotionClip& clip);
}

#endif //GAIT_ANALYSIS_POINTMOTIONCLIP_H
