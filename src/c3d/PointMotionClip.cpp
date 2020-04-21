//
// Created by lasagnaphil on 19. 12. 24..
//

#include "PointMotionClip.h"
#include "c3dfile.h"

using glm::vec3;
using glm::quat;

namespace ga {

static std::vector<std::string> viconMarkerLabels = {
        "SACR",
        "LASI",
        "RASI",
        "LTHI",
        "RTHI",
        "LKNE",
        "RKNE",
        "LTIB",
        "RTIB",
        "LANK",
        "RANK",
        "LTOE",
        "RTOE"
};

static std::vector<std::string> maMarkerLabels = {
        "R.ASIS",
        "L.ASIS",
        "V.Sacral",
        "R.Thigh",
        "R.Knee",
        "R.Shank",
        "R.Ankle",
        "R.Heel",
        "R.Toe",
        "L.Thigh",
        "L.Knee",
        "L.Shank",
        "L.Ankle",
        "L.Heel",
        "L.Toe"
};

std::tuple<vec3, quat> jointTransFromCoronalPlane(vec3 v0, vec3 v1, vec3 v2, float depth, bool rightSide) {
    vec3 n = normalize(cross(v2 - v0, v1 - v0));
    vec3 t1 = normalize(v2 - v0);
    vec3 t2 = normalize(cross(v2 - v0, n));
    vec3 r = (v0 + v2)/2;
    float R = length(v2 - v0) / 2;
    float theta = 2 * asin(depth / (2*R));
    vec3 k = r + R*cos(theta)*t1 + R*sin(theta)*t2;

    vec3 eZ = normalize(v0 - k);
    vec3 beta1 = v1 - v0;
    vec3 beta2 = beta1 - dot(beta1, eZ) * eZ;
    vec3 eY = (rightSide? -1 : 1) * normalize(beta2);
    vec3 eX = cross(eY, eZ);
    quat kq = quat_cast(glm::mat3(eX, eY, eZ));

    return {k, kq};
}

std::tuple<vec3, quat> jointTransFromSagittalPlane(vec3 v0, vec3 v1, vec3 v2, float depth, bool rightSide) {
    vec3 r12 = (v1 + v2)/2;
    float R12 = length(v2 - v1)/2;
    vec3 r20 = (v2 + v0)/2;
    float R20 = length(v0 - v2)/2;
    float R01 = length(v1 - v0)/2;
    float alpha = (R01*R01 - R20*R20 + R12*R12) / (2*R01*R01);
    vec3 r = r12 + alpha * (r20 - r12);
    vec3 t1 = normalize(v2 - r);
    vec3 t2 = normalize(cross(t1, v1 - v0));
    float R = sqrt(R12*R12 - (alpha*R01)*(alpha*R01));
    float theta = 2 * asin(depth / (2*R));
    vec3 k = r + R*cos(theta)*t1 + R*sin(theta)*t2;

    fmt::print("theta = {}, R = {}\n", theta, R);

    vec3 eZ = normalize(v0 - k);
    vec3 eY = (rightSide? -1 : 1) * normalize(v2 - k);
    vec3 eX = cross(eY, eZ);
    quat kq = quat_cast(glm::mat3(eX, eY, eZ));

    return {k, kq};
}

PointMotionClip PointMotionClip::fromCSV(const std::string& file_in) {
    std::ifstream infile(file_in);
    std::string line;

    PointMotionClip pclip;

    std::getline(infile, line);
    pclip.name = line.substr(line.find(',') + 1);
    std::getline(infile, line);
    pclip.point_cnt = std::stoi(line.substr(line.find(',') + 1));
    std::getline(infile, line);
    pclip.frame_cnt = std::stoi(line.substr(line.find(',') + 1));
    std::getline(infile, line);
    pclip.frame_dt = std::stof(line.substr(line.find(',') + 1));

    std::vector<float> data;
    pclip.markerData.reserve(pclip.frame_cnt * pclip.point_cnt * 3);

    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        std::string item;
        glm::vec3 v;
        int counter = 0;
        while (std::getline(iss, item, ',')) {
            v[counter] = std::stof(item);
            counter++;
            if (counter == 3) {
                pclip.markerData.push_back(v);
                counter = 0;
            }
        }
    }
    return pclip;
}

PointMotionClip PointMotionClip::fromC3D(const std::string& filename, MarkerType markerType, bool* valid) {
    PointMotionClip pclip;
    pclip.markerType = markerType;

    C3DFile c3d;
    if (!c3d.load(filename.c_str())) {
        fmt::print(stderr, "Error while loading file {}", filename);
        if (valid) {
            *valid = false;
            return pclip;
        }
        else {
            exit(EXIT_FAILURE);
        }
    }
    if (valid) { *valid = true; }

    pclip.name = std::filesystem::path(filename).stem();
    float fps = c3d.getParamFloat("POINT:RATE");
    if (fps == 0 || glm::isinf(fps) || glm::isnan(fps)) {
        pclip.frame_dt = 1.0f / 60.0f;
    }
    else {
        pclip.frame_dt = 1.0f / fps;
    }
    if (markerType == MarkerType::Vicon)
        pclip.point_cnt = viconMarkerLabels.size();
    else
        pclip.point_cnt = maMarkerLabels.size();
    pclip.frame_cnt = c3d.float_point_data[0].x.size();

    pclip.markerData.resize(pclip.frame_cnt * pclip.point_cnt);

    for (int p = 0; p < pclip.point_cnt; p++) {
        std::string label;
        if (markerType == MarkerType::Vicon) label = viconMarkerLabels[p];
        else label = maMarkerLabels[p];

        const FloatMarkerData& data = c3d.getMarkerTrajectories(label.c_str());
        for (int f = 0; f < pclip.frame_cnt; f++) {
            pclip.markerData[f*pclip.point_cnt + p + 0] = {data.x[f], data.y[f], data.z[f]};
        }
    }

    return pclip;
}

void PointMotionClip::rescale(float scale) {
    int rowLen = 3*point_cnt;
    for (int f = 0; f < frame_cnt; f++) {
        for (int p = 0; p < point_cnt; p++) {
            markerData[f*point_cnt + p] *= scale;
        }
    }
}

void PointMotionClip::strip() {
    int maxZeroLeft, maxZeroRight;
    do {
        maxZeroLeft = 0, maxZeroRight = 0;
        for (int p = 0; p < point_cnt; p++) {
            int zeroCount = 0;
            for (int f = 0; f < frame_cnt; f++) {
                if (getMarkerPos(f, p).x == 0) zeroCount++;
                else break;
            }
            if (zeroCount > maxZeroLeft) maxZeroLeft = zeroCount;
        }
        for (int p = 0; p < point_cnt; p++) {
            int zeroCount = 0;
            for (int f = frame_cnt - 1; f >= 0; f--) {
                if (getMarkerPos(f, p).x == 0) zeroCount++;
                else break;
            }
            if (zeroCount > maxZeroRight) maxZeroRight = zeroCount;
        }

        if (maxZeroLeft > 0)
            markerData.erase(markerData.begin(), markerData.begin() + maxZeroLeft * point_cnt);
        if (maxZeroRight > 0)
            markerData.erase(markerData.end() - maxZeroRight * point_cnt, markerData.end());
        frame_cnt -= (maxZeroLeft + maxZeroRight);

    } while (maxZeroLeft > 0 || maxZeroRight > 0);
}

void PointMotionClip::fillInvalidPoints() {
    for (int p = 0; p < point_cnt; p++) {
        std::vector<std::pair<int, int>> zeroRanges;

        int zeroRangeStart = -1;
        for (int f = 0; f < frame_cnt; f++) {
            glm::vec3 v = getMarkerPos(f, p);
            if (v.x == 0 && zeroRangeStart == -1) {
                zeroRangeStart = f;
            }
            else if (v.x != 0 && zeroRangeStart != -1) {
                int zeroRangeEnd = f-1;
                zeroRanges.push_back({zeroRangeStart, zeroRangeEnd});
                zeroRangeStart = -1;
            }
        }
        if (zeroRanges.size() == 0) continue;

        for (int i = 0; i < zeroRanges.size(); i++) {
            int f0 = zeroRanges[i].first - 2;
            if (i > 0 && f0 == zeroRanges[i-1].second) {
                f0 = zeroRanges[i-1].first - 1;
            }
            int f1 = zeroRanges[i].first - 1;
            int f2 = zeroRanges[i].second + 1;
            int f3 = zeroRanges[i].second + 2;
            if (i < zeroRanges.size() - 1 && f3 == zeroRanges[i+1].first) {
                f3 = zeroRanges[i+1].second + 1;
            }
            glm::vec3 p0 = getMarkerPos(f0 >= 0? f0 : 0, p);
            glm::vec3 p1 = getMarkerPos(f1, p);
            glm::vec3 p2 = getMarkerPos(f2, p);
            glm::vec3 p3 = getMarkerPos(f3 < frame_cnt? f3 : frame_cnt-1, p);
            for (int f = f1+1; f <= f2-1; f++) {
                glm::vec3 a1 = p0 * (f1 - f) / (f1 - f0) + p1 * (f - f0) / (f1 - f0);
                glm::vec3 a2 = p1 * (f2 - f) / (f2 - f1) + p2 * (f - f1) / (f2 - f1);
                glm::vec3 a3 = p2 * (f3 - f) / (f3 - f2) + p3 * (f - f2) / (f3 - f2);
                glm::vec3 b1 = a1 * (f2 - f) / (f2 - f0) + a2 * (f - f0) / (f2 - f0);
                glm::vec3 b2 = a2 * (f3 - f) / (f3 - f1) + a3 * (f - f1) / (f3 - f1);
                glm::vec3 c = b1 * (f2 - f) / (f2 - f1) + b2 * (f - f1) / (f2 - f1);
                setMarkerPos(f, p, c);
            }
        }
    }
}

void PointMotionClip::smooth() {
    for (int p = 0; p < point_cnt; p++) {
        for (int f = 2; f < frame_cnt-2; f++) {
            glm::vec3 v0 = getMarkerPos(f-2, p);
            glm::vec3 v1 = getMarkerPos(f-1, p);
            glm::vec3 v2 = getMarkerPos(f  , p);
            glm::vec3 v3 = getMarkerPos(f+1, p);
            glm::vec3 v4 = getMarkerPos(f+2, p);
            setMarkerPos(f, p, (v0 + 4*v1 + 6*v2 + 4*v3 + v4)/16);
        }
    }
}

std::vector<PointMotionClip> PointMotionClip::splitCycles(float dxMinThreshold, float dxMaxThreshold) {
    assert(jointData.empty());
    assert(vmarkerData.empty());

    int ltoeIdx, rtoeIdx;
    if (markerType == MarkerType::Vicon) ltoeIdx = LTOE, rtoeIdx = RTOE;
    else if (markerType == MarkerType::MotionAnalysis) ltoeIdx = MA_L_Toe, rtoeIdx = MA_R_Toe;
    else return {};

    bool curMarkedR = false, curMarkedL = false;
    std::vector<int> markedR, markedL;
    for (int f = 0; f < frame_cnt-1; f++) {
        float rToeDx = glm::length(getMarkerPos(f+1, rtoeIdx) - getMarkerPos(f, rtoeIdx));
        float rToeHeight = getMarkerPos(f, rtoeIdx).z;
        float lToeDx = glm::length(getMarkerPos(f+1, ltoeIdx) - getMarkerPos(f, ltoeIdx));
        float lToeHeight = getMarkerPos(f, ltoeIdx).z;

        if (!curMarkedR && rToeDx <= dxMinThreshold && rToeHeight <= 0.15f) {
            float dx_lr_diff = getMarkerPos(f, rtoeIdx).x - getMarkerPos(f, ltoeIdx).x;
            if (dx_lr_diff > 0) {
                markedR.push_back(f);
                curMarkedR = true;
            }
        }
        if (curMarkedR && rToeDx >= dxMaxThreshold) {
            curMarkedR = false;
        }

        if (!curMarkedL && lToeDx <= dxMinThreshold && lToeHeight <= 0.15f) {
            float dx_lr_diff = getMarkerPos(f, ltoeIdx).x - getMarkerPos(f, rtoeIdx).x;
            if (dx_lr_diff > 0) {
                markedL.push_back(f);
                curMarkedL = true;
            }
        }
        if (curMarkedL && lToeDx >= dxMaxThreshold) {
            curMarkedL = false;
        }
    }

    std::vector<PointMotionClip> clips;
    if (markedR.size() >= 2) {
        for (int i = 0; i < markedR.size() - 1; i++) {
            int f1 = markedR[i], f2 = markedR[i+1];
            /*
            {
                int f1_adj = f1, f2_adj = f2;
                float dx_min = 10000.0f;
                for (int f = f1; f < std::min(f1 + 10, frame_cnt-1); f++) {
                    float dx = getMarkerPos(f + 1, rtoeIdx).x - getMarkerPos(f, rtoeIdx).x;
                    if (dx_min > dx) {
                        dx_min = dx;
                        f1_adj = f;
                    }
                }
                dx_min = 10000.0f;
                for (int f = f2; f < std::min(f2 + 10, frame_cnt-1); f++) {
                    float dx = glm::length(getMarkerPos(f + 1, rtoeIdx) - getMarkerPos(f, rtoeIdx));
                    if (dx_min > dx) {
                        dx_min = dx;
                        f2_adj = f;
                    }
                }
                fmt::print("adjustment: {} {}\n", f1_adj-f1, f2_adj-f2);
                f1 = f1_adj; f2 = f2_adj;
            }
             */
            if (f2 - f1 > 10) {
                PointMotionClip clip;
                clip.name = fmt::format("{}_{}", name, i);
                clip.markerType = markerType;
                clip.point_cnt = point_cnt;
                clip.frame_cnt = f2 - f1;
                clip.frame_dt = frame_dt;
                clip.markerData.resize(clip.frame_cnt * clip.point_cnt);
                std::copy(markerData.data() + f1*point_cnt, markerData.data() + f2*point_cnt, clip.markerData.data());

                for (int lIdx : markedL) {
                    if (f1 < lIdx && lIdx < f2) {
                        clip.midPhaseIdx = lIdx - f1;
                        fmt::print("MidPhase found in clip {}: {}\n", clip.name, clip.midPhaseIdx);
                        break;
                    }
                }
                if (clip.midPhaseIdx == -1) {
                    fmt::print("MidPhase not found in clip {}!!!\n", clip.name);
                }

                clips.push_back(clip);
            }
        }
    }

    return clips;
}

bool PointMotionClip::isRefClip(float threshold) {
    if (frame_cnt < 20) return true;

    uint32_t ltoe;
    if (markerType == MarkerType::Vicon) ltoe = LTOE;
    else if (markerType == MarkerType::MotionAnalysis) ltoe = MA_L_Toe;

    float xMin = FLT_MAX, xMax = FLT_MIN;
    for (int f = 0; f < frame_cnt; f++) {
        float x = getMarkerPos(f, ltoe).x;
        if (x != 0) {
            if (xMin > x) xMin = x;
            if (xMax < x) xMax = x;
        }
    }
    return xMax - xMin < threshold;
}

void PointMotionClip::invertYZ() {
    for (int f = 0; f < frame_cnt; f++) {
        for (int p = 0; p < point_cnt; p++) {
            markerData[f*point_cnt + p] = glmx::Rx(-M_PI/2) * markerData[f*point_cnt + p];
        }
        if (!jointData.empty()) {
            for (int p = 0; p < 9; p++) {
                jointData[f*9 + p] = glmx::Rx(-M_PI/2) * jointData[f*9 + p];
            }
        }
    }
}

void PointMotionClip::move(glm::vec3 offset) {
    int rowLen = 3*point_cnt;
    for (int f = 0; f < frame_cnt; f++) {
        for (int p = 0; p < point_cnt; p++) {
            markerData[f*point_cnt + p] += offset;
        }
    }
}

void PointMotionClip::setRootToOrigin() {
    int rootIdx;
    if (markerType == MarkerType::Vicon) rootIdx = SACR;
    else if (markerType == MarkerType::MotionAnalysis) rootIdx = MA_V_Sacral;
    glm::vec3 offset = -getMarkerPos(0, rootIdx);
    offset.z = 0;
    move(offset);
}

void PointMotionClip::renderMarkers(DebugRenderer& imRenderer, uint32_t frameIdx) {
    for (int p = 0; p < point_cnt; p++) {
        glm::vec3 pos = getMarkerPos(frameIdx, p);
        imRenderer.drawPoint(pos, colors::Blue, 6.0f, false);
    }
}

void PointMotionClip::renderJoints(DebugRenderer& imRenderer, uint32_t frameIdx) {
    for (int p = 0; p < 9; p++) {
        glm::vec3 pos = getJointPos(frameIdx, p);
        imRenderer.drawPoint(pos, colors::Green, 6.0f, false);
    }
    imRenderer.drawLine(getJointPos(frameIdx, Pelvic), getJointPos(frameIdx, L_Hip), colors::Green, false);
    imRenderer.drawLine(getJointPos(frameIdx, L_Hip), getJointPos(frameIdx, L_Knee), colors::Green, false);
    imRenderer.drawLine(getJointPos(frameIdx, L_Knee), getJointPos(frameIdx, L_Ankle), colors::Green, false);
    imRenderer.drawLine(getJointPos(frameIdx, L_Ankle), getJointPos(frameIdx, L_Toe), colors::Green, false);
    imRenderer.drawLine(getJointPos(frameIdx, Pelvic), getJointPos(frameIdx, R_Hip), colors::Green, false);
    imRenderer.drawLine(getJointPos(frameIdx, R_Hip), getJointPos(frameIdx, R_Knee), colors::Green, false);
    imRenderer.drawLine(getJointPos(frameIdx, R_Knee), getJointPos(frameIdx, R_Ankle), colors::Green, false);
    imRenderer.drawLine(getJointPos(frameIdx, R_Ankle), getJointPos(frameIdx, R_Toe), colors::Green, false);
}

void PointMotionClip::renderVMarkers(DebugRenderer& imRenderer, uint32_t frameIdx) {
    for (int p = 0; p < 4; p++) {
        glm::vec3 pos = getVMarkerPos(frameIdx, p);
        imRenderer.drawPoint(pos, colors::White, 6.0f, false);
    }
}

bool PointMotionClip::checkValidity() {
    int validFrames = 0;
    const float eps = glm::epsilon<float>();
    for (int f = 0; f < frame_cnt; f++) {
        int p;
        for (p = 0; p < point_cnt; p++) {
            glm::vec3 pos = getMarkerPos(f, p);
            if (std::isnan(pos.x) || std::isnan(pos.y) | std::isnan(pos.z)) {
                break;
            }
            if (glm::epsilonEqual(pos.x, 0.f, eps) &&
                glm::epsilonEqual(pos.y, 0.f, eps) &&
                glm::epsilonEqual(pos.z, 0.f, eps)) {
                break;
            }
        }
        if (p == point_cnt) {
            validFrames++;
        }
    }
    return validFrames > 0;
}

std::tuple<glm::vec3, glm::vec3> calcHipJointOffset(float interAsis, float leftLegLength, float rightLegLength) {
    float meanLegLength = (leftLegLength + rightLegLength)/2;

    float leftAsisTrocDist = 0.1288f * leftLegLength - 0.04856f;
    float rightAsisTrocDist = 0.1288f * rightLegLength - 0.04856f;

    float C = meanLegLength * 0.115f - 0.0153f;

    float theta = 0.5f;
    float beta = 0.314f;
    float markerRadius = 0.01f;

    vec3 lHipJointOffset;
    lHipJointOffset.x = C*cosf(theta)*sinf(beta) - (leftAsisTrocDist + markerRadius) * cosf(beta);
    lHipJointOffset.y = -(C*sinf(theta) - interAsis/2);
    lHipJointOffset.z = -C*cosf(theta)*cosf(beta) - (leftAsisTrocDist+ markerRadius) * sinf(beta);

    vec3 rHipJointOffset;
    rHipJointOffset.x = C*cosf(theta)*sinf(beta) - (rightAsisTrocDist + markerRadius) * cosf(beta);
    rHipJointOffset.y = C*sinf(theta) - interAsis/2;
    rHipJointOffset.z = -C*cosf(theta)*cosf(beta) - (rightAsisTrocDist + markerRadius) * sinf(beta);

    return {lHipJointOffset, rHipJointOffset};
}

std::tuple<glm::vec3, glm::vec3> calcHipJointOffset(PointMotionClip& ref) {
    float interAsis = 0;
    float leftLegLength = 0;
    float rightLegLength = 0;

    uint32_t lasi, rasi, lkne, rkne, lank, rank;
    if (ref.markerType == MarkerType::Vicon) {
        lasi = LASI; rasi = RASI; lkne = LKNE; rkne = RKNE; lank = LANK; rank = RANK;
    }
    else if (ref.markerType == MarkerType::MotionAnalysis) {
        lasi = MA_L_ASIS; rasi = MA_R_ASIS; lkne = MA_L_Knee, rkne = MA_R_Knee, lank = MA_L_Ankle, rank = MA_R_Ankle;
    }
    for (int f = 0; f < ref.frame_cnt; f++) {
        auto frameData = ref.getMarkerFrameData(f);

        interAsis += glm::length(frameData[lasi] - frameData[rasi]);

        /*
          Note: The leg length is formally defined as the dist between the ASIS and the medial malleolus
          However, there are some problems:
          1. LMED and RMED is not known, so we need to apoproximate it as LANK and RANK instead
          2. There is a possibility that the patient is not standing up straight for the reference pose
           and in a crouch pose instead. This problem is exacerbated when we don't even have the reference pose
           and only have the walking motion to estimate the skeleton.

          So the best approximation we can do is: adding the distance between ASIS ~ KNEE and KNEE ~ LANK.
          We don't know the exact joint positions yet, so we need to "wing it" with marker positions instead.
        */
        leftLegLength +=
                (glm::length(frameData[lasi] - frameData[lkne]) + glm::length(frameData[lkne] - frameData[lank]));
        rightLegLength +=
                (glm::length(frameData[rasi] - frameData[rkne]) + glm::length(frameData[rkne] - frameData[rank]));
    }

    interAsis /= (float)ref.frame_cnt;
    leftLegLength /= (float)ref.frame_cnt;
    rightLegLength /= (float)ref.frame_cnt;

    return calcHipJointOffset(interAsis, leftLegLength, rightLegLength);
}

std::tuple<std::vector<glm::vec3>, std::vector<glm::quat>> calcLowerBodyJoints(
        nonstd::span<glm::vec3> markerData, MarkerType markerType,
        glm::vec3 lHipJointOffset, glm::vec3 rHipJointOffset) {

    float kneeSize = 0.08f;
    float ankleSize = 0.06f;

    std::vector<glm::vec3> jointPos(9);
    std::vector<glm::quat> jointOrients(9);

    // Step 2. The pelvis

    // obtain the Pelvic Embedded Coordinate System
    uint32_t sacr, lasi, rasi, lank, rank, ltoe, rtoe;
    if (markerType == MarkerType::Vicon) {
        sacr = SACR; lasi = LASI; rasi = RASI; lank = LANK; rank = RANK; ltoe = LTOE, rtoe = RTOE;
    }
    else if (markerType == MarkerType::MotionAnalysis) {
        sacr = MA_V_Sacral;lasi = MA_L_ASIS; rasi = MA_R_ASIS;
        lank = MA_L_Ankle; rank = MA_R_Ankle; ltoe = MA_L_Toe, rtoe = MA_R_Toe;
    }

    vec3 MASI = (markerData[lasi] + markerData[rasi]) / 2;
    vec3 p_beta1 = MASI - markerData[sacr];
    vec3 p_beta2 = markerData[lasi] - markerData[rasi];
    vec3 e_PY = glm::normalize(p_beta2);
    vec3 p_beta3 = p_beta1 - glm::dot(p_beta1, e_PY) * e_PY;
    vec3 e_PX = glm::normalize(p_beta3);
    vec3 e_PZ = glm::cross(e_PX, e_PY);

    // obtain LHJC and RHJC
    vec3 LHJC = MASI + e_PX * lHipJointOffset.x + e_PY * lHipJointOffset.y + e_PZ * lHipJointOffset.z;
    vec3 RHJC = MASI + e_PX * rHipJointOffset.x + e_PY * rHipJointOffset.y + e_PZ * rHipJointOffset.z;

    // obtain lumbar vertebra 5 (hmm... is it needed?)
    vec3 LUMB = (LHJC + RHJC)/2 + glm::vec3(0, 0, 0.828) * glm::length(LHJC - RHJC);

    // obtain the joint orientations of the pelvic
    quat pelvicCS = quat_cast(glm::mat3(e_PX, e_PY, e_PZ));

    // Step 3. The knee
    // Note: We perform dynamic knee/ankle joint center calculation.
    // The Vicon pyCGM2 docs use different methods for approximating the joint pos.
    // We use the pyCGM2 docs instead.

    float KO = kneeSize / 2;

    vec3 LKJC, RKJC;
    quat lHipCS, rHipCS;
    if (markerType == MarkerType::Vicon) {
        std::tie(LKJC, lHipCS) =
                jointTransFromCoronalPlane(LHJC, markerData[LTHI], markerData[LKNE], KO, false);
        std::tie(RKJC, rHipCS) =
                jointTransFromCoronalPlane(RHJC, markerData[RTHI], markerData[RKNE], KO, true);
    }
    else if (markerType == MarkerType::MotionAnalysis) {
        std::tie(LKJC, lHipCS) =
                jointTransFromSagittalPlane(LHJC, markerData[MA_L_Thigh], markerData[MA_L_Knee], KO, false);
        std::tie(RKJC, rHipCS) =
                jointTransFromSagittalPlane(RHJC, markerData[MA_R_Thigh], markerData[MA_R_Knee], KO, true);
    }

    // Step 4. The femur

    vec3 LAJC, RAJC;
    quat lKneeCS, rKneeCS;
    float AO = ankleSize / 2;
    if (markerType == MarkerType::Vicon) {
        std::tie(LAJC, lKneeCS) =
                jointTransFromCoronalPlane(LKJC, markerData[LTIB], markerData[LANK], AO, false);
        std::tie(RAJC, rKneeCS) =
                jointTransFromCoronalPlane(RKJC, markerData[RTIB], markerData[RANK], AO, true);
    }
    else if (markerType == MarkerType::MotionAnalysis) {
        std::tie(LAJC, lKneeCS) =
                jointTransFromSagittalPlane(LKJC, markerData[MA_L_Shank], markerData[MA_L_Ankle], AO, false);
        std::tie(RAJC, rKneeCS) =
                jointTransFromSagittalPlane(RKJC, markerData[MA_R_Shank], markerData[MA_R_Ankle], AO, true);
    }

    // Step 5. The Foot

    vec3 lToePos, rToePos;
    quat lAnkleCS, rAnkleCS;
    vec3 e_LAX = glm::normalize(markerData[ltoe] - LAJC);
    vec3 e_LAY = glm::normalize(markerData[lank] - LAJC);
    e_LAY = glm::normalize(e_LAY - glm::dot(e_LAY, e_LAX) * e_LAX);
    vec3 e_LAZ = glm::cross(e_LAX, e_LAY);
    lAnkleCS = quat_cast(glm::mat3(e_LAX, e_LAY, e_LAZ));

    vec3 e_RAX = glm::normalize(markerData[rtoe] - RAJC);
    vec3 e_RAY = glm::normalize(RAJC - markerData[rank]);
    e_RAY = glm::normalize(e_RAY - glm::dot(e_RAY, e_RAX) * e_RAX);
    vec3 e_RAZ = glm::cross(e_RAX, e_RAY);
    rAnkleCS = quat_cast(glm::mat3(e_RAX, e_RAY, e_RAZ));

    // Our final joint positions and orientations
    jointPos[Pelvic] = MASI;
    jointPos[L_Hip] = LHJC;
    jointPos[L_Knee] = LKJC;
    jointPos[L_Ankle] = LAJC;
    jointPos[L_Toe] = markerData[ltoe];
    jointPos[R_Hip] = RHJC;
    jointPos[R_Knee] = RKJC;
    jointPos[R_Ankle] = RAJC;
    jointPos[R_Toe] = markerData[rtoe];

    jointOrients[Pelvic] = pelvicCS;
    jointOrients[L_Hip] = glm::conjugate(pelvicCS) * lHipCS;
    jointOrients[L_Knee] = glm::conjugate(lHipCS) * lKneeCS;
    jointOrients[L_Ankle] = glm::conjugate(lKneeCS) * lAnkleCS;
    jointOrients[L_Toe] = glm::identity<glm::quat>();
    jointOrients[R_Hip] = glm::conjugate(pelvicCS) * rHipCS;
    jointOrients[R_Knee] = glm::conjugate(rHipCS) * rKneeCS;
    jointOrients[R_Ankle] = glm::conjugate(rKneeCS) * rAnkleCS;
    jointOrients[R_Toe] = glm::identity<glm::quat>();

    return {jointPos, jointOrients};
}

MotionClipData convertToLowerBodyBVH(PointMotionClip& ref, PointMotionClip& clip) {
    using glm::vec3;

    clip.jointData.resize(clip.frame_cnt * 3 * 9);
    clip.vmarkerData.resize(clip.frame_cnt * 3 * 4);

    MotionClipData bvh;
    bvh.valid = true;
    bvh.numChannels = 24;
    bvh.numFrames = clip.frame_cnt;
    bvh.frameTime = clip.frame_dt;
    bvh.poseStates.resize(clip.frame_cnt);

    assert(clip.point_cnt == 13);

    auto [lHipJointOffset, rHipJointOffset] = calcHipJointOffset(ref);

    glm::vec3 pelvisPos = {0, 0, 0};
    float lFemurLength = 0;
    float lTibiaLength = 0;
    float lFootLength = 0;
    float rFemurLength = 0;
    float rTibiaLength = 0;
    float rFootLength = 0;

    int validRefFrameCount = 0;
    const float eps = glm::epsilon<float>();
    for (int f = 0; f < ref.frame_cnt; f++) {
        auto markerData = ref.getMarkerFrameData(f);
        bool valid = true;
        for (auto& d : markerData) {
            if (std::isnan(d.x) || std::isnan(d.y) || std::isnan(d.z)) {
                valid = false;
                break;
            }
            if (glm::epsilonEqual(d.x, 0.f, eps) &&
                glm::epsilonEqual(d.y, 0.f, eps) &&
                glm::epsilonEqual(d.z, 0.f, eps)) {

                valid = false;
                break;
            }
        }
        if (!valid) continue;

        auto [jointPos, jointOrients] = calcLowerBodyJoints(markerData, ref.markerType,
                lHipJointOffset, rHipJointOffset);

        pelvisPos += jointPos[Pelvic];

        lFemurLength += glm::length(jointPos[L_Hip] - jointPos[L_Knee]);
        lTibiaLength += glm::length(jointPos[L_Knee] - jointPos[L_Ankle]);
        lFootLength += glm::length(jointPos[L_Ankle] - jointPos[L_Toe]);

        rFemurLength += glm::length(jointPos[R_Hip] - jointPos[R_Knee]);
        rTibiaLength += glm::length(jointPos[R_Knee] - jointPos[R_Ankle]);
        rFootLength += glm::length(jointPos[R_Ankle] - jointPos[R_Toe]);

        validRefFrameCount++;
    }
    if (validRefFrameCount == 0) {
        fmt::print("Reference motion has too many NaNs and is invalid!\n");
        bvh.valid = false;
        return bvh;
    }

    pelvisPos /= validRefFrameCount;
    lFemurLength /= validRefFrameCount;
    lTibiaLength /= validRefFrameCount;
    lFootLength /= validRefFrameCount;
    rFemurLength /= validRefFrameCount;
    rTibiaLength /= validRefFrameCount;
    rFootLength /= validRefFrameCount;

    // Create the skeleton
    bvh.poseTree.numJoints = 9;
    bvh.poseTree.numNodes = 11;
    bvh.poseTree.allNodes.resize(bvh.poseTree.numNodes);

    bvh.poseTree[0].name = "Pelvic";
    bvh.poseTree[0].offset = vec3(0);
    bvh.poseTree[0].parent = 0;
    bvh.poseTree[0].childJoints = {1, 5};

    bvh.poseTree[1].name = "L_Hip";
    bvh.poseTree[1].offset = lHipJointOffset;
    bvh.poseTree[1].parent = 0;
    bvh.poseTree[1].childJoints = {2};

    bvh.poseTree[2].name = "L_Knee";
    bvh.poseTree[2].offset = vec3(0, 0, -lFemurLength);
    bvh.poseTree[2].parent = 1;
    bvh.poseTree[2].childJoints = {3};

    bvh.poseTree[3].name = "L_Ankle";
    bvh.poseTree[3].offset = vec3(0, 0, -lTibiaLength);
    bvh.poseTree[3].parent = 2;
    bvh.poseTree[3].childJoints = {4};

    bvh.poseTree[4].name = "L_Toe";
    bvh.poseTree[4].offset = vec3(lFootLength, 0, 0);
    bvh.poseTree[4].parent = 3;
    bvh.poseTree[4].childJoints = {9};

    bvh.poseTree[5].name = "R_Hip";
    bvh.poseTree[5].offset = rHipJointOffset;
    bvh.poseTree[5].parent = 0;
    bvh.poseTree[5].childJoints = {6};

    bvh.poseTree[6].name = "R_Knee";
    bvh.poseTree[6].offset = vec3(0, 0, -rFemurLength);
    bvh.poseTree[6].parent = 5;
    bvh.poseTree[6].childJoints = {7};

    bvh.poseTree[7].name = "R_Ankle";
    bvh.poseTree[7].offset = vec3(0, 0, -rTibiaLength);
    bvh.poseTree[7].parent = 6;
    bvh.poseTree[7].childJoints = {8};

    bvh.poseTree[8].name = "R_Toe";
    bvh.poseTree[8].offset = vec3(rFootLength, 0, 0);
    bvh.poseTree[8].parent = 7;
    bvh.poseTree[8].childJoints = {10};

    bvh.poseTree[9].name = "L_Toe_End";
    bvh.poseTree[9].offset = vec3(0);
    bvh.poseTree[9].parent = 4;
    bvh.poseTree[9].childJoints = {};

    bvh.poseTree[10].name = "R_Toe_End";
    bvh.poseTree[10].offset = vec3(0);
    bvh.poseTree[10].parent = 8;
    bvh.poseTree[10].childJoints = {};

    bvh.poseStates.resize(clip.frame_cnt);

    // Create the poses frame by frame
    for (int f = 0; f < clip.frame_cnt; f++) {
        auto markerData = clip.getMarkerFrameData(f);
        auto [jointPos, jointOrients] = calcLowerBodyJoints(markerData, clip.markerType,
                lHipJointOffset, rHipJointOffset);
        clip.jointData.resize(clip.frame_cnt * 9);
        for (int p = 0; p < 9; p++) {
            clip.setJointPos(f, p, jointPos[p]);
        }
        glmx::pose& pose = bvh.poseStates[f];
        pose = glmx::pose::empty(9);
        pose.v = jointPos[Pelvic];
        std::copy(jointOrients.begin(), jointOrients.end(), pose.q.begin());
    }

    // Rotate the BVH so that Y axis goes up instead of Z axis
    bvh.switchZtoYup();

    return bvh;
}

}
