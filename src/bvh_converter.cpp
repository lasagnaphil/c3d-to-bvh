//
// Created by lasagnaphil on 19. 11. 11..
//

#include <iostream>
#include <random>
#include <regex>

#include "imguix/imgui_plot.h"

#include <MotionClipData.h>

#include "App.h"
#include "GizmosRenderer.h"
#include "PhongRenderer.h"
#include "c3d/PointMotionClip.h"

#include "filesystem.h"

namespace fs = std::filesystem;

// #define SPLIT_CYCLES
// #define OUTPUT_PHASE

int main(int argc, char **argv) {
    if (argc != 3) {
        fmt::print("Usage: {} <input_c3d_folder> <output_bvh_folder>\n", argv[0]);
        exit(0);
    }
    std::string c3dDirectory = argv[1];
    std::string bvhDirectory = argv[2];

    auto c3dFiles = std::vector<fs::directory_entry>(
            fs::directory_iterator(c3dDirectory), fs::directory_iterator());
    std::sort(c3dFiles.begin(), c3dFiles.end());

    std::vector<ga::PointMotionClip> clips;
    clips.reserve(c3dFiles.size());

    std::map<std::string, ga::PointMotionClip> refClips;

    for (auto& c3dFile : c3dFiles) {
        bool valid;
        auto clip = ga::PointMotionClip::fromC3D(c3dFile.path(), ga::MarkerType::Vicon, &valid);
        if (!valid) continue;
        clip.rescale(0.001f);
        auto namePrefix = clip.name.substr(0, 5);
        if (clip.isRefClip()) {
            if (clip.checkValidity() && refClips.count(namePrefix) == 0) {
                refClips[namePrefix] = clip;
            }
            continue;
        }
        clip.strip();
        clip.fillInvalidPoints();
        clip.smooth();
#ifdef SPLIT_CYCLES
        auto splitClips = clip.splitCycles();
        for (auto& c : splitClips) {
            c.setRootToOrigin();
        }
        clips.insert(clips.end(), splitClips.begin(), splitClips.end());
#else
        clip.setRootToOrigin();
        clips.push_back(clip);
#endif

    }

#ifdef OUTPUT_PHASE
    std::ofstream ofs("phase_data.tsv");
#endif

#ifdef SPLIT_CYCLES
    fs::create_directory("bvh");
#else
    fs::create_directory(bvhDirectory);
#endif

    int refValid = 0, refInvalid = 0;
    for (auto& clip : clips) {
        auto refClipIt = refClips.find(clip.name.substr(0, 5));
        ga::PointMotionClip refClip;
        if (refClipIt == refClips.end()) {
            fmt::print("Couldn't find reference clip for {}! Using self as reference.\n", clip.name);
            refInvalid++;
            refClip = clip;
        }
        else {
            refValid++;
            refClip = refClipIt->second;
        }

#ifdef OUTPUT_PHASE
        ofs << clip.name << "\t" << clip.frame_cnt << "\t" << clip.midPhaseIdx << std::endl;
#endif
        auto bvh = ga::convertToLowerBodyBVH(refClip, clip);

#ifndef SPLIT_CYCLES
        // If not splitting cycles, make sure exported clip isn't too short or too long
        uint32_t minNumFrames = 30, maxNumFrames = 300;
        if (bvh.numFrames < minNumFrames || bvh.numFrames > maxNumFrames) {
            fmt::print("Result BVH {} is too long or too short! (frame_cnt = {})\n", clip.name, bvh.numFrames);
            continue;
        }
#endif
        if (bvh.checkValidity()) {
#ifdef SPLIT_CYCLES
            bvh.saveToFile("datasets/hospital/bvh/" + clip.name + ".bvh", EulOrdXYZs);
#else
            bvh.saveToFile(std::string(argv[2]) + "/" + clip.name + ".bvh", EulOrdXYZs);
#endif
            fmt::print("Converted {} with reference clip {}!\n", clip.name, refClip.name);
        }
        else {
            fmt::print("Result BVH {} is invalid!\n", clip.name);
        }
    }

    fmt::print("Statistics:\n");
    fmt::print("valid = {}, invalid = {}\n", refValid, refInvalid);

    return 0;
}
