//
// Created by lasagnaphil on 20. 2. 24..
//

#ifndef GAIT_ANALYSIS_FILESYSTEM_H
#define GAIT_ANALYSIS_FILESYSTEM_H

#if __has_include(<filesystem>)
#include <filesystem>
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
namespace std {
    namespace filesystem = experimental::filesystem;
}
#else
#error "no filesystem support"
#endif

#endif //GAIT_ANALYSIS_FILESYSTEM_H
