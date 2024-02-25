// #include "vex.h"
// #include <string>

// #pragma once

// /*
// * Plays a collection of frames in a folder
// */
// class MoviePlayer {
// public:
//     MoviePlayer(std::string basePath);
//     void play();
//     void pause();
//     void seek(int time);

//     int getTime();

// private:
//     std::string basePath;
//     bool playing;
//     int currentFrame;

//     int backgroundTask();

//     static int backgroundTaskWrapper(void* instance) {
//         return static_cast<MoviePlayer*>(instance)->backgroundTask();
//     }
// };