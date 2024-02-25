// #include "funny/movieplayer.h"
// #include "config.h"
// #include <iostream>
// #include <string>
// #include <sstream>

// // using namespace vex;

// template <typename T>
// std::string to_string(T value)
// {
//     std::ostringstream os ;
//     os << value ;
//     return os.str() ;
// }

// MoviePlayer::MoviePlayer(std::string basePath) {
//     this->currentFrame = 0;
//     this->playing = false;
//     this->basePath = basePath;

//     task backgroundTask(backgroundTaskWrapper, this);
// }

// void MoviePlayer::play() {
//     this->playing = true;
// }

// void MoviePlayer::pause() {
//     this->playing = false;
// }

// void MoviePlayer::seek(int time) {
//     this->currentFrame = time;
// }

// int MoviePlayer::getTime() {
//     return this->currentFrame;
// }

// int MoviePlayer::backgroundTask() {
//     while (true) {
//         vex::task::sleep(2);
//         if (this->playing) {
//             this->currentFrame++;
//         } else {
//             continue;
//         }
//         std::string path = this->basePath + "/frame_" + to_string(this->currentFrame) + ".png";
//         if (!Brain.SDcard.exists(path.c_str())) {
//             this->currentFrame = 0;
//             continue;
//         }
//         printf("Path: %s\n", path.c_str());

//         bool suc = Brain.Screen.drawImageFromFile(path.c_str(), 0, 0);
//         printf("Success: %d\n", suc);
//     }
// }