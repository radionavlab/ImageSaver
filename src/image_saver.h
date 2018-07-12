// Author: Tucker Haydon
#pragma once
#include <string>
#include <atomic>
#include "camera_client.h"

namespace image_saver {

class ImageSaver {
public: 
    ImageSaver(const std::string& save_directory_path,
               const std::string& text_file_name,
               const std::string& camera_server_path);
    void Start();
    void Stop();

private:

    std::string save_directory_path_;
    std::string text_file_name_;
    std::string camera_server_path_;
    std::string text_file_path_;
    std::atomic<bool> stopped_{false};

    void MakeSaveDirectory();
    void SaveImage(const quadcam::FrameData& frame_data); 

    std::string NextImageFileName();
    void WriteTextHeader();
    void WriteTextLine(const std::string& filename);

};

}; // namespace image_saver
