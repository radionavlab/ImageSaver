// Author: Tucker Haydon

#ifndef IMAGE_SAVER_IMAGE_SAVER_H
#define IMAGE_SAVER_IMAGE_SAVER_H

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
      std::string camera_server_path_;
      std::atomic<bool> stopped_{false};
    
      void SaveImage(const quadcam::FrameData& frame_data); 
    
      std::string NextImageFileName(); 
  };
  
};
#endif
