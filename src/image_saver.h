// Author: Tucker Haydon

#ifndef IMAGE_SAVER_IMAGE_SAVER_H
#define IMAGE_SAVER_IMAGE_SAVER_H

#include <string>
#include <atomic>
#include "camera_client.h"
#include "metadata_logger.h"

namespace image_saver {

  class ImageSaver {
    private:
      std::string save_directory_path_;
      std::string camera_server_path_;
      std::atomic<bool> stopped_{false};
      MetadataLogger* metadata_logger_ptr_;
    
      /*
       * Save an image to disk.
       */
      void SaveImage(const quadcam::FrameData& frame_data); 
   
     /*
      * Generate the next image name.
      */ 
      std::string NextImageFileName(); 

      /*
       * Make the directory where images are saved
       */
      void MakeSaveDirectory();

    public: 
      /*
       * Constructor.
       */
      ImageSaver(const std::string& save_directory_path,
                 const std::string& camera_server_path);

      /*
       * Start saving images on new thread.
       */
      void Start();

      /*
       * Stop saving images. Asynchronous.
       */
      void Stop(); 
  };
  
};

#endif
