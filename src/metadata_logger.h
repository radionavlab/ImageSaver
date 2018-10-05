// Author: Tucker Haydon

#ifndef IMAGE_SAVER_METADATA_LOGGER_H
#define IMAGE_SAVER_METADATA_LOGGER_H

#include <string>

namespace image_saver {

  class MetadataLogger() {
    private:
      std::string filename_;      
      void MakeLogDirectory();

    public:
      MetadataLogger();
      void LogMetadata();
  };

};
#endif
