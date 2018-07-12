// Author: Tucker Haydon

#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <string>

#include "node.h"
#include "image_saver.h"

image_saver::ImageSaver* is;

void SigHandler(int s) {
    is->Stop();
}

void ConfigureSigHandler() {

    struct sigaction sigIntHandler;
    
    sigIntHandler.sa_handler = SigHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    
    sigaction(SIGINT, &sigIntHandler, NULL);
    sigaction(SIGKILL, &sigIntHandler, NULL);
}

int main(int argc, char **argv) {
    static const std::string camera_server_path  = "/tmp/camera_server";
    static const std::string save_directory_path = "/mnt/storage/images/";
    static const std::string text_file_name      = "image_data_raw.txt";

    ConfigureSigHandler();

    image_saver::Node node(argc, argv);
    is = new image_saver::ImageSaver(save_directory_path, text_file_name, camera_server_path);

    // node.Start();
    is->Start();
    return EXIT_SUCCESS;
}

