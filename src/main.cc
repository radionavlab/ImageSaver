// Author: Tucker Haydon

#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <string>

#include "node.h"
#include "image_saver.h"

bool OK{true};

void SigHandler(int s) {
    OK = false;
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
    image_saver::ImageSaver is(save_directory_path, text_file_name, camera_server_path);

    // node.Start();
    is.Start();

    // while(OK && sleep(1) == 0);
    return EXIT_SUCCESS;
}

