// Author: Tucker Haydon

#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <string>
#include <thread>
#include <iostream>

#include "node.h"
#include "image_saver.h"

namespace{
    image_saver::ImageSaver* is;
    image_saver::Node* node;
}

void WaitExit() {
    std::cin.tie(nullptr);
    do {
        std::cout << "Press 'q' to quit." << std::endl << std::endl;
    } while (std::cin.get() != 'q');
    is->Stop();
    node->Stop();
}

int main(int argc, char **argv) {
    static const std::string camera_server_path  = "/tmp/camera-server";
    static const std::string save_directory_path = "/mnt/storage/images/";

    node = new image_saver::Node(argc, argv);
    is = new image_saver::ImageSaver(save_directory_path, camera_server_path);

    std::thread node_thread(&image_saver::Node::Start, node);
    std::thread is_thread(&image_saver::ImageSaver::Start, is);
    std::thread exit_thread(WaitExit);

    node_thread.join();
    is_thread.join();
    exit_thread.join();

    return EXIT_SUCCESS;
}

