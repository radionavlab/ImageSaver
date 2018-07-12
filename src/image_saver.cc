// Author: Tucker Haydon

#include "image_saver.h"
#include "gps.h"

#include <iomanip>
#include <sys/stat.h>
#include <fstream>
#include <iostream>

namespace image_saver {

ImageSaver::ImageSaver(const std::string& save_directory_path,
                       const std::string& text_file_name,
                       const std::string& camera_server_path) {
    this->save_directory_path_ = save_directory_path;
    this->text_file_name_ = text_file_name;
    this->camera_server_path_ = camera_server_path;
    this->MakeSaveDirectory();
    this->text_file_path_ = this->save_directory_path_ + "/" + this->text_file_name_;
}

void ImageSaver::Stop() {
    this->stopped_ = true;
}

void ImageSaver::Start() {
    quadcam::CameraClient client(this->camera_server_path_);

    this->stopped_ = false;
    while(!this->stopped_) {
        quadcam::FrameData frame_data = client.RequestFrame();
        this->SaveImage(frame_data);
    }
}

void ImageSaver::WriteTextHeader() {
    std::string command = "echo '# IMAGENAME X Y Z POSCOV[6] EL AZ ELSIGMA AZSIGMA ATTCOV[6]' >> " + this->text_file_path_;
    std::system(command.c_str());
}

std::string ImageSaver::NextImageFileName() {
    static int seq = 0;
    std::stringstream ss;
    ss << std::setw(5) << std::setfill('0') << seq++;
    std::string filename = "frame" + ss.str() + ".nv12";
    return filename;
}

void ImageSaver::WriteTextLine(const std::string& filename) {

    // Copy current GPS position
    GPSSolution gps_solution_copy;
    {
        std::lock_guard<std::mutex> lock(gps_solution_mutex);
        gps_solution_copy = gps_solution;
    }


    /* Write camera pose to file */
    std::ostringstream data;
    data << std::fixed << std::setprecision(10);
    
    // Write camera position
    data << " " << gps_solution_copy.x;
    data << " " << gps_solution_copy.y;
    data << " " << gps_solution_copy.z;

    for(int i=0; i < 6; i++) {
        data << " " << gps_solution_copy.posCov[i];
    }

    // Write camera orientation
    data << " " << gps_solution_copy.el;
    data << " " << gps_solution_copy.az;

    data << " " << gps_solution_copy.elSigma;
    data << " " << gps_solution_copy.azSigma;
 
    for(int i=0; i < 6; i++) {
        data << " " << gps_solution_copy.attCov[i];
    }

    std::string command = "echo '" + filename + data.str() + "' >> " + this->text_file_path_;
    std::system(command.c_str());
}

void ImageSaver::SaveImage(const quadcam::FrameData& frame_data) {
    // Write text header
    static bool header = false;
    if(!header) { this->WriteTextHeader(); header = true; }

    std::string filename = this->NextImageFileName();

    // Write text line
    this->WriteTextLine(filename);

    // Write image data to disk
    std::ios_base::sync_with_stdio(false);
    std::fstream img_file(this->save_directory_path_ + "/" + filename, std::ios::out | std::ios::binary);
    img_file.write((char*)frame_data.data.get(), frame_data.meta_data.data_size);
    img_file.close();
}

void ImageSaver::MakeSaveDirectory() {

    /* Make a directory for all of the images */
    time_t now = time(0);
    tm *ltm = localtime(&now);
    std::string year = std::to_string(1900 + ltm->tm_year);
    std::string month = std::to_string(1 + ltm->tm_mon);
    std::string day = std::to_string(ltm->tm_mday);
    std::string hour = std::to_string(ltm->tm_hour);
    std::string min = std::to_string(ltm->tm_min);
    std::string sec = std::to_string(ltm->tm_sec);
    std::string dir = year + "-" + month + "-" + day + "-" + hour + "-" + min + "-" + sec + "/";
    this->save_directory_path_ = this->save_directory_path_ + dir;
    const int err = mkdir(this->save_directory_path_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if(err == -1){
        std::cout << "Could not make save directory!" << std::endl;
        exit(EXIT_FAILURE);
    }
}

};
