// Sync_3DG.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <iostream>             // Terminal IO
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering
#include "example-imgui.hpp"
#include <chrono>
#include <imgui.h>
#include "imgui_impl_glfw.h"
#include <vector>
#include <thread>
#include <atomic>
#include <queue>
#include <direct.h>
#include <iomanip>
#include <mutex>
#include <condition_variable>
#include <GLFW/glfw3.h> // Include GLFW header
#include "lsl_cpp.h" 
#include <stdlib.h>
#include <array>
#include "stdafx.h"
#include <fstream>              // File IO
#include <sstream>              // Stringstreams
#include <opencv2/opencv.hpp>
std::mutex mtx;
std::condition_variable cond_var;
std::condition_variable queueNotFull;
bool dataReady = false;

std::string savePath = "C:/Users/RL000009/source/repos/WP1_recordings/";//"C:/Users/RL000009/OneDrive - Vrije Universiteit Brussel/Documents/ASKILLS/WP1/Recordings/";

// Queue for data records
std::queue<DOUBLE_POSITION_ANGLES_TIME_STAMP_RECORD> dataQueue;
std::queue<int> sIDQueue;
const int maxQueueSize = 10;
rs2::frame_queue framesQueue(50);

// Global atomic flag to control thread execution
std::atomic<bool> running(true);
std::string dirName;

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
// Helper function for writing metadata to disk as a csv file
void metadata_to_csv(const rs2::frame& frm, const std::string& filename);

void errorHandler(int error);

std::string createTimestampMs() {
    auto now = std::chrono::system_clock::now();
    auto nowAsTimeT = std::chrono::system_clock::to_time_t(now);
    auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::tm* nowTm = std::localtime(&nowAsTimeT);

    std::ostringstream oss;
    oss << std::put_time(nowTm, "%Y-%m-%d_%H-%M-%S-") << std::setfill('0') << std::setw(3) << nowMs.count();
    return oss.str();
}

std::string getCurrentDateTime() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

void initializeRealSense(rs2::pipeline &p, rs2::config &cfg) {
    cfg.enable_stream(RS2_STREAM_DEPTH); // Enable depth stream
    cfg.enable_stream(RS2_STREAM_COLOR); // Enable color stream
    // Start the pipeline with the configuration
    p.start(cfg);
    std::cout << "RealSense D405 initialized" << std::endl;
}

void initializeTrackStar(CSystem &ATC3DG) {
    // Hypothetical function calls to initialize the 3D Guidance TrackStar
    
    CSensor* pSensor;
    CXmtr* pXmtr;

    int				errorCode;
    int             i;
    short           id;
    // Initialize the ATC3DG system
    printf("Initializing ATC3DG system...\n");
    errorCode = InitializeBIRDSystem();
    if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode);

    errorCode = GetBIRDSystemConfiguration(&ATC3DG.m_config);
    if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode);

    pSensor = new CSensor[ATC3DG.m_config.numberSensors];
    for (i = 0; i < ATC3DG.m_config.numberSensors; i++)
    {
        errorCode = GetSensorConfiguration(i, &(pSensor + i)->m_config);
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode);
    }


    pXmtr = new CXmtr[ATC3DG.m_config.numberTransmitters];
    for (i = 0; i < ATC3DG.m_config.numberTransmitters; i++)
    {
        errorCode = GetTransmitterConfiguration(i, &(pXmtr + i)->m_config);
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode);
    }


    // Set the data format type for each attached sensor.
    for (i = 0; i < ATC3DG.m_config.numberSensors; i++)
    {
        DATA_FORMAT_TYPE type = DOUBLE_POSITION_ANGLES_TIME_STAMP;
        errorCode = SetSensorParameter(i, DATA_FORMAT, &type, sizeof(type));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode);
    }

    pXmtr = new CXmtr[ATC3DG.m_config.numberTransmitters];
    for (i = 0; i < ATC3DG.m_config.numberTransmitters; i++)
    {
        errorCode = GetTransmitterConfiguration(i, &(pXmtr + i)->m_config);
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode);
    }

    for (id = 0; id < ATC3DG.m_config.numberTransmitters; id++)
    {
        if ((pXmtr + id)->m_config.attached)
        {
            // Transmitter selection is a system function.
            // Using the SELECT_TRANSMITTER parameter we send the id of the
            // transmitter that we want to run with the SetSystemParameter() call
            errorCode = SetSystemParameter(SELECT_TRANSMITTER, &id, sizeof(id));
            if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode);
            break;
        }
    }
    std::cout << "3D Guidance TrackStar initialized" << std::endl;
}

// Function to wait for the ENTER key and then signal all threads to stop
void waitForEnter() {
    std::cout << "Press ENTER to stop the application...\n";
    std::cin.get();
    running.store(false); // Set stopSignal to true to signal all threads to stop
}

void acquireRealSenseData(rs2::pipeline& p) {
    int fpscounter = 0;
    while (running) {
        rs2::frameset frameset = p.wait_for_frames(); // Get a set of frames from the camera
        for (auto&& frame : frameset) {
            framesQueue.enqueue(frame); // Enqueue each frame
            fpscounter++;
        }
        // Optional: Break condition or sleep
    }
}

// Consumer function for data source 1 using RealSense frames
void saveRealSenseData() {
    std::stringstream filename;
    std::string folderName = getCurrentDateTime();

    std::string dir = "./" + dirName + "/RealSense/";

    rs2::colorizer color_map;

    while (running) {
        rs2::frame frame;
        if (framesQueue.poll_for_frame(&frame)) { // Try to get a frame from the queue
            auto vf = frame.as<rs2::video_frame>();
           
            // Use the colorizer to get an rgb image for the depth stream
            if (vf.is<rs2::depth_frame>()) vf = color_map.process(frame);

            // Write images to disk
            double timestamp = vf.get_timestamp();
            std::stringstream filename;
            filename << dir << "frame_" << std::fixed << std::setprecision(0) << timestamp << "-" << vf.get_profile().stream_name() << ".png";
            
            //stbi_write_png(filename.str().c_str(), vf.get_width(), vf.get_height(),
                //vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
            std::cout << "Saved " << filename.str() << std::endl;

            /*
            std::stringstream csv_file;
            csv_file << dir << "frame_" << std::fixed << std::setprecision(0) << timestamp << "-" << vf.get_profile().stream_name()
                << "-metadata.csv";
            metadata_to_csv(vf, csv_file.str());
            */
            
        }
    }
}

void acquireTrackStarData(CSystem& ATC3DG) {

    int				errorCode;		// used to hold error code returned from procedure call
    int				i = 0;
    int				sensorID;
    char			output[256];
    int				numberBytes;
    clock_t			goal;
    clock_t			wait = 33;		// 33 ms delay used to "pace" the calls to get a data record


    goal = wait + clock();

    while (running){
        std::unique_lock<std::mutex> lk(mtx);
        queueNotFull.wait(lk, [] { return dataQueue.size()  <maxQueueSize; });

        // delay 10ms between collecting data
        // wait till time delay expires
        while (goal > clock());
        // set up time delay for next loop
        goal = wait + clock();
        // Fetch data from TrackStar

        // Collect data from all birds
        // Loop through all sensors and get a data record if the sensor is attached.
        // Print result to screen
        // Note: The default data format is DOUBLE_POSITION_ANGLES. We can use this
        // format without first setting it.
        
        DOUBLE_POSITION_ANGLES_TIME_STAMP_RECORD record, * pRecord = &record;

        // scan the sensors and request a record if the sensor is physically attached
        for (sensorID = 0; sensorID < ATC3DG.m_config.numberSensors; sensorID++)
        {
            // sensor attached so get record
            errorCode = GetAsynchronousRecord(sensorID, pRecord, sizeof(record));
            if (errorCode != BIRD_ERROR_SUCCESS) { errorHandler(errorCode); }
            // send output to console
                sprintf(output, "%4d [%d] %8.3f %8.3f %8.3f: %8.2f %8.2f %8.2f\n",
                    record.time,
                    sensorID,
                    record.x,
                    record.y,
                    record.z,
                    record.a,
                    record.e,
                    record.r
                );
            numberBytes = strlen(output);
            printf("%s", output);

            dataQueue.push(record);
            sIDQueue.push(sensorID);

        }

        lk.unlock();
        cond_var.notify_one();

        dataReady = true;

        // Synchronize and process your data here


    }

}

void saveTrackStarData(std::string trackstarDir) {

    std::stringstream filename;
    std::string folderName = getCurrentDateTime();

    
    filename << trackstarDir << "/record_" << folderName << ".csv";

    // Open a file in append mode
    std::ofstream outFile(filename.str(), std::ios::app);

    // Check if file is open
    if (!outFile.is_open()) {
        std::cerr << "Failed to open file: " << filename.str() << std::endl;
        return;
    }

    outFile
        << "Timestamp" << ", " << "Sensor ID, "
        << "X" << ", " << "Y" << ", " << "Z" << ", "
        << "A" << ", " << "E" << ", " << "R" << "\n";

    while (running) {
        std::unique_lock<std::mutex> lk(mtx);
        cond_var.wait(lk, [] { return !dataQueue.empty(); });

        if (!dataQueue.empty()) {
            DOUBLE_POSITION_ANGLES_TIME_STAMP_RECORD record = dataQueue.front();
            int sensorID = sIDQueue.front();
            dataQueue.pop();
            sIDQueue.pop();

            lk.unlock();
            queueNotFull.notify_one();

            
            outFile << std::fixed << std::setprecision(3) << record.time << ", " << sensorID << ", ";
            outFile << record.x << ", " << record.y << ", " << record.z << ", "
                << record.a << ", " << record.e << ", " << record.r << "\n";

    
            // Save the record to disk (implement actual saving logic here)
        }

    }

    // Close the file
    outFile.close();
}

void timestamp2LSL(lsl::stream_info info) {
    // make a new outlet
    lsl::stream_outlet outlet(info, 0, 100);
    std::vector<int> sample(2,0.0);
    int duration_ms = 0;
    while (running) {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        sample[0] = static_cast<uint32_t>(in_time_t);
        sample[1] = duration_ms++;
        outlet.push_sample(sample);
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1000 Hz
    }

}

int main() {
    try

    {

        // LSL timestamp setup 
        lsl::stream_info info("SyncApp", "Timestamp", 2, 1000, lsl::cf_int32, "SyncApp - Timestamp");
        

        CSystem	ATC3DG;
        try {
            initializeTrackStar(ATC3DG);
        }
        catch (std::exception& e) {
            std::cout << "3D Guidance TrackStar error" << std::endl;
        }


        std::cout << "Ready. Press ENTER to start recording (set LSL folder now)" << std::endl;
        std::string input2;
        std::getline(std::cin, input2);
        //window app(1280, 720, "RealSense Capture"); // Initialization
        //ImGui_ImplGlfw_Init(app, false);      // ImGui library intializition


        //Create main records directory
        dirName = savePath + getCurrentDateTime();
        bool newdir = _mkdir(dirName.c_str());
        std::string trackstarDir = dirName + "/3DG";
        std::string realsenseDir = dirName + "/RealSense";
        newdir = _mkdir(trackstarDir.c_str());
        newdir = _mkdir(realsenseDir.c_str());

        std::string serial;
        rs2::pipeline p;
        rs2::config cfg;
        std::string bagFile = realsenseDir + "/" + getCurrentDateTime() + "_record.bag";

        std::cout << "Press ENTER to start capturing" << std::endl;
        std::string input;
        std::getline(std::cin, input);

        cfg.enable_record_to_file(bagFile);
        if (!device_with_streams({ RS2_STREAM_COLOR,RS2_STREAM_DEPTH }, serial))
            return EXIT_SUCCESS;
        if (!serial.empty())
            cfg.enable_device(serial);
        initializeRealSense(p, cfg);
   

        std::thread timestamp2LSLThread(timestamp2LSL, info);
        std::thread trackstarDataThread(acquireTrackStarData, ATC3DG);
        std::thread trackstarSaveThread(saveTrackStarData, trackstarDir);
        std::thread realsenseDataThread(acquireRealSenseData, p);
        std::thread realsenseSaveThread(saveRealSenseData);

        /*
        while (app && running) {
            rs2::frame frame;
            if (framesQueue.poll_for_frame(&frame)) {
                app.show(frame);
            }
        }
        */
        std::thread stopper(waitForEnter);

        timestamp2LSLThread.join();
        realsenseDataThread.join();
        realsenseSaveThread.join();
        stopper.join();
        trackstarDataThread.join();
        trackstarSaveThread.join();

     
    }
    catch (const rs2::error& e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return 0;

}


void errorHandler(int error) {
    char buffer[1024];
    char* pBuffer = &buffer[0];
    int numberBytes;

    while (error != BIRD_ERROR_SUCCESS) {
        error = GetErrorText(error, pBuffer, sizeof(buffer), SIMPLE_MESSAGE);
        numberBytes = strlen(buffer);
        buffer[numberBytes] = '\n'; // append a newline to buffer
        printf("%s", buffer);
        std::string input;
        std::getline(std::cin, input);
    }

    
    exit(0);
}


void metadata_to_csv(const rs2::frame& frm, const std::string& filename)
{
    std::ofstream csv;

    csv.open(filename);

    //    std::cout << "Writing metadata to " << filename << endl;
    csv << "Stream," << rs2_stream_to_string(frm.get_profile().stream_type()) << "\nMetadata Attribute,Value\n";

    // Record all the available metadata attributes
    for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
    {
        if (frm.supports_frame_metadata((rs2_frame_metadata_value)i))
        {
            csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
                << frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
        }
    }

    csv.close();
}
