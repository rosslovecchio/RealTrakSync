// trakstarLSL.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "include/lsl_cpp.h"
#include <array>
#include <iostream>
#include "stdafx.h"
#include <cmath>
#include <iostream>
#include <thread>

void errorHandler(int error);

const char* sensor_names[] = { "Sensor0", "Sensor1", "Sensor2", "Sensor3" }; // Modify with actual sensor names
const char* chls[] = { "ID","STATUS", "time","x","y","z","azimuth","elevation","roll"};


int main() {

    // LSL SETUP
    int n_channels = 9;
    int samplingrate = 80;
    int max_buffered = 360;

    // 3DG SETUP
    CSystem			ATC3DG;
    CSensor         *pSensor;
    CXmtr           *pXmtr;
    int				errorCode;
    int				i;
    int				sensorID;
    short			id;
    int				records;
    char			output[256];
    int				numberBytes;
    clock_t			goal;
    clock_t			wait = 10;	// 10 ms delay

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
    try {
        // Initialize the LSL outlet
        lsl::stream_info info("TrackstarSensors", "PositionAndOrientation", n_channels*4, samplingrate, lsl::cf_float32, "TrackstarSensors");
        info.desc().append_child_value("manufacturer", "LSL");
        lsl::xml_element channels = info.desc().append_child("channels");
        int ch;
        int k = 0;
        for(k = 0; k < 36; k++)
        {
            ch = k % 9;
            channels.append_child("channel")
                .append_child_value("label", chls[ch])
                .append_child_value("unit", "MM/DEG/RAD")
                .append_child_value("type", "POS/ORIENT");
        }
           
        /*
            channels.append_child("channel").append_child_value("label", "ID" + std::to_string(k));
            channels.append_child("channel").append_child_value("label", "ID" + std::to_string(k));


            channels.append_child("channel").append_child_value("label", "status" + std::to_string(k));
            channels.append_child("channel").append_child_value("label", "timestamp" + std::to_string(k));
            channels.append_child("channel").append_child_value("label", "x" + std::to_string(k));
            channels.append_child("channel").append_child_value("label", "y" + std::to_string(k));
            channels.append_child("channel").append_child_value("label", "z" + std::to_string(k));
            channels.append_child("channel").append_child_value("label", "roll" + std::to_string(k));
            channels.append_child("channel").append_child_value("label", "pitch" + std::to_string(k));
            channels.append_child("channel").append_child_value("label", "yaw" + std::to_string(k)); */ 
       
        lsl::stream_outlet outlet(info , 0, max_buffered);

        printf("LSL conf done");

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


        DOUBLE_POSITION_ANGLES_TIME_Q_RECORD record[8*4], *pRecord = record;
        // Set the data format type for each attached sensor.
        for (i = 0; i < ATC3DG.m_config.numberSensors; i++)
        {
            DATA_FORMAT_TYPE type = DOUBLE_POSITION_ANGLES_TIME_Q;
            errorCode = SetSensorParameter(i, DATA_FORMAT, &type, sizeof(type));
            if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode);
        }

        goal = wait + clock();
        // Loop through all sensors
        std::vector<float> sample(9 * 4); // Data sample to be sent
        // Main loop for streaming data
        while (true) {
           

            goal = wait + clock();

            for (int sensorID = 0; sensorID < 4; sensorID++) { // Assuming 4 sensors, modify accordingly
                
                errorCode = GetSynchronousRecord(ALL_SENSORS, pRecord, sizeof(record[0]) * ATC3DG.m_config.numberSensors);
                if (errorCode != BIRD_ERROR_SUCCESS)
                {
                    errorHandler(errorCode);
                }


                unsigned int status = GetSensorStatus(sensorID);

                if (status == VALID_STATUS)
                {
                    printf( "[%d] 0x%04x %8.3f %8.3f %8.3f: %8.2f %8.2f %8.2f %8.4f\n",
                        sensorID,
                        status,
                        record[sensorID].x,
                        record[sensorID].y,
                        record[sensorID].z,
                        record[sensorID].a,
                        record[sensorID].e,
                        record[sensorID].r,
                        record[sensorID].time
                    );

                    // Extract position and orientation data
                    sample[0 + n_channels * sensorID] = sensorID;  
                    sample[1 + n_channels * sensorID] = status;
                    sample[2 + n_channels * sensorID] = record[sensorID].time;
                    sample[3 + n_channels * sensorID] = record[sensorID].x;
                    sample[4 + n_channels * sensorID] = record[sensorID].y;
                    sample[5 + n_channels * sensorID] = record[sensorID].z;
                    sample[6 + n_channels * sensorID] = record[sensorID].a;
                    sample[7 + n_channels * sensorID] = record[sensorID].e;
                    sample[8 + n_channels * sensorID] = record[sensorID].r;

                    
                }
                // Push the sample to the LSL outlet
                outlet.push_sample(sample);

            }

            
        }
    }
    catch (std::exception& e) {
        std::cerr << "Got an exception: " << e.what() << std::endl;

        

    }

    id = -1;
    errorCode = SetSystemParameter(SELECT_TRANSMITTER, &id, sizeof(id));
    if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode);

    delete[] pSensor;
    delete[] pXmtr;

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
    }
    exit(0);
}