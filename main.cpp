/* 
 * File:   Ph-Controller.cpp
 * Author: tsobieroy
 * Description: Driver file for the ph Controller 
 * Created on 12. Oktober 2016, 09:28
 */
#include <iostream>
#include <cstdlib>
#include <string>
#include <string.h>
#include <cmath>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>   
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <inttypes.h> 
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
#include <cstdint>



using namespace std;


float ec_high = 0.0;
float ec_low = 0.0;
float temperature_corrected_ec_high = 0.0;
float temperature_corrected_ec_low = 0.0;

//One temperature value for one probe
string temperature_Path;

/**
 * @brief Gives back based on the temperature value the change in ph throught temp
 * @param temperature_C
 */
float adjustEcLowValue(float temperature_C) {

    if (temperature_C <= 10.0) return 0.0;
    if (temperature_C > 10.0 & temperature_C <= 15.0) return 0.0;
    if (temperature_C > 15.0 & temperature_C <= 20.0) return 0.0;
    if (temperature_C > 20.0 & temperature_C <= 25.0) return 0.0;
    if (temperature_C > 25.0 & temperature_C <= 30.0) return 0.01;
    if (temperature_C > 30.0 & temperature_C <= 35.0) return 0.02;
    if (temperature_C > 35.0 & temperature_C <= 40.0) return 0.03;
    if (temperature_C > 40.0 & temperature_C <= 50.0) return 0.05;
    if (temperature_C > 50.0) return 0.08;

    return 0.0;
}

/**
 * @brief Gives back based on the temperature value the change in ph throught temp
 * @param temperature_C
 */
float adjustEcHighValue(float temperature_C) {

    if (temperature_C < 0.0) return 0.13;
    if (temperature_C > 0.0 & temperature_C <= 5.0) return 0.13;
    if (temperature_C > 5.0 & temperature_C <= 10.0) return 0.1;
    if (temperature_C > 10.0 & temperature_C <= 15.0) return 0.07;
    if (temperature_C > 15.0 & temperature_C <= 20.0) return 0.04;
    if (temperature_C > 20.0 & temperature_C <= 25.0) return 0.03;
    if (temperature_C > 25.0 & temperature_C <= 30.0) return 0.01;
    if (temperature_C > 30.0 & temperature_C <= 35.0) return 0.0;
    if (temperature_C > 35.0 & temperature_C <= 40.0) return -0.01;
    if (temperature_C > 40.0 & temperature_C <= 50.0) return -0.02;
    if (temperature_C > 50.0) return -0.02;
    return 0.0;
}

/**
 * @brief corrects the temperature 
 * @param temperature_C
 * @return 
 */
float makeTemperatureCorrection(float temperature_C) {
    temperature_corrected_ec_high = adjustEcHighValue(temperature_C);
    temperature_corrected_ec_low = adjustEcLowValue(temperature_C);

}

/**
 * @brief mV of the probe
 * @param file_Descriptor
 * @return Voltage
 */
float get_Probe_mV(int i2c_Address, int i2c_Port) {
    //we have to set it to zero, regardless whats read in console param
    i2c_Port = 0;
    //and we have to the special configuration of wiringpi
    uint wired_Address = wiringPiI2CSetup(i2c_Address);
    //Loading phAdress - using wiringPi, so using special address range  
    int raw = wiringPiI2CReadReg16(wired_Address, i2c_Port);
    raw = raw >> 8 | ((raw << 8) &0xffff);
 
    //std::cout << raw << endl;
    //3.3 equals the voltage
    //Design Decision: 3.3V implementation   
    //4096 - 12bit in total 
    if (raw > 0) {
        return (((float) raw / 4096) * 3.3) * 1000;
    } else {
        return -1;
    }
}

/**
 * @brief converts celsius to Kelvin
 * @param temperature_C
 * @return Kelvin
 */
float convertCelsiusToKelvin(float temperature_C) {
    return temperature_C + 273.15;
}

/**
 * @brief converts Kelvin to degree celsius
 * @param temperature_K
 * @return 
 */
float convertKelvinToCelsius(float temperature_K) {
    return temperature_K - 273.15;
}

/**
 * @brief calculates the nernst equation
 * @param mean_measurements the mean of all mV measurements
 * @return result -> should be a valid ph_Value
 */
float calculateEC(float mean_measurements, float temperature_K) {
   
    //Defining Variables for temperature adjusting
    float ecLow_temperature_controlled = ec_low * (1 + adjustEcLowValue(ec_low));
    float ecHigh_temperature_controlled = ec_high * (1 + adjustEcHighValue(ec_high));    

    ecLow_temperature_controlled = ec_low; 
    ecHigh_temperature_controlled = ec_high;
    
    if (mean_measurements > ecHigh_temperature_controlled) {
        return 1413 + log10(ecHigh_temperature_controlled - mean_measurements);      
    }
    if (mean_measurements <= ecHigh_temperature_controlled && mean_measurements > ecLow_temperature_controlled) {

        return ((1413 - 84) / (ecHigh_temperature_controlled - ecLow_temperature_controlled))*(mean_measurements - ecLow_temperature_controlled) + 84;
    }
        
    if (mean_measurements < ecLow_temperature_controlled) {
        return 84 - log10(ecHigh_temperature_controlled - mean_measurements);
    }
    
    return 0;
}

/**
 * @brief gets the mean out of a number of measurements
 * @param measurement_size
 * @param ph_probe_Address
 * @return mean > 0 if enough data was collected, else -1 
 */
float getMeanMeasurements(int measurement_size, int i2c_Address, int i2c_Port) {
    float measurements[measurement_size];
    float mean_measurements = 0.0;
    float mV = 0.0;
    int valid_data = 0;



    for (int i = 0; i < measurement_size; i++) {
        //Gets mV from Probe    
        mV = get_Probe_mV(i2c_Address, i2c_Port);
        //if Probe Value is valid
        if (mV > 0) {
            //add to measurements array
            measurements[valid_data] = mV;
            valid_data += 1;
        }
    }
    //if one or more values are valid
    if (measurements[0] != 0) {
        for (int i = 0; i < valid_data; i++) {
            mean_measurements += measurements[i] / valid_data;
        }
        /* Debug: */
        // std::cout << "Mean: " << mean_measurements << endl 
        //        << "Valid Data Collected:" << valid_data << endl;
        return mean_measurements;
    }
    else {
        //if no valid data was found, return -1 for mean
        return -1;
    }
}

/**
 * @brief sends out post statement to specified server
 * @param argc
 * @param argv
 * @return 
 */
void writeXmlFile(string ec_Value, string temperature, string ec_Probe_Address, string measurement_mV, string deviceName) {
    cout << "<SymbioFilter>" << endl <<
            "<DeviceName>" << deviceName << "</DeviceName>" << endl <<
            "<EC>" << ec_Value << "</EC>" << endl <<
            "<ProbeAddress>" << ec_Probe_Address << "</ProbeAddress>" << endl <<
            "<Temperature_C>" << temperature << "</Temperature_C>" << endl <<
            "<Voltage_mV>" << measurement_mV << "</Voltage_mV>" << endl <<
            "</SymbioFilter>" << endl;
}

/**
 * @brief converts float to String
 * @param f float
 * @return string
 */
string floatToString(float f) {

    std::ostringstream ss;
    ss << f;
    std::string s(ss.str());
    return s;
}

/**
 * @brief converts integer to string
 * @param i
 * @return 
 */
string intToString(int i) {
    ostringstream convert;
    convert << i;
    return convert.str();
}

/**
 * @brief gets temperature in celsius
 * @param i temperature assigned 
 * @return temperature in celsius
 */
float getTemperatureCelsius(int i) {
    string line;
    string filename;
    string tmp;

    //Define Filename
    filename = temperature_Path;
    //Open File
    std::ifstream in(filename);

    //search for t=
    while (getline(in, line)) {
        tmp = "";
        if (strstr(line.c_str(), "t=")) {
            tmp = strstr(line.c_str(), "t=");
            tmp.erase(0, 2);
            if (tmp.length() > 1) {
                in.close();
                return strtof(tmp.c_str(), NULL) / 1000;
            }
        }
    }
    in.close();

    return -1000;
}

/**
 * @brief Main Function to work on server
 * @param argc argument count
 * @param argv argument - only one argument should be supplied: Address of probe
 * @return 
 */
int main(int argc, char *argv[]) {

    if (argc != 5) {
        std::cout << "Arguments: " << argc << endl;
        std::cout << "Error: You did not provide required arguments." << endl
                << "Usage: Aqualight-Conductivity TemperatureFile ECProbeI2CAddress EC(84µS) EC1278(µS)"
                << endl
                << "ECProbeI2CAddress has to be provided by integer number of port. E.g. Port:0x4d becomes 77."
                << endl
                << "EC Values normally come from database and are created by ec-calibrator"
                << endl;

        return 0;
    }
    //Initialize variables
    temperature_Path = argv[1];
    string str = argv[2];
    int pos = str.find_first_of(':');
    string i2c_FD = str.substr(0, pos);
    string ec_Address = str.substr(pos + 1);

    ec_low = stof(argv[3]);
    ec_high = stof(argv[4]);

    //Here is still some bug
    uint i2c_Address = stoi(i2c_FD); //0x4d;
    uint i2c_Port = stoi(ec_Address);
    //stoi(ph_Address);
    int ec_Probe_Address = i2c_Address;
    float mean;

    float temperature_C = getTemperatureCelsius(0);
    float temperature_K = convertCelsiusToKelvin(temperature_C);

    string ec_Value;
    string measurement_mV;
    string ec_Probe_Address_String;
    string deviceName = "Symbiofilter";

    //All temperature files
    //All Ph probes            
    if (i2c_Address >= 0) {
        //Setting Up Gpio
        wiringPiSetupGpio();
        //Access ph-Probe          
        ec_Probe_Address_String = intToString(i2c_Address);

        //Calculate Mean Value
        //mean = getMeanMeasurements(65535, ph_Probe_Address,i2c_Port);
        //Debug

        mean = getMeanMeasurements(255, i2c_Address, i2c_Port);
        if (mean > 0) {
            measurement_mV = floatToString(mean);
            ec_Value = floatToString(calculateEC(mean, temperature_K));
            //give all data to Console                              
            writeXmlFile(ec_Value, floatToString(temperature_C), ec_Probe_Address_String, measurement_mV, deviceName);
        }
        //return success        
        return 0;
    } else {
        std::cout << -1 << endl;
        //return fail
        return 1;
    }
}



