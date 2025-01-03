#ifndef LOGGER_TYPES_H
#define LOGGER_TYPES_H

#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <vector>

using namespace std;

class LoggerGoPoint {
public:
    LoggerGoPoint(const std::string& filename) 
        : filename(filename), start_time(std::chrono::steady_clock::now()) {
        // Open the file in write mode
        std::ofstream file(filename, std::ios::out);
        if (file.is_open()) {
            // Write the header row
            file << "time,X_d,X,Y_d,Y,Z_d,Z,Yaw_d,Yaw\n";
        }
    }
    
    ~LoggerGoPoint() {}

    void log(double X_d, double X, double Y_d, double Y, double Z_d, double Z, double Yaw_d, double Yaw) {
        using namespace std::chrono;
        auto current_time = steady_clock::now();
        auto elapsed = duration_cast<milliseconds>(current_time - start_time).count();
        
        // Calculate seconds and centiseconds
        int ss = static_cast<int>(elapsed / 1000);
        int cc = static_cast<int>((elapsed % 1000) / 10);
        
        // Format time and variables
        std::ostringstream ss_stream;
        ss_stream << std::setw(2) << std::setfill('0') << ss << '.'
                  << std::setw(2) << std::setfill('0') << cc;
        std::string label_time = ss_stream.str();
        
        std::ostringstream log_stream;
        log_stream << label_time << ','
                  << std::fixed << std::setprecision(6)
                  << X_d << ','
                  << X << ','
                  << Y_d << ','
                  << Y << ','
                  << Z_d << ','
                  << Z << ','
                  << Yaw_d << ','
                  << Yaw << '\n';
        std::string log_line = log_stream.str();
        
        // Write the log line to the file
        std::ofstream file(filename, std::ios::app);
        if (file.is_open()) {
            file << log_line;
        }
    }

private:
    std::string filename;
    std::chrono::steady_clock::time_point start_time;
};


#endif 