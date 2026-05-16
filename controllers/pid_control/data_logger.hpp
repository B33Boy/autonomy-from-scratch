#ifndef DATA_LOGGER_HPP
#define DATA_LOGGER_HPP

#include <fstream>
#include <string>
#include <filesystem>

struct PIDState
{
    double time;
    double setpoint;
    double measured;
    double err;
    double sum_err;
    double der;
    double output;
    double l_speed;
    double r_speed;
};

/**
 * TODO: Logger that registers webots sensors, stores them in a vector and runs them all in a single step.
 */
class DataLogger
{
public:
    explicit DataLogger(std::string filename) : file_(std::move(filename))
    {
        std::filesystem::create_directories("out_data");

        if (!file_.is_open())
        {
            throw std::runtime_error("Failed to open log file!");
        }

        file_ << "time,setpoint,measured,err,sum_err,der,output,l_speed,r_speed\n";
    }

    void log(PIDState const &entry)
    {
        file_ << entry.time << "," << entry.setpoint << "," << entry.measured << "," << entry.err << "," << entry.sum_err << "," << entry.der << "," << entry.output << "," << entry.l_speed << "," << entry.r_speed << "\n";
    }

private:
    std::ofstream file_;
};

#endif // DATA_LOGGER_HPP