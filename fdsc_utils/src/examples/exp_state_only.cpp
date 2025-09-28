#include <fdsc_utils/free_dog_sdk_h.hpp>
#include <iostream>
#include <thread>
#include <chrono>

void show_info(const FDSC::lowState &lstate) {
    std::cout << "=============================" << std::endl;
    std::cout << "------------------HardSoft Version info: -------------------" << std::endl;
    std::cout << "SN: " << "\t";
    FDSC::byte_print(lstate.SN, false);
    std::cout << FDSC::decode_sn(lstate.SN) << std::endl;
    std::cout << "Version: " << "\t";
    FDSC::byte_print(lstate.version, false);
    std::cout << FDSC::decode_version(lstate.version) << std::endl;
    std::cout << "Bandwidth: " << "\t" << FDSC::hex_to_kp_kd(lstate.bandWidth) << std::endl;
    std::cout << "------------------IMU info: -------------------" << std::endl;
    std::cout << "IMU Quaternion: ";
    FDSC::pretty_show_vector(lstate.imu_quaternion);
    std::cout << std::endl;
    std::cout << "IMU Gyroscope: ";
    FDSC::pretty_show_vector(lstate.imu_gyroscope);
    std::cout << std::endl;
    std::cout << "IMU Accelerometer: ";
    FDSC::pretty_show_vector(lstate.imu_accelerometer);
    std::cout << std::endl;
    std::cout << "IMU RPY: ";
    FDSC::pretty_show_vector(lstate.imu_rpy);
    std::cout << "IMU Temperature: " << float(lstate.temperature_imu) << std::endl;
    std::cout << "------------------Joint info: -------------------" << std::endl;
    
    // Joint name mapping (similar to Python version)
    std::vector<std::string> joint_names = {
        "FR_0", "FR_1", "FR_2",
        "FL_0", "FL_1", "FL_2", 
        "RR_0", "RR_1", "RR_2",
        "RL_0", "RL_1", "RL_2"
    };
    
    for (int i = 0; i < 12; i++) {
        std::cout << "MotorState " << joint_names[i] << " MODE:\t\t" << lstate.motorState[i].mode << std::endl;
        std::cout << "MotorState " << joint_names[i] << " q:\t\t" << lstate.motorState[i].q << std::endl;
        std::cout << "MotorState " << joint_names[i] << " dq:\t\t" << lstate.motorState[i].dq << std::endl;
    }
    
    std::cout << "------------------BMS info: -------------------" << std::endl;
    std::cout << "SOC: " << int(lstate.SOC) << std::endl;
    std::cout << "Current: " << lstate.current << "mA" << std::endl;
    std::cout << "Cycles: " << lstate.cycle << std::endl;
    std::cout << "Temps BQ: " << lstate.BQ_NTC[0] << "°C, " << lstate.BQ_NTC[1] << "°C" << std::endl;
    std::cout << "Temps MCU: " << lstate.MCU_NTC[0] << "°C, " << lstate.MCU_NTC[1] << "°C" << std::endl;
    std::cout << "------------------Foot Force info: -------------------" << std::endl;
    std::cout << "Footforce: ";
    for (auto data_f : lstate.footForce) {
        std::cout << float(data_f) << " ";
    }
    std::cout << std::endl;
    std::cout << "FootforceEst: ";
    for (auto data_f : lstate.footForceEst) {
        std::cout << float(data_f) << " ";
    }
    std::cout << std::endl;
    std::cout << "=============================" << std::endl;
}

int main() {
    std::string settings = "LOW_WIRED_DEFAULTS";  // Using wired defaults, can change to WIFI if needed
    FDSC::UnitreeConnection conn(settings);
    conn.startRecv();

    FDSC::lowCmd lcmd;
    FDSC::lowState lstate;

    // Send an empty command to tell the dog the receive port and initialize the connection
    std::vector<uint8_t> cmd_bytes = lcmd.buildCmd(false);
    conn.send(cmd_bytes);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Some time to collect packets

    int motiontime = 0;
    bool show_data = true;

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        motiontime++;

        std::vector<std::vector<uint8_t>> dataall;
        conn.getData(dataall);

        if (!dataall.empty()) {
            std::vector<uint8_t> data = dataall.back();
            lstate.parseData(data);

            if (show_data && motiontime % 100 == 0) {  // Print every 100 cycles
                std::cout << "receive pkg len: " << dataall.size() << std::endl;
                std::cout << "loop: " << motiontime << " datalen: " << std::dec << data.size() << std::endl;
                show_info(lstate);
            }
        } else {
            std::cout << "motiontime: " << motiontime << " datasize: " << dataall.size() << std::endl;
        }

        // Send empty command to maintain connection (no motor control)
        cmd_bytes = lcmd.buildCmd(false);
        conn.send(cmd_bytes);
    }

    return 0;
}