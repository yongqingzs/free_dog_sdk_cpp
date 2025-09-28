#include <fdsc_utils/free_dog_sdk_h.hpp>
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

int main() {
    std::string settings = "LOW_WIRED_DEFAULTS";
    FDSC::UnitreeConnection conn(settings);
    conn.startRecv();

    FDSC::lowCmd lcmd;
    FDSC::lowState lstate;
    FDSC::MotorCmdArray mCmdArr;

    // Define joint positions for standing up and lying down
    double stand_up_joint_pos[12] = {0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
                                     0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763};
    double stand_down_joint_pos[12] = {0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,
                                       1.22187, -2.44375, -0.0473455, 1.22187, -2.44375};

    // Send an empty command to initialize the connection
    std::vector<uint8_t> cmd_bytes = lcmd.buildCmd(false);
    conn.send(cmd_bytes);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    double dt = 0.002;
    double runing_time = 0.0;
    double phase = 0.0;
    int motiontime = 0;

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
        motiontime++;
        runing_time += dt;

        // Get data
        std::vector<std::vector<uint8_t>> dataall;
        conn.getData(dataall);
        if (!dataall.empty()) {
            std::vector<uint8_t> data = dataall.back();
            lstate.parseData(data);
        }

        if (runing_time < 3.0) {
            // Stand up in first 3 seconds
            phase = tanh(runing_time / 1.2);
            for (int i = 0; i < 12; i++) {
                double target_pos = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i];
                double kp = phase * 50.0 + (1 - phase) * 20.0;
                double kd = 3.5;

                // Set motor command
                std::string motor_name = "FR_0"; // Need to map i to motor names
                switch (i) {
                    case 0: motor_name = "FR_0"; break;
                    case 1: motor_name = "FR_1"; break;
                    case 2: motor_name = "FR_2"; break;
                    case 3: motor_name = "FL_0"; break;
                    case 4: motor_name = "FL_1"; break;
                    case 5: motor_name = "FL_2"; break;
                    case 6: motor_name = "RR_0"; break;
                    case 7: motor_name = "RR_1"; break;
                    case 8: motor_name = "RR_2"; break;
                    case 9: motor_name = "RL_0"; break;
                    case 10: motor_name = "RL_1"; break;
                    case 11: motor_name = "RL_2"; break;
                }

                std::vector<float> joint_cmd{static_cast<float>(target_pos), 0.0f, 0.0f, static_cast<float>(kp), static_cast<float>(kd)};
                mCmdArr.setMotorCmd(motor_name, FDSC::MotorModeLow::Servo, joint_cmd);
            }
            lcmd.motorCmd = mCmdArr;
        } else {
            // Maintain standing position
            for (int i = 0; i < 12; i++) {
                double target_pos = stand_up_joint_pos[i];
                double kp = 50.0;
                double kd = 3.5;

                std::string motor_name = "FR_0"; // Need to map i to motor names
                switch (i) {
                    case 0: motor_name = "FR_0"; break;
                    case 1: motor_name = "FR_1"; break;
                    case 2: motor_name = "FR_2"; break;
                    case 3: motor_name = "FL_0"; break;
                    case 4: motor_name = "FL_1"; break;
                    case 5: motor_name = "FL_2"; break;
                    case 6: motor_name = "RR_0"; break;
                    case 7: motor_name = "RR_1"; break;
                    case 8: motor_name = "RR_2"; break;
                    case 9: motor_name = "RL_0"; break;
                    case 10: motor_name = "RL_1"; break;
                    case 11: motor_name = "RL_2"; break;
                }

                std::vector<float> joint_cmd{static_cast<float>(target_pos), 0.0f, 0.0f, static_cast<float>(kp), static_cast<float>(kd)};
                mCmdArr.setMotorCmd(motor_name, FDSC::MotorModeLow::Servo, joint_cmd);
            }
            lcmd.motorCmd = mCmdArr;
        }

        // Send command
        std::vector<uint8_t> cmdBytes = lcmd.buildCmd(false);
        conn.send(cmdBytes);

        if (motiontime % 500 == 0) {
            std::cout << "Running time: " << runing_time << " Phase: " << phase << std::endl;
        }
    }

    return 0;
}