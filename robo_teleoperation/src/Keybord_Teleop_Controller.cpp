
#include "robo_teleoperation/Keybord_Teleop_Controller.hpp"

namespace robo_teleoperation {

    Keybord_Teleop_Controller::Keybord_Teleop_Controller() : rclcpp::Node("Keyboard_Teleop_Interface") {

        teleop_publisher = this->create_publisher<robot_teleoperation_interface::msg::Teleop>("teleop_command_jog", 10);

    }


    Keybord_Teleop_Controller::~Keybord_Teleop_Controller() {

    }

    int Keybord_Teleop_Controller::printSelectMenue() {
        RCLCPP_INFO(logger, "Select Movement Mode: 1. Joint Space 2. World Frame 3. Tool Frame");
        try {
            std::cin >> mode;
        } catch (...) {
            RCLCPP_INFO(logger, "Invalid Input");
            mode = 0;
            // Clean Input Buffer
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
        return mode;

    }

    char Keybord_Teleop_Controller::getKey() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
            perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
            perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
            perror("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
            perror("tcsetattr ~ICANON");
        return (buf);
    }

    void Keybord_Teleop_Controller::controllLoop() {

        // TODO: read the Key from the Keyboard and call the corresponding function
        // Default is Joint Space
        mode = 0;
        // Print Message of Inputmaps
        RCLCPP_INFO(logger, "Use the following Keys to control the Robot: ");
        RCLCPP_INFO(logger, "w: Increase Joint 1, s: Decrease Joint 1");
        RCLCPP_INFO(logger, "a: Increase Joint 2, d: Decrease Joint 2");
        RCLCPP_INFO(logger, "q: Increase Joint 3, e: Decrease Joint 3");
        RCLCPP_INFO(logger, "z: Increase Joint 4, x: Decrease Joint 4");
        RCLCPP_INFO(logger, "c: Increase Joint 5, v: Decrease Joint 5");
        RCLCPP_INFO(logger, "b: Increase Joint 6, n: Decrease Joint 6");
        RCLCPP_INFO(logger, "1: Joint Space, 2: World Frame, 3: Tool Frame");
        RCLCPP_INFO(logger, "4: Increase Speed, 5: Decrease Speed");
        RCLCPP_INFO(logger, "6: Open Close Tool");
        RCLCPP_INFO(logger, "Press ^c to Exit");

        while (rclcpp::ok()) {
            char key = getKey();
            if (keyMap.find(key) != keyMap.end()) {
                // Joint Space
                // Move Joint
                switch (key) {
                    case 'w':
                        axis_values[0] += 0.5;
                        break;
                    case 's':
                        axis_values[0] -= 0.1;
                        break;
                    case 'a':
                        axis_values[1] += 0.1;
                        break;
                    case 'd':
                        axis_values[1] -= 0.1;
                        break;
                    case 'q':
                        axis_values[2] += 0.1;
                        break;
                    case 'e':
                        axis_values[2] -= 0.1;
                        break;
                    case 'z':
                        axis_values[3] += 0.1;
                        break;
                    case 'x':
                        axis_values[3] -= 0.1;
                        break;
                    case 'c':
                        axis_values[4] += 0.1;
                        break;
                    case 'v':
                        axis_values[4] -= 0.1;
                        break;
                    case 'b':
                        axis_values[5] += 0.1;
                        break;
                    case 'n':
                        axis_values[5] -= 0.1;
                        break;
                    default:
                        break;

                }

            } else if (optionMap.find(key) != optionMap.end()) {
                switch (optionMap[key]) {
                    case 1:
                        mode = 0;
                        break;
                    case 2:
                        mode = 1;
                        break;
                    case 3:
                        mode = 2;

                        break;
                    case 4:
                        // Increase Speed
                        speed += 10;
                        break;
                    case 5:
                        // Decrease Speed
                        speed -= 10;
                        if (speed < 0)
                            speed = 5;
                        break;
                    case 6:
                        // Open Close Tool
                        tool_open = !tool_open;
                        break;
                    default:
                        break;
                }

            }
            // Exit the Programm when ^c is pressed
            if (key == 3) {
                break;
            }

            // Send the new Joint Values to the Backend
            robot_teleoperation_interface::msg::Teleop msg;
            // Tool
            msg.gpios.push_back(tool_open);
            // Speed
            msg.speed = speed;
            // Joint Values
            msg.data_values = axis_values;
            // Frame
            msg.frame = mode;

            // Publish the new Message
            teleop_publisher->publish(msg);

        }

    }

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_unique<robo_teleoperation::Keybord_Teleop_Controller>();
    node->controllLoop();
    rclcpp::shutdown();
    return 0;
}
