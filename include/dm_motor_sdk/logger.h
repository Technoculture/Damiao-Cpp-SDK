#ifndef DM_MOTOR_SDK_LOGGER_H
#define DM_MOTOR_SDK_LOGGER_H

#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <memory>

namespace dm_motor_sdk {

class Logger {
public:
    static void initialize(const std::string& logger_name = "robot", const std::string& file_path = "logs/robot.log") {
        if (spdlog::get(logger_name) == nullptr) {
            // Rotating logs: 5MB x 3 files
            auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                file_path, 5 * 1024 * 1024, 3
            );

            auto logger = std::make_shared<spdlog::logger>(logger_name, file_sink);
            logger->set_level(spdlog::level::debug);
            logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");
            spdlog::register_logger(logger);
        }
    }

    static std::shared_ptr<spdlog::logger> get(const std::string& logger_name = "robot") {
        return spdlog::get(logger_name);
    }
};

} // namespace dm_motor_sdk

#endif // DM_MOTOR_SDK_LOGGER_H