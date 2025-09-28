/*
C.A.R.E. CAN Interface Node
Чистый мост между CAN шиной и ROS2 топиками
Без логики - только протокол!

Функции:
- Читает CAN сообщения от ESP32/STM32 контроллеров
- Преобразует в ROS2 сообщения care_common
- Публикует данные целей и статус устройств
- Подписывается на команды безопасности от care_safety_controller
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "care_common/msg/radar_target.hpp"
#include "care_common/msg/radar_targets.hpp"
#include "care_common/msg/system_status.hpp"
#include "care_common/msg/device_info.hpp"
#include "care_common/msg/safety_command.hpp"

using namespace std::chrono_literals;

class CareCanInterfaceNode : public rclcpp::Node
{
public:
    CareCanInterfaceNode() : Node("care_can_interface_node"), can_socket_(-1)
    {
        // Параметры
        this->declare_parameter("can_interface", "can0");
        this->declare_parameter("can_bitrate", 500000);
        this->declare_parameter("max_devices", 15);
        this->declare_parameter("target_timeout_ms", 500);
        this->declare_parameter("status_timeout_ms", 2000);
        
        can_interface_ = this->get_parameter("can_interface").as_string();
        max_devices_ = this->get_parameter("max_devices").as_int();
        target_timeout_ = std::chrono::milliseconds(this->get_parameter("target_timeout_ms").as_int());
        status_timeout_ = std::chrono::milliseconds(this->get_parameter("status_timeout_ms").as_int());
        
        // Publishers - отдельный топик для каждого устройства
        for (int device_id = 1; device_id <= max_devices_; device_id++) {
            auto targets_pub = this->create_publisher<care_common::msg::RadarTargets>(
                "/care/device_" + std::to_string(device_id) + "/targets", 10);
            targets_publishers_[device_id] = targets_pub;
            
            auto status_pub = this->create_publisher<care_common::msg::DeviceInfo>(
                "/care/device_" + std::to_string(device_id) + "/status", 10);
            status_publishers_[device_id] = status_pub;
        }
        
        // Subscriber для команд безопасности
        safety_command_sub_ = this->create_subscription<care_common::msg::SafetyCommand>(
            "/care/safety_commands", 10,
            std::bind(&CareCanInterfaceNode::safety_command_callback, this, std::placeholders::_1));
        
        // Инициализация CAN интерфейса
        if (initialize_can()) {
            RCLCPP_INFO(this->get_logger(), "🚌 CAN Interface initialized on %s", can_interface_.c_str());
            
            // Таймер для чтения CAN сообщений
            can_timer_ = this->create_wall_timer(
                1ms, std::bind(&CareCanInterfaceNode::can_read_callback, this));
            
            // Таймер для проверки таймаутов
            timeout_timer_ = this->create_wall_timer(
                100ms, std::bind(&CareCanInterfaceNode::check_timeouts, this));
                
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to initialize CAN interface %s", can_interface_.c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "🔌 C.A.R.E. CAN Interface Node started");
    }
    
    ~CareCanInterfaceNode()
    {
        if (can_socket_ >= 0) {
            close(can_socket_);
        }
    }

private:
    // CAN Protocol Constants (из спецификации)
    static constexpr uint32_t CAN_TARGET_DATA_BASE_ID = 0x200;
    static constexpr uint32_t CAN_STATUS_BASE_ID = 0x300;
    static constexpr uint32_t CAN_SLOWDOWN_COMMAND_ID = 0x100;    // Команда замедления
    static constexpr uint32_t CAN_EMERGENCY_STOP_ID = 0x101;      // Команда экстренной остановки
    
    // Структуры данных (соответствуют CAN протоколу)
    struct CanTargetMessage {
        int16_t x;          // X coordinate in mm
        int16_t y;          // Y coordinate in mm  
        uint16_t distance;  // Distance in mm
        int16_t speed;      // Speed in cm/s
    } __attribute__((packed));
    
    struct CanStatusMessage {
        uint8_t device_id;      // Device identifier
        uint8_t active_targets; // Number of active targets
        uint8_t system_status;  // System status flags
        uint8_t battery_level;  // Battery level
    } __attribute__((packed));
    
    struct CanSlowdownMessage {
        uint8_t command;        // Slowdown command
        uint8_t source_id;      // Source device ID
        uint8_t speed_limit;    // Speed limit (0-100%)
        uint8_t duration;       // Duration in seconds (0=indefinite)
    } __attribute__((packed));
    
    struct CanEmergencyMessage {
        uint8_t command;        // Emergency command
        uint8_t source_id;      // Source device ID
    } __attribute__((packed));
    
    // Состояние устройств
    struct DeviceState {
        std::map<uint8_t, care_common::msg::RadarTarget> targets; // target_id -> target
        care_common::msg::DeviceInfo device_info;
        std::chrono::steady_clock::time_point last_target_time;
        std::chrono::steady_clock::time_point last_status_time;
        bool online = false;
    };
    
    bool initialize_can()
    {
        // Создание CAN сокета
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error creating CAN socket");
            return false;
        }
        
        // Настройка интерфейса
        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, can_interface_.c_str());
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error getting CAN interface index for %s", can_interface_.c_str());
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }
        
        // Привязка к интерфейсу
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        
        if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error binding CAN socket");
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }
        
        // Неблокирующий режим
        int flags = fcntl(can_socket_, F_GETFL, 0);
        fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);
        
        return true;
    }
    
    void can_read_callback()
    {
        struct can_frame frame;
        ssize_t nbytes;
        
        // Читаем все доступные сообщения
        while ((nbytes = read(can_socket_, &frame, sizeof(struct can_frame))) > 0) {
            if (nbytes == sizeof(struct can_frame)) {
                process_can_message(frame);
            }
        }
    }
    
    void process_can_message(const struct can_frame& frame)
    {
        uint32_t can_id = frame.can_id;
        
        // Определяем тип сообщения по ID
        if (can_id >= CAN_TARGET_DATA_BASE_ID && can_id < CAN_STATUS_BASE_ID) {
            process_target_message(frame);
        } else if (can_id >= CAN_STATUS_BASE_ID && can_id < CAN_STATUS_BASE_ID + 0x100) {
            process_status_message(frame);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Unknown CAN message ID: 0x%03X", can_id);
        }
    }
    
    void process_target_message(const struct can_frame& frame)
    {
        if (frame.can_dlc != 8) {
            RCLCPP_WARN(this->get_logger(), "Invalid target message length: %d", frame.can_dlc);
            return;
        }
        
        // Декодируем ID устройства и цели из CAN ID
        uint32_t can_id = frame.can_id;
        uint8_t device_id = ((can_id - CAN_TARGET_DATA_BASE_ID) / 10) + 1;
        uint8_t target_id = (can_id - CAN_TARGET_DATA_BASE_ID) % 10;
        
        if (device_id > max_devices_ || target_id > 2) {
            RCLCPP_WARN(this->get_logger(), "Invalid device_id=%d or target_id=%d", device_id, target_id);
            return;
        }
        
        // Распаковка данных (big endian)
        CanTargetMessage can_msg;
        can_msg.x = (frame.data[0] << 8) | frame.data[1];
        can_msg.y = (frame.data[2] << 8) | frame.data[3];
        can_msg.distance = (frame.data[4] << 8) | frame.data[5];
        can_msg.speed = (frame.data[6] << 8) | frame.data[7];
        
        // Создание ROS2 сообщения
        care_common::msg::RadarTarget target;
        target.id = target_id;
        target.device_id = device_id;
        target.x = can_msg.x;
        target.y = can_msg.y;
        target.distance = can_msg.distance;
        target.speed = can_msg.speed;
        target.angle = std::atan2(can_msg.y, can_msg.x) * 180.0 / M_PI;
        target.valid = true;
        target.timestamp = this->get_clock()->now();
        target.last_seen = target.timestamp;
        target.confidence = 1.0;
        target.tracking_age = 1;
        target.in_safety_zone = false; // Будет вычислено в safety controller
        target.safety_distance = 0.0;
        
        // Обновление состояния устройства
        auto& device_state = device_states_[device_id];
        device_state.targets[target_id] = target;
        device_state.last_target_time = std::chrono::steady_clock::now();
        device_state.online = true;
        
        // Публикация обновленного массива целей
        publish_targets_for_device(device_id);
        
        RCLCPP_DEBUG(this->get_logger(), "📡 Target from device %d: x=%d, y=%d, dist=%d", 
                    device_id, can_msg.x, can_msg.y, can_msg.distance);
    }
    
    void process_status_message(const struct can_frame& frame)
    {
        if (frame.can_dlc != 4) {
            RCLCPP_WARN(this->get_logger(), "Invalid status message length: %d", frame.can_dlc);
            return;
        }
        
        // Декодируем ID устройства
        uint32_t can_id = frame.can_id;
        uint8_t device_id = ((can_id - CAN_STATUS_BASE_ID) / 10) + 1;
        
        if (device_id > max_devices_) {
            RCLCPP_WARN(this->get_logger(), "Invalid device_id=%d in status", device_id);
            return;
        }
        
        // Распаковка статуса
        CanStatusMessage status_msg;
        status_msg.device_id = frame.data[0];
        status_msg.active_targets = frame.data[1];
        status_msg.system_status = frame.data[2];
        status_msg.battery_level = frame.data[3];
        
        // Обновление информации об устройстве
        auto& device_state = device_states_[device_id];
        auto& device_info = device_state.device_info;
        
        device_info.device_id = device_id;
        device_info.device_name = "CARE_Device_" + std::to_string(device_id);
        device_info.device_type = "ESP32/STM32"; // Определяется автоматически
        device_info.communication_status = (status_msg.system_status == 0) ? 
            care_common::msg::DeviceInfo::STATUS_ONLINE : 
            care_common::msg::DeviceInfo::STATUS_WARNING;
        device_info.last_seen = this->get_clock()->now();
        device_info.active_targets = status_msg.active_targets;
        device_info.battery_level = status_msg.battery_level;
        
        device_state.last_status_time = std::chrono::steady_clock::now();
        device_state.online = true;
        
        // Публикация статуса устройства
        if (status_publishers_.count(device_id)) {
            status_publishers_[device_id]->publish(device_info);
        }
        
        RCLCPP_DEBUG(this->get_logger(), "📊 Status from device %d: targets=%d, status=%d, battery=%d%%", 
                    device_id, status_msg.active_targets, status_msg.system_status, status_msg.battery_level);
    }
    
    void publish_targets_for_device(uint8_t device_id)
    {
        if (!targets_publishers_.count(device_id)) return;
        
        auto& device_state = device_states_[device_id];
        
        care_common::msg::RadarTargets targets_msg;
        targets_msg.header.stamp = this->get_clock()->now();
        targets_msg.header.frame_id = "radar_link_" + std::to_string(device_id);
        targets_msg.device_id = device_id;
        
        // Собираем все активные цели
        for (const auto& [target_id, target] : device_state.targets) {
            targets_msg.targets.push_back(target);
        }
        
        targets_msg.target_count = targets_msg.targets.size();
        targets_msg.device_status = 0; // OK
        targets_msg.radar_status = 0;  // OK
        targets_msg.update_rate = 10.0; // Предполагаемая частота
        
        // Вычисляем статистику безопасности
        targets_msg.targets_in_safety_zone = 0;
        targets_msg.emergency_detected = false;
        targets_msg.closest_distance = 10000.0; // Большое значение
        
        for (const auto& target : targets_msg.targets) {
            if (target.distance < targets_msg.closest_distance) {
                targets_msg.closest_distance = target.distance;
            }
        }
        
        targets_publishers_[device_id]->publish(targets_msg);
    }
    
    void check_timeouts()
    {
        auto now = std::chrono::steady_clock::now();
        
        for (auto& [device_id, device_state] : device_states_) {
            // Проверка таймаута целей
            if (device_state.online && 
                (now - device_state.last_target_time) > target_timeout_) {
                // Очищаем устаревшие цели
                device_state.targets.clear();
                publish_targets_for_device(device_id);
            }
            
            // Проверка таймаута статуса
            if (device_state.online && 
                (now - device_state.last_status_time) > status_timeout_) {
                device_state.online = false;
                device_state.device_info.communication_status = care_common::msg::DeviceInfo::STATUS_OFFLINE;
                
                if (status_publishers_.count(device_id)) {
                    status_publishers_[device_id]->publish(device_state.device_info);
                }
                
                RCLCPP_WARN(this->get_logger(), "⚠️ Device %d went offline (status timeout)", device_id);
            }
        }
    }
    
    void safety_command_callback(const care_common::msg::SafetyCommand::SharedPtr msg)
    {
        struct can_frame frame;
        
        switch (msg->command_type) {
            case care_common::msg::SafetyCommand::COMMAND_SLOWDOWN: {
                // Команда замедления
                frame.can_id = CAN_SLOWDOWN_COMMAND_ID;
                frame.can_dlc = 4;
                
                CanSlowdownMessage slowdown_msg;
                slowdown_msg.command = 0x01; // SLOWDOWN_ACTIVATE
                slowdown_msg.source_id = 0;  // ROS2 system
                slowdown_msg.speed_limit = 50; // 50% скорости (можно сделать параметром)
                slowdown_msg.duration = 0;     // Indefinite
                
                std::memcpy(frame.data, &slowdown_msg, sizeof(slowdown_msg));
                
                if (write(can_socket_, &frame, sizeof(struct can_frame)) == sizeof(struct can_frame)) {
                    RCLCPP_INFO(this->get_logger(), "⚠️ Slowdown command sent: limit=%d%%, target=%d", 
                               slowdown_msg.speed_limit, msg->target_device_id);
                }
                break;
            }
            
            case care_common::msg::SafetyCommand::COMMAND_CLEAR_SLOWDOWN: {
                // Отмена замедления
                frame.can_id = CAN_SLOWDOWN_COMMAND_ID;
                frame.can_dlc = 4;
                
                CanSlowdownMessage slowdown_msg;
                slowdown_msg.command = 0x00; // SLOWDOWN_CLEAR
                slowdown_msg.source_id = 0;
                slowdown_msg.speed_limit = 100;
                slowdown_msg.duration = 0;
                
                std::memcpy(frame.data, &slowdown_msg, sizeof(slowdown_msg));
                
                if (write(can_socket_, &frame, sizeof(struct can_frame)) == sizeof(struct can_frame)) {
                    RCLCPP_INFO(this->get_logger(), "✅ Slowdown cleared for target=%d", msg->target_device_id);
                }
                break;
            }
            
            case care_common::msg::SafetyCommand::COMMAND_STOP: {
                // Экстренная остановка
                frame.can_id = CAN_EMERGENCY_STOP_ID;
                frame.can_dlc = 2;
                
                CanEmergencyMessage emergency_msg;
                emergency_msg.command = 0x01; // EMERGENCY_STOP
                emergency_msg.source_id = 0;  // ROS2 system
                
                std::memcpy(frame.data, &emergency_msg, sizeof(emergency_msg));
                
                if (write(can_socket_, &frame, sizeof(struct can_frame)) == sizeof(struct can_frame)) {
                    RCLCPP_ERROR(this->get_logger(), "🚨 EMERGENCY STOP sent to target=%d", msg->target_device_id);
                }
                break;
            }
            
            case care_common::msg::SafetyCommand::COMMAND_RESUME: {
                // Возобновление работы
                frame.can_id = CAN_EMERGENCY_STOP_ID;
                frame.can_dlc = 2;
                
                CanEmergencyMessage emergency_msg;
                emergency_msg.command = 0x00; // EMERGENCY_CLEAR
                emergency_msg.source_id = 0;
                
                std::memcpy(frame.data, &emergency_msg, sizeof(emergency_msg));
                
                if (write(can_socket_, &frame, sizeof(struct can_frame)) == sizeof(struct can_frame)) {
                    RCLCPP_INFO(this->get_logger(), "✅ Emergency cleared for target=%d", msg->target_device_id);
                }
                break;
            }
            
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown safety command type: %d", msg->command_type);
                return;
        }
        
        // Проверка ошибки отправки
        if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to send safety command type=%d", msg->command_type);
        }
    }
    
    // Параметры
    std::string can_interface_;
    int max_devices_;
    std::chrono::milliseconds target_timeout_;
    std::chrono::milliseconds status_timeout_;
    
    // CAN интерфейс
    int can_socket_;
    
    // ROS2 интерфейсы
    std::map<uint8_t, rclcpp::Publisher<care_common::msg::RadarTargets>::SharedPtr> targets_publishers_;
    std::map<uint8_t, rclcpp::Publisher<care_common::msg::DeviceInfo>::SharedPtr> status_publishers_;
    rclcpp::Subscription<care_common::msg::SafetyCommand>::SharedPtr safety_command_sub_;
    
    // Таймеры
    rclcpp::TimerBase::SharedPtr can_timer_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    
    // Состояние устройств
    std::map<uint8_t, DeviceState> device_states_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CareCanInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
