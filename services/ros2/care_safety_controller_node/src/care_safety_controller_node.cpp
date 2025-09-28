/*
C.A.R.E. Safety Controller Node
Централизованная система принятия решений о безопасности

Функции:
- Получает данные от всех источников (CAN, Demo, UART)
- Анализирует зоны безопасности
- Принимает решения об экстренной остановке
- Отправляет команды безопасности контроллерам
- Ведет логи событий безопасности
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "care_common/msg/radar_target.hpp"
#include "care_common/msg/radar_targets.hpp"
#include "care_common/msg/system_status.hpp"
#include "care_common/msg/safety_zone.hpp"
#include "care_common/msg/safety_command.hpp"
#include "care_common/srv/set_safety_zone.hpp"
#include "care_common/srv/emergency_stop.hpp"
#include "care_common/srv/get_system_status.hpp"

using namespace std::chrono_literals;

class CareSafetyControllerNode : public rclcpp::Node
{
public:
    CareSafetyControllerNode() : Node("care_safety_controller_node"), emergency_active_(false), command_sequence_(0)
    {
        // Параметры
        this->declare_parameter("max_devices", 15);
        this->declare_parameter("safety_check_rate", 20.0); // 20 Hz
        this->declare_parameter("default_safety_distance", 300.0); // mm
        this->declare_parameter("default_safety_angle", 60.0); // degrees
        this->declare_parameter("emergency_hysteresis", 50.0); // mm
        this->declare_parameter("min_trigger_time", 0.1); // seconds
        this->declare_parameter("log_events", true);
        this->declare_parameter("log_file", "/tmp/care_safety.log");
        
        max_devices_ = this->get_parameter("max_devices").as_int();
        safety_check_rate_ = this->get_parameter("safety_check_rate").as_double();
        default_safety_distance_ = this->get_parameter("default_safety_distance").as_double();
        default_safety_angle_ = this->get_parameter("default_safety_angle").as_double();
        emergency_hysteresis_ = this->get_parameter("emergency_hysteresis").as_double();
        min_trigger_time_ = std::chrono::duration<double>(this->get_parameter("min_trigger_time").as_double());
        log_events_ = this->get_parameter("log_events").as_bool();
        log_file_ = this->get_parameter("log_file").as_string();
        
        // Инициализация зоны безопасности по умолчанию
        initialize_default_safety_zone();
        
        // Publishers
        safety_command_pub_ = this->create_publisher<care_common::msg::SafetyCommand>(
            "/care/safety_commands", 10);
        system_status_pub_ = this->create_publisher<care_common::msg::SystemStatus>(
            "/care/system_status", 10);
        
        // Subscribers - подписываемся на все возможные источники данных
        for (int device_id = 1; device_id <= max_devices_; device_id++) {
            auto targets_sub = this->create_subscription<care_common::msg::RadarTargets>(
                "/care/device_" + std::to_string(device_id) + "/targets", 10,
                [this, device_id](const care_common::msg::RadarTargets::SharedPtr msg) {
                    this->targets_callback(msg, device_id);
                });
            targets_subscribers_[device_id] = targets_sub;
        }
        
        // Подписка на демо данные
        demo_targets_sub_ = this->create_subscription<care_common::msg::RadarTargets>(
            "/care/demo/targets", 10,
            std::bind(&CareSafetyControllerNode::demo_targets_callback, this, std::placeholders::_1));
        
        // Services
        set_safety_zone_srv_ = this->create_service<care_common::srv::SetSafetyZone>(
            "/care/set_safety_zone",
            std::bind(&CareSafetyControllerNode::set_safety_zone_callback, this,
                std::placeholders::_1, std::placeholders::_2));
        
        emergency_stop_srv_ = this->create_service<care_common::srv::EmergencyStop>(
            "/care/emergency_stop",
            std::bind(&CareSafetyControllerNode::emergency_stop_callback, this,
                std::placeholders::_1, std::placeholders::_2));
        
        get_system_status_srv_ = this->create_service<care_common::srv::GetSystemStatus>(
            "/care/get_system_status",
            std::bind(&CareSafetyControllerNode::get_system_status_callback, this,
                std::placeholders::_1, std::placeholders::_2));
        
        // Таймер для периодической проверки безопасности
        safety_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / safety_check_rate_),
            std::bind(&CareSafetyControllerNode::safety_check_callback, this));
        
        // Таймер для публикации статуса системы
        status_timer_ = this->create_wall_timer(
            1s, std::bind(&CareSafetyControllerNode::publish_system_status, this));
        
        // Открытие файла логов
        if (log_events_) {
            log_stream_.open(log_file_, std::ios::app);
            log_event("SYSTEM_START", "Safety Controller started");
        }
        
        RCLCPP_INFO(this->get_logger(), "🛡️ C.A.R.E. Safety Controller started");
        RCLCPP_INFO(this->get_logger(), "📊 Safety check rate: %.1f Hz", safety_check_rate_);
        RCLCPP_INFO(this->get_logger(), "⚠️ Default safety distance: %.0f mm", default_safety_distance_);
    }
    
    ~CareSafetyControllerNode()
    {
        if (log_events_ && log_stream_.is_open()) {
            log_event("SYSTEM_STOP", "Safety Controller stopped");
            log_stream_.close();
        }
    }

private:
    void initialize_default_safety_zone()
    {
        default_safety_zone_.header.stamp = this->get_clock()->now();
        default_safety_zone_.header.frame_id = "base_link";
        
        default_safety_zone_.min_distance = default_safety_distance_;
        default_safety_zone_.max_distance = 6000.0; // 6m max range
        default_safety_zone_.horizontal_angle = default_safety_angle_;
        default_safety_zone_.vertical_angle = 35.0; // LD2450 vertical FoV
        
        // Две зоны безопасности
        default_safety_zone_.slowdown_distance = default_safety_distance_ + 200.0;  // Зона замедления
        default_safety_zone_.critical_distance = default_safety_distance_;          // Зона экстренной остановки
        
        // Центр и ориентация зоны
        default_safety_zone_.center.x = 0.0;
        default_safety_zone_.center.y = 0.0; 
        default_safety_zone_.center.z = 0.0;
        default_safety_zone_.orientation.w = 1.0;
        default_safety_zone_.orientation.x = 0.0;
        default_safety_zone_.orientation.y = 0.0;
        default_safety_zone_.orientation.z = 0.0;
        
        // Настройки зоны
        default_safety_zone_.zone_active = true;
        default_safety_zone_.zone_id = 1;
        default_safety_zone_.zone_name = "Primary Dual Safety Zone";
        default_safety_zone_.priority = 255; // Highest priority
        default_safety_zone_.hysteresis = emergency_hysteresis_;
    }
    
    void targets_callback(const care_common::msg::RadarTargets::SharedPtr msg, uint8_t device_id)
    {
        // Обновляем данные целей для устройства
        device_targets_[device_id] = *msg;
        device_last_update_[device_id] = std::chrono::steady_clock::now();
        
        RCLCPP_DEBUG(this->get_logger(), "📡 Received %d targets from device %d", 
                    msg->target_count, device_id);
    }
    
    void demo_targets_callback(const care_common::msg::RadarTargets::SharedPtr msg)
    {
        // Обрабатываем демо данные как устройство 0
        demo_targets_ = *msg;
        demo_last_update_ = std::chrono::steady_clock::now();
        
        RCLCPP_DEBUG(this->get_logger(), "🎭 Received %d demo targets", msg->target_count);
    }
    
    void safety_check_callback()
    {
        // Собираем все цели со всех источников
        std::vector<care_common::msg::RadarTarget> all_targets;
        
        // Цели с устройств
        for (const auto& [device_id, targets_msg] : device_targets_) {
            for (const auto& target : targets_msg.targets) {
                if (target.valid) {
                    all_targets.push_back(target);
                }
            }
        }
        
        // Демо цели
        for (const auto& target : demo_targets_.targets) {
            if (target.valid) {
                all_targets.push_back(target);
            }
        }
        
        // Анализ безопасности с двумя зонами
        bool emergency_detected = false;
        bool slowdown_detected = false;
        std::vector<care_common::msg::RadarTarget> critical_targets;
        std::vector<care_common::msg::RadarTarget> slowdown_targets;
        
        for (auto& target : all_targets) {
            SafetyAnalysis analysis = analyze_target_safety(target, default_safety_zone_);
            
            if (analysis.is_critical) {
                emergency_detected = true;
                critical_targets.push_back(target);
            } else if (analysis.is_slowdown) {
                slowdown_detected = true;
                slowdown_targets.push_back(target);
            }
            
            // Обновляем информацию о безопасности в цели
            target.in_safety_zone = analysis.in_zone;
            target.safety_distance = analysis.distance_to_boundary;
        }
        
        // Принятие решений о безопасности
        handle_emergency_decision(emergency_detected, critical_targets);
        handle_slowdown_decision(slowdown_detected, slowdown_targets);
        
        // Обновление статистики
        total_targets_processed_ += all_targets.size();
        if (emergency_detected) {
            total_emergency_events_++;
        }
    }
    
    struct SafetyAnalysis {
        bool in_zone = false;
        bool is_slowdown = false;    // В зоне замедления
        bool is_critical = false;    // В критической зоне (экстренная остановка)
        double distance_to_boundary = 0.0;
        double angle_deviation = 0.0;
    };
    
    SafetyAnalysis analyze_target_safety(const care_common::msg::RadarTarget& target, 
                                       const care_common::msg::SafetyZone& zone)
    {
        SafetyAnalysis analysis;
        
        // Проверка расстояния
        analysis.distance_to_boundary = target.distance - zone.critical_distance;
        
        // Проверка угла (горизонтальный)
        double target_angle = std::abs(target.angle);
        analysis.angle_deviation = target_angle - (zone.horizontal_angle / 2.0);
        
        // Цель в зоне, если в пределах расстояния И угла
        analysis.in_zone = (target.distance <= zone.max_distance) && 
                          (target_angle <= zone.horizontal_angle / 2.0);
        
        if (analysis.in_zone) {
            // Критическое нарушение (экстренная остановка)
            if (target.distance <= zone.critical_distance) {
                analysis.is_critical = true;
            }
            // Зона замедления
            else if (target.distance <= zone.slowdown_distance) {
                analysis.is_slowdown = true;
            }
        }
        
        return analysis;
    }
    
    void handle_emergency_decision(bool emergency_detected, 
                                 const std::vector<care_common::msg::RadarTarget>& dangerous_targets)
    {
        auto now = std::chrono::steady_clock::now();
        
        if (emergency_detected) {
            if (!emergency_active_) {
                // Проверяем минимальное время срабатывания
                if (first_emergency_detection_.time_since_epoch().count() == 0) {
                    first_emergency_detection_ = now;
                    return; // Ждем подтверждения
                }
                
                if ((now - first_emergency_detection_) >= min_trigger_time_) {
                    // Активируем экстренную остановку
                    activate_emergency_stop(dangerous_targets);
                }
            }
        } else {
            if (emergency_active_) {
                // Проверяем гистерезис для снятия экстренной остановки
                if (check_emergency_clear_conditions()) {
                    deactivate_emergency_stop();
                }
            } else {
                // Сбрасываем таймер первого обнаружения
                first_emergency_detection_ = std::chrono::steady_clock::time_point{};
            }
        }
    }
    
    void activate_emergency_stop(const std::vector<care_common::msg::RadarTarget>& dangerous_targets)
    {
        emergency_active_ = true;
        emergency_start_time_ = std::chrono::steady_clock::now();
        
        // Формируем команду экстренной остановки
        care_common::msg::SafetyCommand command;
        command.header.stamp = this->get_clock()->now();
        command.command_type = care_common::msg::SafetyCommand::COMMAND_STOP;
        command.target_device_id = 0; // Все устройства
        command.priority = 255; // Максимальный приоритет
        command.sequence_number = ++command_sequence_;
        command.source_node_id = 1;
        command.source_node_name = "care_safety_controller";
        command.issued_time = command.header.stamp;
        command.require_ack = true;
        command.ack_timeout = rclcpp::Duration::from_seconds(1.0);
        
        // Дополнительная информация о причине
        std::string reason = "Emergency stop: " + std::to_string(dangerous_targets.size()) + " targets in critical zone";
        command.command_data = reason;
        
        // Публикация команды
        safety_command_pub_->publish(command);
        
        // Логирование
        log_event("EMERGENCY_STOP", reason);
        
        RCLCPP_ERROR(this->get_logger(), "🚨 EMERGENCY STOP ACTIVATED: %zu dangerous targets", 
                    dangerous_targets.size());
        
        for (const auto& target : dangerous_targets) {
            RCLCPP_ERROR(this->get_logger(), "⚠️ Dangerous target: device=%d, id=%d, distance=%.0fmm, angle=%.1f°", 
                        target.device_id, target.id, (double)target.distance, target.angle);
        }
    }
    
    bool check_emergency_clear_conditions()
    {
        // Проверяем, что все цели вышли из критической зоны с учетом гистерезиса
        double clear_distance = default_safety_zone_.critical_distance + emergency_hysteresis_;
        
        // Проверяем все источники данных
        for (const auto& [device_id, targets_msg] : device_targets_) {
            for (const auto& target : targets_msg.targets) {
                if (target.valid && target.distance < clear_distance && 
                    std::abs(target.angle) <= default_safety_zone_.horizontal_angle / 2.0) {
                    return false; // Еще есть цели в опасной зоне
                }
            }
        }
        
        // Проверяем демо цели
        for (const auto& target : demo_targets_.targets) {
            if (target.valid && target.distance < clear_distance && 
                std::abs(target.angle) <= default_safety_zone_.horizontal_angle / 2.0) {
                return false;
            }
        }
        
        return true; // Все цели вышли из опасной зоны
    }
    
    void deactivate_emergency_stop()
    {
        emergency_active_ = false;
        auto duration = std::chrono::steady_clock::now() - emergency_start_time_;
        
        // Формируем команду возобновления
        care_common::msg::SafetyCommand command;
        command.header.stamp = this->get_clock()->now();
        command.command_type = care_common::msg::SafetyCommand::COMMAND_RESUME;
        command.target_device_id = 0; // Все устройства
        command.priority = 200;
        command.sequence_number = ++command_sequence_;
        command.source_node_id = 1;
        command.source_node_name = "care_safety_controller";
        command.issued_time = command.header.stamp;
        command.require_ack = true;
        command.ack_timeout = rclcpp::Duration::from_seconds(1.0);
        command.command_data = "Emergency cleared - resuming operations";
        
        // Публикация команды
        safety_command_pub_->publish(command);
        
        // Логирование
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
        std::string reason = "Emergency cleared after " + std::to_string(duration_ms) + "ms";
        log_event("EMERGENCY_CLEAR", reason);
        
        RCLCPP_INFO(this->get_logger(), "✅ Emergency stop deactivated - operations resumed");
    }
    
    void handle_slowdown_decision(bool slowdown_detected, 
                                const std::vector<care_common::msg::RadarTarget>& slowdown_targets)
    {
        static bool slowdown_active = false;
        static std::chrono::steady_clock::time_point slowdown_start_time;
        
        if (slowdown_detected && !emergency_active_) {  // Не активируем замедление если уже есть экстренная остановка
            if (!slowdown_active) {
                // Активируем замедление
                slowdown_active = true;
                slowdown_start_time = std::chrono::steady_clock::now();
                
                care_common::msg::SafetyCommand command;
                command.header.stamp = this->get_clock()->now();
                command.command_type = care_common::msg::SafetyCommand::COMMAND_SLOWDOWN;
                command.target_device_id = 0; // Все устройства
                command.priority = 200;
                command.sequence_number = ++command_sequence_;
                command.source_node_id = 1;
                command.source_node_name = "care_safety_controller";
                command.issued_time = command.header.stamp;
                command.require_ack = false;
                
                std::string reason = "Slowdown activated: " + std::to_string(slowdown_targets.size()) + " targets in slowdown zone";
                command.command_data = reason;
                
                safety_command_pub_->publish(command);
                log_event("SLOWDOWN_ACTIVATE", reason);
                
                RCLCPP_WARN(this->get_logger(), "⚠️ SLOWDOWN ACTIVATED: %zu targets in slowdown zone", 
                           slowdown_targets.size());
            }
        } else if (!slowdown_detected && slowdown_active) {
            // Деактивируем замедление
            slowdown_active = false;
            auto duration = std::chrono::steady_clock::now() - slowdown_start_time;
            
            care_common::msg::SafetyCommand command;
            command.header.stamp = this->get_clock()->now();
            command.command_type = care_common::msg::SafetyCommand::COMMAND_CLEAR_SLOWDOWN;
            command.target_device_id = 0;
            command.priority = 200;
            command.sequence_number = ++command_sequence_;
            command.source_node_id = 1;
            command.source_node_name = "care_safety_controller";
            command.issued_time = command.header.stamp;
            command.require_ack = false;
            
            auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
            std::string reason = "Slowdown cleared after " + std::to_string(duration_ms) + "ms";
            command.command_data = reason;
            
            safety_command_pub_->publish(command);
            log_event("SLOWDOWN_CLEAR", reason);
            
            RCLCPP_INFO(this->get_logger(), "✅ Slowdown deactivated - normal speed resumed");
        }
    }
    
    void publish_system_status()
    {
        care_common::msg::SystemStatus status;
        status.header.stamp = this->get_clock()->now();
        
        // Общий статус системы
        if (emergency_active_) {
            status.overall_status = care_common::msg::SystemStatus::SYSTEM_EMERGENCY;
            status.status_message = "Emergency stop active";
        } else {
            status.overall_status = care_common::msg::SystemStatus::SYSTEM_OK;
            status.status_message = "System operating normally";
        }
        
        // Статус устройств
        status.active_devices = device_targets_.size();
        status.offline_devices = 0; // TODO: подсчет offline устройств
        
        // Статус системы безопасности
        status.safety_system_active = true;
        status.emergency_stop_active = emergency_active_;
        status.last_safety_check = this->get_clock()->now();
        
        // Метрики производительности
        auto now = std::chrono::steady_clock::now();
        if (start_time_.time_since_epoch().count() == 0) {
            start_time_ = now;
        }
        auto uptime_duration = now - start_time_;
        status.system_uptime = std::chrono::duration<float>(uptime_duration).count();
        status.message_rate = safety_check_rate_;
        status.total_targets_seen = total_targets_processed_;
        status.emergency_events = total_emergency_events_;
        
        // Использование ресурсов (заглушки)
        status.cpu_usage = 0.1; // TODO: реальный мониторинг
        status.memory_usage = 0.05;
        status.can_message_count = 0; // TODO: счетчик CAN сообщений
        
        system_status_pub_->publish(status);
    }
    
    void log_event(const std::string& event_type, const std::string& message)
    {
        if (!log_events_ || !log_stream_.is_open()) return;
        
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        log_stream_ << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
        log_stream_ << "." << std::setfill('0') << std::setw(3) << ms.count();
        log_stream_ << " [" << event_type << "] " << message << std::endl;
        log_stream_.flush();
    }
    
    // Service callbacks
    void set_safety_zone_callback(
        const std::shared_ptr<care_common::srv::SetSafetyZone::Request> request,
        std::shared_ptr<care_common::srv::SetSafetyZone::Response> response)
    {
        default_safety_zone_ = request->safety_zone;
        
        response->success = true;
        response->message = "Safety zone updated successfully";
        response->zone_id = default_safety_zone_.zone_id;
        response->applied_time = this->get_clock()->now();
        
        log_event("SAFETY_ZONE_UPDATE", "Safety zone configuration updated");
        RCLCPP_INFO(this->get_logger(), "🛡️ Safety zone updated: distance=%.0fmm, angle=%.0f°", 
                   default_safety_zone_.min_distance, default_safety_zone_.horizontal_angle);
    }
    
    void emergency_stop_callback(
        const std::shared_ptr<care_common::srv::EmergencyStop::Request> request,
        std::shared_ptr<care_common::srv::EmergencyStop::Response> response)
    {
        response->executed_time = this->get_clock()->now();
        
        switch (request->command) {
            case care_common::srv::EmergencyStop::Request::STOP_ALL:
                if (!emergency_active_) {
                    std::vector<care_common::msg::RadarTarget> empty_targets;
                    activate_emergency_stop(empty_targets);
                    response->success = true;
                    response->message = "Emergency stop activated manually";
                } else {
                    response->success = false;
                    response->message = "Emergency stop already active";
                }
                break;
                
            case care_common::srv::EmergencyStop::Request::RESUME_ALL:
                if (emergency_active_ && request->force) {
                    deactivate_emergency_stop();
                    response->success = true;
                    response->message = "Emergency stop deactivated manually";
                } else if (!emergency_active_) {
                    response->success = false;
                    response->message = "Emergency stop not active";
                } else {
                    response->success = false;
                    response->message = "Cannot resume - unsafe conditions detected";
                }
                break;
                
            default:
                response->success = false;
                response->message = "Unknown emergency command";
        }
        
        response->devices_affected = max_devices_;
        response->system_safe = !emergency_active_;
    }
    
    void get_system_status_callback(
        const std::shared_ptr<care_common::srv::GetSystemStatus::Request> request,
        std::shared_ptr<care_common::srv::GetSystemStatus::Response> response)
    {
        // Создаем актуальный статус системы
        care_common::msg::SystemStatus status;
        status.header.stamp = this->get_clock()->now();
        
        if (emergency_active_) {
            status.overall_status = care_common::msg::SystemStatus::SYSTEM_EMERGENCY;
            status.status_message = "Emergency stop active";
        } else {
            status.overall_status = care_common::msg::SystemStatus::SYSTEM_OK;
            status.status_message = "System operating normally";
        }
        
        status.active_devices = device_targets_.size();
        status.safety_system_active = true;
        status.emergency_stop_active = emergency_active_;
        status.total_targets_seen = total_targets_processed_;
        status.emergency_events = total_emergency_events_;
        
        response->success = true;
        response->message = "System status retrieved successfully";
        response->system_status = status;
        
        // Детальная информация об устройствах (если запрошена)
        if (request->include_device_details) {
            // TODO: добавить детальную информацию об устройствах
        }
    }
    
    // Параметры
    int max_devices_;
    double safety_check_rate_;
    double default_safety_distance_;
    double default_safety_angle_;
    double emergency_hysteresis_;
    std::chrono::duration<double> min_trigger_time_;
    bool log_events_;
    std::string log_file_;
    
    // Состояние системы безопасности
    bool emergency_active_;
    std::chrono::steady_clock::time_point emergency_start_time_;
    std::chrono::steady_clock::time_point first_emergency_detection_;
    care_common::msg::SafetyZone default_safety_zone_;
    uint32_t command_sequence_;
    
    // Данные целей со всех источников
    std::map<uint8_t, care_common::msg::RadarTargets> device_targets_;
    std::map<uint8_t, std::chrono::steady_clock::time_point> device_last_update_;
    care_common::msg::RadarTargets demo_targets_;
    std::chrono::steady_clock::time_point demo_last_update_;
    
    // Статистика
    std::chrono::steady_clock::time_point start_time_;
    uint32_t total_targets_processed_ = 0;
    uint32_t total_emergency_events_ = 0;
    
    // Логирование
    std::ofstream log_stream_;
    
    // ROS2 интерфейсы
    rclcpp::Publisher<care_common::msg::SafetyCommand>::SharedPtr safety_command_pub_;
    rclcpp::Publisher<care_common::msg::SystemStatus>::SharedPtr system_status_pub_;
    
    std::map<uint8_t, rclcpp::Subscription<care_common::msg::RadarTargets>::SharedPtr> targets_subscribers_;
    rclcpp::Subscription<care_common::msg::RadarTargets>::SharedPtr demo_targets_sub_;
    
    rclcpp::Service<care_common::srv::SetSafetyZone>::SharedPtr set_safety_zone_srv_;
    rclcpp::Service<care_common::srv::EmergencyStop>::SharedPtr emergency_stop_srv_;
    rclcpp::Service<care_common::srv::GetSystemStatus>::SharedPtr get_system_status_srv_;
    
    // Таймеры
    rclcpp::TimerBase::SharedPtr safety_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CareSafetyControllerNode>());
    rclcpp::shutdown();
    return 0;
}
