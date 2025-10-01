/**
 * @file care_demo_node.cpp
 * @brief C.A.R.E. Demo Node - Демонстрационный узел с симуляцией радара и визуализацией в RViz2
 * @author C.A.R.E. Development Team
 * @date 2025-10-01
 * @version 2.0.0
 *
 * @details Этот узел создаёт симуляцию работы радара LD2450 для демонстрации и отладки.
 * Генерирует синтетические цели, публикует визуализацию для RViz2 и TF трансформации.
 *
 * ## Основные функции:
 * - Генерация реалистичных движущихся целей
 * - Публикация TF трансформаций (map -> base_link -> radar_link)
 * - Визуализация целей как стрелок (Marker) в RViz2
 * - Отображение зоны безопасности (цилиндр)
 * - Публикация статуса системы
 *
 * ## ROS2 Интерфейсы:
 *
 * ### Публикации:
 * - `/care/radar_targets` (MarkerArray) - визуализация целей для RViz2
 * - `/care/safety_zone` (Marker) - визуализация зоны безопасности
 * - `/care/status` (String) - текстовый статус системы
 * - TF: `map` -> `base_link` -> `radar_link`
 *
 * ## Алгоритм симуляции:
 * 1. Генерируется 1-3 случайные цели с реалистичными параметрами:
 *    - Позиция: в пределах 6000мм от радара
 *    - Скорость: -200..+200 см/с
 *    - Траектория: линейное движение с обновлением каждые 100мс
 * 2. Публикуются TF трансформации с синхронизированными timestamp
 * 3. Цели визуализируются как цветные стрелки (красные в зоне опасности, зелёные вне)
 * 4. Зона безопасности отображается как полупрозрачный цилиндр
 *
 * ## Важно:
 * - Используется **единый timestamp** для TF и маркеров (критично для RViz2!)
 * - TF публикуется перед маркерами с задержкой 1мс для попадания в кэш
 * - Статические трансформации публикуются только один раз при старте
 *
 * ## Пример запуска с RViz2:
 * @code
 * # Терминал 1: Запуск демо ноды
 * ros2 run care_demo_node care_demo_node
 *
 * # Терминал 2: Запуск RViz2 с конфигом
 * rviz2 -d $(ros2 pkg prefix care_demo_node)/share/care_demo_node/config/care_demo.rviz
 * @endcode
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <random>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class CareRadarPublisher : public rclcpp::Node
{
public:
    CareRadarPublisher()
    : Node("care_radar_publisher"), count_(0)
    {
        // Publishers
        targets_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/care/radar_targets", 10);
        safety_zone_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/care/safety_zone", 10);
        status_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/care/status", 10);

        // TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Static TF broadcaster для стабильных трансформаций
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        // Timer - единый цикл для всех публикаций
        timer_ = this->create_wall_timer(
            100ms, std::bind(&CareRadarPublisher::timer_callback, this));

        // Initialize random generator
        gen_.seed(std::chrono::system_clock::now().time_since_epoch().count());

        // Публикуем статические трансформации
        publish_static_transforms();

        RCLCPP_INFO(this->get_logger(), "🎯 C.A.R.E. Radar Publisher Started!");
        RCLCPP_INFO(this->get_logger(), "📡 Publishing mock radar data for RViz2 visualization");
    }

private:
    void timer_callback()
    {
        // КРИТИЧНО: Получаем время ОДИН раз для всего цикла
        auto current_stamp = this->get_clock()->now();

        // 1. СНАЧАЛА публикуем TF с текущим временем
        publish_transforms_with_time(current_stamp);

        // 2. Небольшая задержка для гарантии попадания TF в кэш
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // 3. ПОТОМ публикуем маркеры с ТЕМ ЖЕ временем
        publish_radar_targets_with_time(current_stamp);
        publish_safety_zone_with_time(current_stamp);
        publish_status();
        count_++;
    }

    void publish_static_transforms()
    {
        std::vector<geometry_msgs::msg::TransformStamped> static_transforms;

        // map -> base_link (статическая трансформация)
        geometry_msgs::msg::TransformStamped map_to_base;
        map_to_base.header.stamp = this->get_clock()->now();
        map_to_base.header.frame_id = "map";
        map_to_base.child_frame_id = "base_link";
        map_to_base.transform.translation.x = 0.0;
        map_to_base.transform.translation.y = 0.0;
        map_to_base.transform.translation.z = 0.0;
        map_to_base.transform.rotation.x = 0.0;
        map_to_base.transform.rotation.y = 0.0;
        map_to_base.transform.rotation.z = 0.0;
        map_to_base.transform.rotation.w = 1.0;
        static_transforms.push_back(map_to_base);

        // base_link -> radar_link (статическая трансформация)
        geometry_msgs::msg::TransformStamped base_to_radar;
        base_to_radar.header.stamp = this->get_clock()->now();
        base_to_radar.header.frame_id = "base_link";
        base_to_radar.child_frame_id = "radar_link";
        base_to_radar.transform.translation.x = 0.2;
        base_to_radar.transform.translation.y = 0.0;
        base_to_radar.transform.translation.z = 0.1;
        base_to_radar.transform.rotation.x = 0.0;
        base_to_radar.transform.rotation.y = 0.0;
        base_to_radar.transform.rotation.z = 0.0;
        base_to_radar.transform.rotation.w = 1.0;
        static_transforms.push_back(base_to_radar);

        static_tf_broadcaster_->sendTransform(static_transforms);
        RCLCPP_INFO(this->get_logger(), "📡 Static TF transforms published");
    }

    void publish_transforms_with_time(const rclcpp::Time& stamp)
    {
        // КРИТИЧНО: Используем ТОЧНО то же время, что и маркеры

        // map -> base_link
        geometry_msgs::msg::TransformStamped map_to_base;
        map_to_base.header.stamp = stamp;
        map_to_base.header.frame_id = "map";
        map_to_base.child_frame_id = "base_link";
        map_to_base.transform.translation.x = 0.0;
        map_to_base.transform.translation.y = 0.0;
        map_to_base.transform.translation.z = 0.0;
        map_to_base.transform.rotation.x = 0.0;
        map_to_base.transform.rotation.y = 0.0;
        map_to_base.transform.rotation.z = 0.0;
        map_to_base.transform.rotation.w = 1.0;

        // base_link -> radar_link
        geometry_msgs::msg::TransformStamped base_to_radar;
        base_to_radar.header.stamp = stamp;
        base_to_radar.header.frame_id = "base_link";
        base_to_radar.child_frame_id = "radar_link";
        base_to_radar.transform.translation.x = 0.2;
        base_to_radar.transform.translation.y = 0.0;
        base_to_radar.transform.translation.z = 0.1;
        base_to_radar.transform.rotation.x = 0.0;
        base_to_radar.transform.rotation.y = 0.0;
        base_to_radar.transform.rotation.z = 0.0;
        base_to_radar.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform({map_to_base, base_to_radar});
    }

    void publish_radar_targets_with_time(const rclcpp::Time& current_stamp)
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();

        // LD2450 FoV: 60° horizontal, 35° vertical
        const double H_FOV = 60.0 * M_PI / 180.0; // ±30°
        const double V_FOV = 35.0 * M_PI / 180.0; // ±17.5°
        const int MAX_TARGETS = 5;

        // КРИТИЧНО: Используем переданное время (то же, что и TF)
        double current_time = count_ * 0.1; // 10Hz update rate

        for (int i = 0; i < MAX_TARGETS; i++) {
            // Target lifecycle: появление с задержкой 1 сек, жизнь 5 сек, исчезновение 2 сек
            double target_cycle = 8.0; // 1 + 5 + 2 = 8 секунд полный цикл
            double target_phase = fmod(current_time + i * 1.0, target_cycle); // сдвиг по 1 сек

            if (target_phase >= 1.0 && target_phase < 6.0) { // активная фаза 5 секунд
                auto marker = visualization_msgs::msg::Marker();
                marker.header.frame_id = "radar_link";
                marker.header.stamp = current_stamp; // Единое время!
                marker.ns = "radar_targets";
                marker.id = i;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;

                // Движение цели в пределах FoV
                double life_time = target_phase - 1.0; // время жизни цели (0-5 сек)

                // Горизонтальное движение в пределах ±30°
                double h_angle = (H_FOV/2) * std::sin(life_time * 0.8 + i * 1.2) * 0.8;

                // Вертикальное движение в пределах ±17.5°
                double v_angle = (V_FOV/2) * std::sin(life_time * 1.2 + i * 0.8) * 0.6;

                // Дистанция с небольшим изменением
                double distance = 2.0 + i * 0.5 + 0.3 * std::sin(life_time * 0.5 + i);

                // Позиция в 3D пространстве
                marker.pose.position.x = distance * std::cos(v_angle) * std::cos(h_angle);
                marker.pose.position.y = distance * std::cos(v_angle) * std::sin(h_angle);
                marker.pose.position.z = distance * std::sin(v_angle);

                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                // Размер зависит от дистанции
                double scale = 0.08 + 0.04 / distance;
                marker.scale.x = scale;
                marker.scale.y = scale;
                marker.scale.z = scale;

                // Цвет зависит от опасности
                double h_angle_deg = std::abs(h_angle * 180.0 / M_PI);
                bool is_dangerous = distance < 1.5 && h_angle_deg < 30.0;

                if (is_dangerous) {
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                } else if (distance < 2.5) {
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                } else {
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                }

                // Плавное появление/исчезновение
                if (life_time < 0.5) {
                    marker.color.a = life_time * 1.6; // плавное появление
                } else if (life_time > 4.5) {
                    marker.color.a = (5.0 - life_time) * 1.6; // плавное исчезновение
                } else {
                    marker.color.a = 0.8;
                }

                // ИСПРАВЛЕНО: Увеличиваем время жизни маркера до 1 секунды
                marker.lifetime = rclcpp::Duration::from_seconds(1.0);
                marker_array.markers.push_back(marker);
            }
        }

        targets_publisher_->publish(marker_array);
    }

    void publish_safety_zone_with_time(const rclcpp::Time& current_stamp)
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();

        // LD2450 FoV параметры
        const double H_FOV = 60.0 * M_PI / 180.0; // ±30°
        const double V_FOV = 35.0 * M_PI / 180.0; // ±17.5°
        const double safety_distance = 1.5;
        const double max_range = 6.0;

        // 1. Горизонтальный FoV (в плоскости XY)
        auto h_fov_marker = visualization_msgs::msg::Marker();
        h_fov_marker.header.frame_id = "radar_link";
        h_fov_marker.header.stamp = current_stamp;
        h_fov_marker.ns = "fov_horizontal";
        h_fov_marker.id = 0;
        h_fov_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        h_fov_marker.action = visualization_msgs::msg::Marker::ADD;
        h_fov_marker.pose.orientation.w = 1.0;
        h_fov_marker.scale.x = 0.01;
        h_fov_marker.color.r = 0.0;
        h_fov_marker.color.g = 0.8;
        h_fov_marker.color.b = 1.0;
        h_fov_marker.color.a = 0.4;
        h_fov_marker.lifetime = rclcpp::Duration::from_seconds(1.0);

        // Горизонтальный FoV границы
        for (double angle = -H_FOV/2; angle <= H_FOV/2; angle += 0.05) {
            geometry_msgs::msg::Point p;
            p.x = max_range * std::cos(angle);
            p.y = max_range * std::sin(angle);
            p.z = 0.0;
            h_fov_marker.points.push_back(p);
        }

        // Замыкание к центру
        geometry_msgs::msg::Point origin;
        origin.x = 0.0; origin.y = 0.0; origin.z = 0.0;
        h_fov_marker.points.push_back(origin);

        geometry_msgs::msg::Point start;
        start.x = max_range * std::cos(-H_FOV/2);
        start.y = max_range * std::sin(-H_FOV/2);
        start.z = 0.0;
        h_fov_marker.points.push_back(start);

        marker_array.markers.push_back(h_fov_marker);

        // 2. Вертикальный FoV (в плоскости XZ)
        auto v_fov_marker = visualization_msgs::msg::Marker();
        v_fov_marker.header.frame_id = "radar_link";
        v_fov_marker.header.stamp = current_stamp;
        v_fov_marker.ns = "fov_vertical";
        v_fov_marker.id = 0;
        v_fov_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        v_fov_marker.action = visualization_msgs::msg::Marker::ADD;
        v_fov_marker.pose.orientation.w = 1.0;
        v_fov_marker.scale.x = 0.01;
        v_fov_marker.color.r = 0.0;
        v_fov_marker.color.g = 1.0;
        v_fov_marker.color.b = 0.8;
        v_fov_marker.color.a = 0.4;
        v_fov_marker.lifetime = rclcpp::Duration::from_seconds(1.0);

        // Вертикальный FoV границы (в плоскости XZ)
        for (double angle = -V_FOV/2; angle <= V_FOV/2; angle += 0.05) {
            geometry_msgs::msg::Point p;
            p.x = max_range * std::cos(angle);
            p.y = 0.0;
            p.z = max_range * std::sin(angle);
            v_fov_marker.points.push_back(p);
        }

        // Замыкание к центру
        v_fov_marker.points.push_back(origin);

        geometry_msgs::msg::Point v_start;
        v_start.x = max_range * std::cos(-V_FOV/2);
        v_start.y = 0.0;
        v_start.z = max_range * std::sin(-V_FOV/2);
        v_fov_marker.points.push_back(v_start);

        marker_array.markers.push_back(v_fov_marker);

        // 3. Зона безопасности (красная)
        auto safety_marker = visualization_msgs::msg::Marker();
        safety_marker.header.frame_id = "radar_link";
        safety_marker.header.stamp = current_stamp;
        safety_marker.ns = "safety_zone";
        safety_marker.id = 0;
        safety_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        safety_marker.action = visualization_msgs::msg::Marker::ADD;
        safety_marker.pose.orientation.w = 1.0;
        safety_marker.scale.x = 0.03;
        safety_marker.color.r = 1.0;
        safety_marker.color.g = 0.0;
        safety_marker.color.b = 0.0;
        safety_marker.color.a = 0.6;
        safety_marker.lifetime = rclcpp::Duration::from_seconds(1.0);

        // Зона безопасности в горизонтальной плоскости
        for (double angle = -H_FOV/2; angle <= H_FOV/2; angle += 0.05) {
            geometry_msgs::msg::Point p;
            p.x = safety_distance * std::cos(angle);
            p.y = safety_distance * std::sin(angle);
            p.z = 0.0;
            safety_marker.points.push_back(p);
        }

        safety_marker.points.push_back(origin);

        geometry_msgs::msg::Point safety_start;
        safety_start.x = safety_distance * std::cos(-H_FOV/2);
        safety_start.y = safety_distance * std::sin(-H_FOV/2);
        safety_start.z = 0.0;
        safety_marker.points.push_back(safety_start);

        marker_array.markers.push_back(safety_marker);

        // Публикуем зону безопасности
        safety_zone_publisher_->publish(safety_marker);

        // Публикуем FoV отдельно
        static rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fov_publisher_;
        if (!fov_publisher_) {
            fov_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/care/fov_visualization", 10);
        }

        visualization_msgs::msg::MarkerArray fov_array;
        fov_array.markers.push_back(h_fov_marker);
        fov_array.markers.push_back(v_fov_marker);
        fov_publisher_->publish(fov_array);
    }

    void publish_status()
    {
        auto message = std_msgs::msg::String();

        // Check for emergency conditions
        bool emergency = (count_ % 50) < 10; // Emergency every 5 seconds for 1 second

        if (emergency) {
            message.data = "🚨 EMERGENCY STOP: Object in safety zone!";
        } else {
            message.data = "✅ C.A.R.E. System Normal - Mock sensor active";
        }

        status_publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr targets_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr safety_zone_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::mt19937 gen_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CareRadarPublisher>());
    rclcpp::shutdown();
    return 0;
}
