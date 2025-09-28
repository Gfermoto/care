# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

# C.A.R.E. ESP32 Main Component
idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "../include"
    REQUIRES 
        driver
        esp_wifi
        nvs_flash
        freertos
        esp_event
        esp_netif
)
