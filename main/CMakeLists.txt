idf_component_register(
    SRCS "app_main.cpp" 
         "svpwm/esp_svpwm.c" 
         "foc/esp_foc.c"
         "SPICREATE/SPICREATE.cpp"
         "CAN/CanComm.cpp"
    INCLUDE_DIRS "svpwm" 
                 "foc"
                 "SPICREATE"
                 "icm42688"
                 "gptimer"
                 "Quaternion"
                 "SdmmcLogger"
                 "CAN"
    REQUIRES driver esp_timer log sdmmc fatfs
)