idf_component_register(
    SRCS "app_main.cpp" 
         "svpwm/esp_svpwm.c" 
         "foc/esp_foc.c"
         "SPICREATE/SPICREATE.cpp"
        "icm42688/icm42688.cpp"
    INCLUDE_DIRS "svpwm" 
                 "foc"
                 "SPICREATE"
                 "icm42688"
    REQUIRES driver esp_timer log
)