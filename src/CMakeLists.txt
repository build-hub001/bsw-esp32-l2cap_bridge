# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

idf_component_register(
    SRCS 
        "main.c"
        "hid_l2cap.c"
    INCLUDE_DIRS 
        "." 
    REQUIRES  
        esp_timer driver nvs_flash json bt
    )