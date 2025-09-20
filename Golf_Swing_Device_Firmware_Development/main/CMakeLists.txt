idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES  push_button )


spiffs_create_partition_image(storage ../partition FLASH_IN_PROJECT)