idf_component_register(SRCS "buzzer.c" "main.c"
                    INCLUDE_DIRS ".")
                    spiffs_create_partition_image(storage ../partition FLASH_IN_PROJECT)