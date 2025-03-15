idf_component_register(SRCS "sensor_node.c"
                            "driver/bme280/bme280.c"
                            "driver/bme280/bme280_i2c.c"
                            "driver/pms5003/pms5003.c"
                            "driver/lora/lora.c"
                            "app/packet_assembly/packet_assembly.c"
                            "app/monitoring/monitoring.c"
                            "app/device/device.c"
                    INCLUDE_DIRS ".")
