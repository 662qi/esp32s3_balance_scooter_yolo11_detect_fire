set(src_dirs
            LED
            KEY
            GPTIM
            OLED
            MYI2C
            MPU6050
            MYUART
            PWM
            MOTOR
            ENCODER
            PID)

set(include_dirs
            .
            LED
            KEY
            GPTIM
            OLED
            MYI2C
            MPU6050
            MYUART
            PWM
            MOTOR
            ENCODER
            PID)

set(requires
            driver
            esp_timer
            esp_driver_gptimer)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

component_compile_options(-ffast-math -O3 -Wno-error=format -Wno-format)