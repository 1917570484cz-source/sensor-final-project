# drivers

ESP32-S3 固件主驱动集中在 `../main.c` 中，包括：

- I2C 初始化与扫描
- MPU6050/MPU6500 兼容 IMU 读取
- HMC5883L 磁力计读取
- BMP280/BME280 气压计补偿读取
- GNSS UART/NMEA 接收
- PPS GPIO 中断时间同步

本目录存放 PC 端串口记录和离线读取辅助脚本。
