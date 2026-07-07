#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_check.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#define I2C_PORT I2C_NUM_0

// 当前固定在 ESP32-S3 扩展板上，I2C 飞线为 SDA=GPIO8、SCL=GPIO9。
// 如果以后换成普通 ESP32 开发板，则使用下面 else 分支的 GPIO21/GPIO22。
#if CONFIG_IDF_TARGET_ESP32S3
#define I2C_SDA_IO GPIO_NUM_8
#define I2C_SCL_IO GPIO_NUM_9
#else
#define I2C_SDA_IO GPIO_NUM_21
#define I2C_SCL_IO GPIO_NUM_22
#endif

#define I2C_FREQ_HZ 100000
#define I2C_TIMEOUT_MS 100

#define GPS_UART_PORT UART_NUM_1
#define GPS_UART_RX_IO GPIO_NUM_16
#define GPS_UART_TX_IO GPIO_NUM_17
#define GPS_UART_BAUD 9600
#define GPS_UART_BUF_SIZE 1024
#define GPS_NMEA_LINE_MAX 160
#define GPS_STATUS_PERIOD_US 5000000LL
#define GPS_PPS_IO GPIO_NUM_21
#define GPS_SYNC_TASK_STACK 4096
#define GPS_SYNC_TASK_PRIORITY 10

#define MPU_ADDR 0x68
#define MPU_REG_SMPLRT_DIV 0x19
#define MPU_REG_CONFIG 0x1A
#define MPU_REG_GYRO_CONFIG 0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_PWR_MGMT_1 0x6B
#define MPU_REG_WHO_AM_I 0x75
#define MPU_WHO_AM_I_6050 0x68
#define MPU_WHO_AM_I_6500 0x70
#define ACCEL_SCALE_2G 16384.0f
#define GYRO_SCALE_500 65.5f

#define HMC_ADDR 0x1E
#define HMC_REG_CRA 0x00
#define HMC_REG_CRB 0x01
#define HMC_REG_MODE 0x02
#define HMC_REG_DOUT 0x03
#define HMC_REG_SR 0x09
#define HMC_REG_IDA 0x0A
#define HMC_GAIN_LSB_PER_GAUSS 1090.0f

#define BMP280_ADDR_PRIMARY 0x76
#define BMP280_ADDR_SECONDARY 0x77
#define BMP280_REG_ID 0xD0
#define BMP280_REG_RESET 0xE0
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_CALIB 0x88

#define LOOP_PERIOD_MS 4
#define SENSOR_PRINT_PERIOD_MS 100
#define HMC_READ_PERIOD_MS 20
#define BMP_READ_PERIOD_MS 100
#define COMPLEMENTARY_ALPHA 0.98f
#define DEG_PER_RAD 57.2957795f
#define RAD_PER_DEG 0.0174532925f
#define MAHONY_TWO_KP 2.0f
#define MAHONY_TWO_KI 0.0f
#define BMP_LOG_MAGIC 0x424D5031u
#define BMP_LOG_MAX_SAMPLES 360
#define BMP_LOG_DEFAULT_SECONDS 180
#define BMP_LOG_INTERVAL_MS 500
#define BMP_LOG_CMD_BUF_LEN 96
#define GPS_LOG_MAGIC 0x47505332u
#define GPS_LOG_MAX_SAMPLES 600
#define GPS_LOG_DEFAULT_SECONDS 300
#define PERF_REPORT_SAMPLES 1000

static const char *TAG = "NO_GPS_FUSION";

// 三个传感器共用同一条 I2C 总线，启动时分别添加为独立设备句柄。
static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t mpu_dev;
static i2c_master_dev_handle_t hmc_dev;
static i2c_master_dev_handle_t bmp_dev;
static SemaphoreHandle_t i2c_mutex;

static bool has_mpu;
static bool has_hmc;
static bool has_bmp;
static bool has_gps_uart;
static uint8_t bmp_addr;

static bool attitude_initialized;
static int64_t last_attitude_us;
static float roll_deg;
static float pitch_deg;
static bool mahony_initialized;
static int64_t last_mahony_us;
static float mahony_q0 = 1.0f;
static float mahony_q1;
static float mahony_q2;
static float mahony_q3;
static float mahony_integral_x;
static float mahony_integral_y;
static float mahony_integral_z;
static float mahony_roll_deg;
static float mahony_pitch_deg;
static float mahony_yaw_deg;
static float p0_pa;

// MPU6050/MPU6500 加速度计六位置标定参数。
// 标定文件：data/accel_calibration_20260705_114101.txt
// 模型：raw = A * true + C_BIAS，固件中使用 corrected = A_INV * (raw - C_BIAS)。
// 本次验证：标定前平均模长误差 20.068 mg，标定后平均向量残差 1.798 mg。
static const float A_INV[3][3] = {
    {1.000085932f, 0.028224218f, 0.001797831f},
    {-0.028359154f, 1.000349137f, -0.021617191f},
    {-0.004685815f, 0.009701288f, 0.989644763f},
};

static const float C_BIAS[3] = {
    -0.013395542f,
    -0.004354629f,
    -0.044735174f,
};

static const float GYRO_GX_COEF[3] = {2.08491024f, 0.02963530f, -0.00101791f};
static const float GYRO_GY_COEF[3] = {-0.95428364f, 0.08098127f, -0.00180494f};
static const float GYRO_GZ_COEF[3] = {1.28524201f, -0.06686667f, 0.00116076f};

// HMC5883L 磁力计椭球标定参数。
// 标定文件：data/mag_calibration_20260705_134935.txt
// 阶段结果：磁场模长标准差由 3.303 uT 降至 1.892 uT，后续仍需 12 方向航向验证。
static const float MAG_C[3] = {
    2.380116446f,
    -2.222724669f,
    1.008089606f,
};

static const float MAG_W[3][3] = {
    {1.185171607f, 0.006230829f, 0.030442137f},
    {0.006230829f, 1.154072552f, 0.005094636f},
    {0.030442137f, 0.005094636f, 1.210237276f},
};

typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} bmp280_calib_t;

typedef struct {
    int64_t timestamp_us;
    // raw 字段保留未标定数据，用于重新标定和报告对比。
    float ax_raw;
    float ay_raw;
    float az_raw;
    float gx_raw;
    float gy_raw;
    float gz_raw;
    // 非 raw 字段为固件补偿后的正式输出，姿态算法也使用这些值。
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float temp;
} imu_data_t;

typedef struct {
    int64_t timestamp_us;
    float bx;
    float by;
    float bz;
    float magnitude;
} mag_data_t;

typedef struct {
    int64_t timestamp_us;
    float temperature_c;
    float pressure_pa;
    float altitude_m;
} baro_data_t;

typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    int32_t pressure_pa_x100;
    int16_t temp_c_x100;
    int16_t altitude_cm;
} bmp_log_sample_t;

typedef struct {
    uint32_t magic;
    uint32_t sequence;
    uint32_t count;
    uint32_t interval_ms;
    uint32_t duration_ms;
    float p0_pa;
} bmp_log_meta_t;

typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    uint32_t utc_ms;
    int32_t lat_e7;
    int32_t lon_e7;
    int32_t altitude_cm;
    int32_t baro_pressure_pa_x100;
    int16_t baro_temp_c_x100;
    int16_t baro_altitude_cm;
    uint16_t speed_kn_x100;
    uint16_t course_deg_x100;
    uint16_t hdop_x100;
    uint8_t satellites;
    uint8_t quality;
    uint8_t checksum_ok;
} gps_log_sample_t;

typedef struct {
    uint32_t magic;
    uint32_t sequence;
    uint32_t count;
    uint32_t duration_ms;
} gps_log_meta_t;

typedef struct {
    uint32_t count;
    int64_t window_start_us;
    int64_t window_end_us;
    uint32_t imu_update_count;
    uint32_t comp_update_count;
    uint32_t mahony_update_count;
    uint32_t sensor_print_count;
    int64_t loop_sum_us;
    int64_t loop_max_us;
    int64_t mpu_sum_us;
    int64_t mpu_max_us;
    int64_t comp_sum_us;
    int64_t comp_max_us;
    int64_t hmc_sum_us;
    int64_t hmc_max_us;
    int64_t yaw_sum_us;
    int64_t yaw_max_us;
    int64_t mahony_sum_us;
    int64_t mahony_max_us;
    int64_t bmp_sum_us;
    int64_t bmp_max_us;
} perf_stats_t;

static bmp280_calib_t bmp_calib;
static int32_t bmp_t_fine;
static bmp_log_sample_t bmp_log_buffer[BMP_LOG_MAX_SAMPLES];
static gps_log_sample_t gps_log_buffer[GPS_LOG_MAX_SAMPLES];
static bool nvs_ready;
static char bmp_log_cmd_buf[BMP_LOG_CMD_BUF_LEN];
static size_t bmp_log_cmd_len;
static perf_stats_t perf_stats;
static char gps_nmea_line[GPS_NMEA_LINE_MAX];
static size_t gps_nmea_len;
static uint32_t gps_nmea_lines;
static uint32_t gps_nmea_bytes;
static int64_t gps_last_status_us;
static volatile uint32_t gps_pps_count;
static volatile int64_t gps_last_pps_us;
static portMUX_TYPE gps_pps_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t gps_reported_pps_count;
static int64_t gps_prev_reported_pps_us;
static TaskHandle_t gps_sync_task_handle;
static bool gps_log_capture_active;
static int64_t gps_log_capture_start_us;
static uint32_t gps_log_capture_count;
static uint16_t gps_latest_speed_kn_x100;
static uint16_t gps_latest_course_deg_x100;

static int16_t be_i16(const uint8_t *buf)
{
    return (int16_t)((buf[0] << 8) | buf[1]);
}

static uint16_t le_u16(const uint8_t *buf)
{
    return (uint16_t)(buf[0] | (buf[1] << 8));
}

static void gps_poll_uart(void);
static bool gps_log_handle_command(const char *cmd);

static int16_t le_i16(const uint8_t *buf)
{
    return (int16_t)(buf[0] | (buf[1] << 8));
}

static esp_err_t add_i2c_device(uint8_t addr, i2c_master_dev_handle_t *out)
{
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    return i2c_master_bus_add_device(i2c_bus, &dev_config, out);
}

static esp_err_t dev_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    if (i2c_mutex && xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, data, len, I2C_TIMEOUT_MS);
    if (i2c_mutex) {
        xSemaphoreGive(i2c_mutex);
    }
    return err;
}

static esp_err_t dev_write_u8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    if (i2c_mutex && xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = i2c_master_transmit(dev, data, sizeof(data), I2C_TIMEOUT_MS);
    if (i2c_mutex) {
        xSemaphoreGive(i2c_mutex);
    }
    return err;
}

static esp_err_t i2c_bus_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = I2C_SCL_IO,
        .sda_io_num = I2C_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_mutex = xSemaphoreCreateMutex();
    if (!i2c_mutex) {
        return ESP_ERR_NO_MEM;
    }
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &i2c_bus), TAG, "I2C bus init failed");
    ESP_LOGI(TAG, "I2C initialized: SDA=%d SCL=%d freq=%d Hz", I2C_SDA_IO, I2C_SCL_IO, I2C_FREQ_HZ);
    return ESP_OK;
}

static void i2c_scan(void)
{
    // 上电自检：确认 0x68(MPU)、0x1E(HMC)、0x76(BMP) 是否在总线上。
    printf("I2C_SCAN_BEGIN\n");
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        esp_err_t err = i2c_master_probe(i2c_bus, addr, 30);
        if (err == ESP_OK) {
            printf("I2C_DEVICE,0x%02X\n", addr);
        }
    }
    printf("I2C_SCAN_END\n");
}

static esp_err_t mpu_init(void)
{
    ESP_RETURN_ON_ERROR(add_i2c_device(MPU_ADDR, &mpu_dev), TAG, "add MPU failed");

    uint8_t id = 0;
    esp_err_t err = dev_read(mpu_dev, MPU_REG_WHO_AM_I, &id, 1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "MPU not found at 0x%02X: %s", MPU_ADDR, esp_err_to_name(err));
        return err;
    }
    if (id != MPU_WHO_AM_I_6050 && id != MPU_WHO_AM_I_6500 && id != 0x69) {
        ESP_LOGW(TAG, "Unexpected MPU WHO_AM_I=0x%02X; continuing for board test", id);
    } else {
        ESP_LOGI(TAG, "MPU detected: WHO_AM_I=0x%02X", id);
    }

    ESP_RETURN_ON_ERROR(dev_write_u8(mpu_dev, MPU_REG_PWR_MGMT_1, 0x01), TAG, "wake MPU failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(dev_write_u8(mpu_dev, MPU_REG_SMPLRT_DIV, 3), TAG, "MPU sample rate failed");
    ESP_RETURN_ON_ERROR(dev_write_u8(mpu_dev, MPU_REG_CONFIG, 0x03), TAG, "MPU DLPF failed");
    ESP_RETURN_ON_ERROR(dev_write_u8(mpu_dev, MPU_REG_GYRO_CONFIG, 0x08), TAG, "MPU gyro range failed");
    ESP_RETURN_ON_ERROR(dev_write_u8(mpu_dev, MPU_REG_ACCEL_CONFIG, 0x00), TAG, "MPU accel range failed");

    has_mpu = true;
    return ESP_OK;
}

static void apply_accel_calibration(float *ax, float *ay, float *az)
{
    // 六位置标定的在线应用：先减零偏，再乘 3x3 逆矩阵修正比例因子与轴间耦合。
    float raw[3] = {
        *ax - C_BIAS[0],
        *ay - C_BIAS[1],
        *az - C_BIAS[2],
    };
    float corrected[3];
    for (int row = 0; row < 3; row++) {
        corrected[row] = A_INV[row][0] * raw[0] +
                         A_INV[row][1] * raw[1] +
                         A_INV[row][2] * raw[2];
    }
    *ax = corrected[0];
    *ay = corrected[1];
    *az = corrected[2];
}

static float compensate_gyro(float raw, const float coef[3], float temp)
{
    // 陀螺仪温度补偿使用二次多项式 bias = a + b*T + c*T^2。
    float bias = coef[0] + coef[1] * temp + coef[2] * temp * temp;
    return raw - bias;
}

static esp_err_t mpu_read_sample(imu_data_t *out)
{
    uint8_t raw[14];
    ESP_RETURN_ON_ERROR(dev_read(mpu_dev, MPU_REG_ACCEL_XOUT_H, raw, sizeof(raw)), TAG, "read MPU sample failed");

    int16_t raw_ax = be_i16(&raw[0]);
    int16_t raw_ay = be_i16(&raw[2]);
    int16_t raw_az = be_i16(&raw[4]);
    int16_t raw_temp = be_i16(&raw[6]);
    int16_t raw_gx = be_i16(&raw[8]);
    int16_t raw_gy = be_i16(&raw[10]);
    int16_t raw_gz = be_i16(&raw[12]);

    out->timestamp_us = esp_timer_get_time();
    out->ax_raw = raw_ax / ACCEL_SCALE_2G;
    out->ay_raw = raw_ay / ACCEL_SCALE_2G;
    out->az_raw = raw_az / ACCEL_SCALE_2G;
    out->gx_raw = raw_gx / GYRO_SCALE_500;
    out->gy_raw = raw_gy / GYRO_SCALE_500;
    out->gz_raw = raw_gz / GYRO_SCALE_500;

    // 加速度：同时输出 raw 和标定后数据，便于验证“标定前/后误差”。
    out->ax = out->ax_raw;
    out->ay = out->ay_raw;
    out->az = out->az_raw;
    out->temp = raw_temp / 340.0f + 36.53f;
    apply_accel_calibration(&out->ax, &out->ay, &out->az);

    // 陀螺仪：保留 raw，同时输出温度补偿后的 gx/gy/gz。
    out->gx = compensate_gyro(out->gx_raw, GYRO_GX_COEF, out->temp);
    out->gy = compensate_gyro(out->gy_raw, GYRO_GY_COEF, out->temp);
    out->gz = compensate_gyro(out->gz_raw, GYRO_GZ_COEF, out->temp);
    return ESP_OK;
}

static void complementary_update(const imu_data_t *data)
{
    // 互补滤波：加速度计给低频姿态，陀螺仪积分给短时动态响应。
    float roll_acc = atan2f(data->ay, data->az) * DEG_PER_RAD;
    float pitch_acc = atan2f(-data->ax, sqrtf(data->ay * data->ay + data->az * data->az)) * DEG_PER_RAD;

    if (!attitude_initialized) {
        roll_deg = roll_acc;
        pitch_deg = pitch_acc;
        last_attitude_us = data->timestamp_us;
        attitude_initialized = true;
        return;
    }

    float dt_s = (data->timestamp_us - last_attitude_us) / 1000000.0f;
    last_attitude_us = data->timestamp_us;
    if (dt_s <= 0.0f || dt_s > 1.0f) {
        dt_s = LOOP_PERIOD_MS / 1000.0f;
    }

    roll_deg += data->gx * dt_s;
    pitch_deg += data->gy * dt_s;
    roll_deg = COMPLEMENTARY_ALPHA * roll_deg + (1.0f - COMPLEMENTARY_ALPHA) * roll_acc;
    pitch_deg = COMPLEMENTARY_ALPHA * pitch_deg + (1.0f - COMPLEMENTARY_ALPHA) * pitch_acc;
}

static bool normalize3(float *x, float *y, float *z)
{
    float norm = sqrtf((*x) * (*x) + (*y) * (*y) + (*z) * (*z));
    if (!isfinite(norm) || norm <= 1e-6f) {
        return false;
    }
    *x /= norm;
    *y /= norm;
    *z /= norm;
    return true;
}

static void mahony_set_from_euler(float roll, float pitch, float yaw)
{
    float cr = cosf(0.5f * roll * RAD_PER_DEG);
    float sr = sinf(0.5f * roll * RAD_PER_DEG);
    float cp = cosf(0.5f * pitch * RAD_PER_DEG);
    float sp = sinf(0.5f * pitch * RAD_PER_DEG);
    float cy = cosf(0.5f * yaw * RAD_PER_DEG);
    float sy = sinf(0.5f * yaw * RAD_PER_DEG);

    mahony_q0 = cr * cp * cy + sr * sp * sy;
    mahony_q1 = sr * cp * cy - cr * sp * sy;
    mahony_q2 = cr * sp * cy + sr * cp * sy;
    mahony_q3 = cr * cp * sy - sr * sp * cy;
    float q_norm = sqrtf(mahony_q0 * mahony_q0 + mahony_q1 * mahony_q1 +
                         mahony_q2 * mahony_q2 + mahony_q3 * mahony_q3);
    if (q_norm > 1e-6f) {
        mahony_q0 /= q_norm;
        mahony_q1 /= q_norm;
        mahony_q2 /= q_norm;
        mahony_q3 /= q_norm;
    }
}

static void mahony_update_euler(void)
{
    mahony_roll_deg = atan2f(2.0f * (mahony_q0 * mahony_q1 + mahony_q2 * mahony_q3),
                             1.0f - 2.0f * (mahony_q1 * mahony_q1 + mahony_q2 * mahony_q2)) *
                      DEG_PER_RAD;
    float sinp = 2.0f * (mahony_q0 * mahony_q2 - mahony_q3 * mahony_q1);
    if (sinp > 1.0f) {
        sinp = 1.0f;
    } else if (sinp < -1.0f) {
        sinp = -1.0f;
    }
    mahony_pitch_deg = asinf(sinp) * DEG_PER_RAD;
    mahony_yaw_deg = atan2f(2.0f * (mahony_q0 * mahony_q3 + mahony_q1 * mahony_q2),
                            1.0f - 2.0f * (mahony_q2 * mahony_q2 + mahony_q3 * mahony_q3)) *
                     DEG_PER_RAD;
    if (mahony_yaw_deg < 0.0f) {
        mahony_yaw_deg += 360.0f;
    }
}

static bool mahony_update(const imu_data_t *imu, float mx, float my, float mz, float yaw_seed)
{
    float ax = imu->ax;
    float ay = imu->ay;
    float az = imu->az;
    if (!normalize3(&ax, &ay, &az) || !normalize3(&mx, &my, &mz)) {
        return false;
    }

    if (!mahony_initialized) {
        mahony_set_from_euler(roll_deg, pitch_deg, yaw_seed);
        mahony_update_euler();
        last_mahony_us = imu->timestamp_us;
        mahony_initialized = true;
        return true;
    }

    float dt_s = (imu->timestamp_us - last_mahony_us) / 1000000.0f;
    last_mahony_us = imu->timestamp_us;
    if (dt_s <= 0.0f || dt_s > 1.0f) {
        dt_s = LOOP_PERIOD_MS / 1000.0f;
    }

    float q0q0 = mahony_q0 * mahony_q0;
    float q0q1 = mahony_q0 * mahony_q1;
    float q0q2 = mahony_q0 * mahony_q2;
    float q0q3 = mahony_q0 * mahony_q3;
    float q1q1 = mahony_q1 * mahony_q1;
    float q1q2 = mahony_q1 * mahony_q2;
    float q1q3 = mahony_q1 * mahony_q3;
    float q2q2 = mahony_q2 * mahony_q2;
    float q2q3 = mahony_q2 * mahony_q3;
    float q3q3 = mahony_q3 * mahony_q3;

    float hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) +
                       my * (q1q2 - q0q3) +
                       mz * (q1q3 + q0q2));
    float hy = 2.0f * (mx * (q1q2 + q0q3) +
                       my * (0.5f - q1q1 - q3q3) +
                       mz * (q2q3 - q0q1));
    float bx = sqrtf(hx * hx + hy * hy);
    float bz = 2.0f * (mx * (q1q3 - q0q2) +
                       my * (q2q3 + q0q1) +
                       mz * (0.5f - q1q1 - q2q2));

    float halfvx = q1q3 - q0q2;
    float halfvy = q0q1 + q2q3;
    float halfvz = q0q0 - 0.5f + q3q3;
    float halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    float halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    float halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    float halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    float halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    float halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    float gx = imu->gx * RAD_PER_DEG;
    float gy = imu->gy * RAD_PER_DEG;
    float gz = imu->gz * RAD_PER_DEG;

    if (MAHONY_TWO_KI > 0.0f) {
        mahony_integral_x += MAHONY_TWO_KI * halfex * dt_s;
        mahony_integral_y += MAHONY_TWO_KI * halfey * dt_s;
        mahony_integral_z += MAHONY_TWO_KI * halfez * dt_s;
        gx += mahony_integral_x;
        gy += mahony_integral_y;
        gz += mahony_integral_z;
    } else {
        mahony_integral_x = 0.0f;
        mahony_integral_y = 0.0f;
        mahony_integral_z = 0.0f;
    }

    gx += MAHONY_TWO_KP * halfex;
    gy += MAHONY_TWO_KP * halfey;
    gz += MAHONY_TWO_KP * halfez;

    gx *= 0.5f * dt_s;
    gy *= 0.5f * dt_s;
    gz *= 0.5f * dt_s;

    float qa = mahony_q0;
    float qb = mahony_q1;
    float qc = mahony_q2;
    mahony_q0 += -qb * gx - qc * gy - mahony_q3 * gz;
    mahony_q1 += qa * gx + qc * gz - mahony_q3 * gy;
    mahony_q2 += qa * gy - qb * gz + mahony_q3 * gx;
    mahony_q3 += qa * gz + qb * gy - qc * gx;

    float q_norm = sqrtf(mahony_q0 * mahony_q0 + mahony_q1 * mahony_q1 +
                         mahony_q2 * mahony_q2 + mahony_q3 * mahony_q3);
    if (!isfinite(q_norm) || q_norm <= 1e-6f) {
        mahony_initialized = false;
        return false;
    }
    mahony_q0 /= q_norm;
    mahony_q1 /= q_norm;
    mahony_q2 /= q_norm;
    mahony_q3 /= q_norm;
    mahony_update_euler();
    return true;
}

static esp_err_t hmc_init(void)
{
    ESP_RETURN_ON_ERROR(add_i2c_device(HMC_ADDR, &hmc_dev), TAG, "add HMC failed");

    uint8_t id[3] = {0};
    esp_err_t err = dev_read(hmc_dev, HMC_REG_IDA, id, sizeof(id));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "HMC not found at 0x%02X: %s", HMC_ADDR, esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "HMC ID bytes: 0x%02X 0x%02X 0x%02X", id[0], id[1], id[2]);
    if (id[0] != 'H' || id[1] != '4' || id[2] != '3') {
        ESP_LOGW(TAG, "HMC ID is not H43; check whether the module is QMC5883L-compatible");
    }

    ESP_RETURN_ON_ERROR(dev_write_u8(hmc_dev, HMC_REG_CRA, 0x18), TAG, "HMC CRA failed");
    ESP_RETURN_ON_ERROR(dev_write_u8(hmc_dev, HMC_REG_CRB, 0x20), TAG, "HMC CRB failed");
    ESP_RETURN_ON_ERROR(dev_write_u8(hmc_dev, HMC_REG_MODE, 0x00), TAG, "HMC mode failed");
    has_hmc = true;
    return ESP_OK;
}

static esp_err_t hmc_read_sample(mag_data_t *out)
{
    uint8_t status = 0;
    ESP_RETURN_ON_ERROR(dev_read(hmc_dev, HMC_REG_SR, &status, 1), TAG, "read HMC status failed");
    if ((status & 0x01) == 0) {
        return ESP_ERR_NOT_FINISHED;
    }

    uint8_t raw[6];
    ESP_RETURN_ON_ERROR(dev_read(hmc_dev, HMC_REG_DOUT, raw, sizeof(raw)), TAG, "read HMC sample failed");

    int16_t rx = be_i16(&raw[0]);
    int16_t rz = be_i16(&raw[2]);
    int16_t ry = be_i16(&raw[4]);
    if (rx == -4096 || ry == -4096 || rz == -4096) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    out->timestamp_us = esp_timer_get_time();
    out->bx = ((float)rx / HMC_GAIN_LSB_PER_GAUSS) * 100.0f;
    out->by = ((float)ry / HMC_GAIN_LSB_PER_GAUSS) * 100.0f;
    out->bz = ((float)rz / HMC_GAIN_LSB_PER_GAUSS) * 100.0f;
    out->magnitude = sqrtf(out->bx * out->bx + out->by * out->by + out->bz * out->bz);
    return ESP_OK;
}

static void mag_calibrate(float bx, float by, float bz, float *bx_cal, float *by_cal, float *bz_cal)
{
    float m0 = bx - MAG_C[0];
    float m1 = by - MAG_C[1];
    float m2 = bz - MAG_C[2];

    *bx_cal = MAG_W[0][0] * m0 + MAG_W[0][1] * m1 + MAG_W[0][2] * m2;
    *by_cal = MAG_W[1][0] * m0 + MAG_W[1][1] * m1 + MAG_W[1][2] * m2;
    *bz_cal = MAG_W[2][0] * m0 + MAG_W[2][1] * m1 + MAG_W[2][2] * m2;
}

static float compute_yaw(float bx, float by)
{
    float yaw = atan2f(-by, bx) * DEG_PER_RAD;
    if (yaw < 0.0f) {
        yaw += 360.0f;
    }
    return yaw;
}

static float compute_tilt_compensated_yaw(float bx, float by, float bz)
{
    float roll = roll_deg / DEG_PER_RAD;
    float pitch = pitch_deg / DEG_PER_RAD;

    float bx_h = bx * cosf(pitch) +
                 by * sinf(roll) * sinf(pitch) +
                 bz * cosf(roll) * sinf(pitch);
    float by_h = by * cosf(roll) - bz * sinf(roll);
    return compute_yaw(bx_h, by_h);
}

static esp_err_t bmp_add_and_probe(uint8_t addr)
{
    i2c_master_dev_handle_t candidate = NULL;
    ESP_RETURN_ON_ERROR(add_i2c_device(addr, &candidate), TAG, "add BMP candidate failed");
    uint8_t id = 0;
    esp_err_t err = dev_read(candidate, BMP280_REG_ID, &id, 1);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(candidate);
        return err;
    }
    if (id != 0x58 && id != 0x60) {
        ESP_LOGW(TAG, "Device at 0x%02X has BMP/BME ID 0x%02X, expected 0x58 or 0x60", addr, id);
        i2c_master_bus_rm_device(candidate);
        return ESP_ERR_NOT_FOUND;
    }

    bmp_dev = candidate;
    bmp_addr = addr;
    ESP_LOGI(TAG, "BMP/BME detected at 0x%02X, chip_id=0x%02X", bmp_addr, id);
    return ESP_OK;
}

static esp_err_t bmp_read_calibration(void)
{
    uint8_t buf[24];
    ESP_RETURN_ON_ERROR(dev_read(bmp_dev, BMP280_REG_CALIB, buf, sizeof(buf)), TAG, "read BMP calib failed");

    bmp_calib.dig_T1 = le_u16(&buf[0]);
    bmp_calib.dig_T2 = le_i16(&buf[2]);
    bmp_calib.dig_T3 = le_i16(&buf[4]);
    bmp_calib.dig_P1 = le_u16(&buf[6]);
    bmp_calib.dig_P2 = le_i16(&buf[8]);
    bmp_calib.dig_P3 = le_i16(&buf[10]);
    bmp_calib.dig_P4 = le_i16(&buf[12]);
    bmp_calib.dig_P5 = le_i16(&buf[14]);
    bmp_calib.dig_P6 = le_i16(&buf[16]);
    bmp_calib.dig_P7 = le_i16(&buf[18]);
    bmp_calib.dig_P8 = le_i16(&buf[20]);
    bmp_calib.dig_P9 = le_i16(&buf[22]);

    ESP_LOGI(TAG, "BMP calib T1=%u T2=%d T3=%d P1=%u",
             bmp_calib.dig_T1, bmp_calib.dig_T2, bmp_calib.dig_T3, bmp_calib.dig_P1);
    return ESP_OK;
}

static esp_err_t bmp_init(void)
{
    esp_err_t err = bmp_add_and_probe(BMP280_ADDR_PRIMARY);
    if (err != ESP_OK) {
        err = bmp_add_and_probe(BMP280_ADDR_SECONDARY);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BMP280/BME280 not found at 0x76 or 0x77");
        return err;
    }

    ESP_RETURN_ON_ERROR(dev_write_u8(bmp_dev, BMP280_REG_RESET, 0xB6), TAG, "BMP reset failed");
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_RETURN_ON_ERROR(bmp_read_calibration(), TAG, "BMP calibration failed");
    ESP_RETURN_ON_ERROR(dev_write_u8(bmp_dev, BMP280_REG_CONFIG, 0x10), TAG, "BMP config failed");
    ESP_RETURN_ON_ERROR(dev_write_u8(bmp_dev, BMP280_REG_CTRL_MEAS, 0x57), TAG, "BMP ctrl_meas failed");

    has_bmp = true;
    return ESP_OK;
}

static int32_t bmp280_compensate_T(int32_t adc_T)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)bmp_calib.dig_T1 << 1))) *
                    ((int32_t)bmp_calib.dig_T2)) >>
                   11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)bmp_calib.dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)bmp_calib.dig_T1))) >>
                     12) *
                    ((int32_t)bmp_calib.dig_T3)) >>
                   14;
    bmp_t_fine = var1 + var2;
    return (bmp_t_fine * 5 + 128) >> 8;
}

static uint32_t bmp280_compensate_P(int32_t adc_P)
{
    int64_t var1 = ((int64_t)bmp_t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)bmp_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp_calib.dig_P3) >> 8) +
           ((var1 * (int64_t)bmp_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp_calib.dig_P1) >> 33;
    if (var1 == 0) {
        return 0;
    }
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp_calib.dig_P7) << 4);
    return (uint32_t)p;
}

static float pressure_to_altitude(float pressure_pa, float sea_level_pa)
{
    if (pressure_pa <= 0.0f || sea_level_pa <= 0.0f) {
        return NAN;
    }
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.1903f));
}

static esp_err_t bmp_read_sample(baro_data_t *out)
{
    uint8_t buf[6];
    ESP_RETURN_ON_ERROR(dev_read(bmp_dev, BMP280_REG_PRESS_MSB, buf, sizeof(buf)), TAG, "read BMP data failed");

    int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((int32_t)buf[2] >> 4);
    int32_t adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | ((int32_t)buf[5] >> 4);

    int32_t temp_x100 = bmp280_compensate_T(adc_T);
    uint32_t pressure_q24_8 = bmp280_compensate_P(adc_P);
    out->timestamp_us = esp_timer_get_time();
    out->temperature_c = temp_x100 / 100.0f;
    out->pressure_pa = pressure_q24_8 / 256.0f;
    out->altitude_m = pressure_to_altitude(out->pressure_pa, p0_pa);
    return ESP_OK;
}

static void bmp_set_initial_p0(void)
{
    if (!has_bmp) {
        return;
    }

    float sum = 0.0f;
    int count = 0;
    for (int i = 0; i < 20; i++) {
        baro_data_t sample;
        if (bmp_read_sample(&sample) == ESP_OK && sample.pressure_pa > 0.0f) {
            sum += sample.pressure_pa;
            count++;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (count > 0) {
        p0_pa = sum / count;
        ESP_LOGI(TAG, "BMP relative altitude reference P0=%.2f Pa", p0_pa);
    }
}

static void nvs_init_for_logs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err == ESP_OK) {
        nvs_ready = true;
    } else {
        ESP_LOGW(TAG, "NVS init failed: %s", esp_err_to_name(err));
    }
}

static esp_err_t bmp_log_open(nvs_open_mode_t mode, nvs_handle_t *handle)
{
    if (!nvs_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    return nvs_open("bmplog", mode, handle);
}

static uint32_t bmp_log_clamp_seconds(uint32_t seconds)
{
    uint32_t max_seconds = (BMP_LOG_MAX_SAMPLES * BMP_LOG_INTERVAL_MS) / 1000;
    if (seconds == 0) {
        return BMP_LOG_DEFAULT_SECONDS;
    }
    if (seconds > max_seconds) {
        return max_seconds;
    }
    return seconds;
}

static void bmp_log_print_status(void)
{
    nvs_handle_t nvs;
    uint8_t armed = 0;
    bmp_log_meta_t meta = {0};
    size_t meta_len = sizeof(meta);
    if (bmp_log_open(NVS_READONLY, &nvs) != ESP_OK) {
        printf("BMPLOG_STATUS,nvs=0,armed=0,stored=0,count=0\n");
        return;
    }
    (void)nvs_get_u8(nvs, "armed", &armed);
    esp_err_t meta_err = nvs_get_blob(nvs, "meta", &meta, &meta_len);
    nvs_close(nvs);

    bool stored = (meta_err == ESP_OK && meta_len == sizeof(meta) && meta.magic == BMP_LOG_MAGIC && meta.count > 0);
    printf("BMPLOG_STATUS,nvs=1,armed=%u,stored=%u,count=%lu,duration_s=%lu\n",
           armed, stored ? 1 : 0, stored ? (unsigned long)meta.count : 0UL,
           stored ? (unsigned long)(meta.duration_ms / 1000) : 0UL);
}

static esp_err_t bmp_log_arm(uint32_t seconds)
{
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(bmp_log_open(NVS_READWRITE, &nvs), TAG, "open BMP log NVS failed");

    uint32_t clamped = bmp_log_clamp_seconds(seconds);
    uint32_t duration_ms = clamped * 1000;
    uint32_t sequence = 0;
    (void)nvs_get_u32(nvs, "sequence", &sequence);
    sequence++;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_u8(nvs, "armed", 1));
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_u32(nvs, "duration", duration_ms));
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_u32(nvs, "sequence", sequence));
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_erase_key(nvs, "meta"));
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_erase_key(nvs, "samples"));
    esp_err_t err = nvs_commit(nvs);
    nvs_close(nvs);

    if (err == ESP_OK) {
        printf("BMPLOG_ARMED,seconds=%lu,sequence=%lu,next_boot=1\n",
               (unsigned long)clamped, (unsigned long)sequence);
    }
    return err;
}

static void bmp_log_clear(void)
{
    nvs_handle_t nvs;
    if (bmp_log_open(NVS_READWRITE, &nvs) != ESP_OK) {
        printf("BMPLOG_CLEAR,nvs=0,ok=0\n");
        return;
    }
    (void)nvs_erase_key(nvs, "armed");
    (void)nvs_erase_key(nvs, "duration");
    (void)nvs_erase_key(nvs, "meta");
    (void)nvs_erase_key(nvs, "samples");
    esp_err_t err = nvs_commit(nvs);
    nvs_close(nvs);
    printf("BMPLOG_CLEAR,ok=%u\n", err == ESP_OK ? 1 : 0);
}

static void bmp_log_dump(void)
{
    nvs_handle_t nvs;
    bmp_log_meta_t meta = {0};
    size_t meta_len = sizeof(meta);
    size_t sample_len = sizeof(bmp_log_buffer);
    if (bmp_log_open(NVS_READONLY, &nvs) != ESP_OK) {
        printf("BMPLOG_DUMP,nvs=0,ok=0\n");
        return;
    }

    esp_err_t meta_err = nvs_get_blob(nvs, "meta", &meta, &meta_len);
    esp_err_t sample_err = nvs_get_blob(nvs, "samples", bmp_log_buffer, &sample_len);
    nvs_close(nvs);

    if (meta_err != ESP_OK || sample_err != ESP_OK || meta.magic != BMP_LOG_MAGIC ||
        meta_len != sizeof(meta) || sample_len < meta.count * sizeof(bmp_log_sample_t)) {
        printf("BMPLOG_DUMP,ok=0,count=0\n");
        return;
    }

    printf("BMPLOG_HEADER,sequence,count,interval_ms,duration_ms,p0_pa\n");
    printf("BMPLOG_META,%lu,%lu,%lu,%lu,%.2f\n",
           (unsigned long)meta.sequence, (unsigned long)meta.count,
           (unsigned long)meta.interval_ms, (unsigned long)meta.duration_ms, meta.p0_pa);
    printf("BMPLOG_CSV_HEADER,t_s,bmp_temp_c,pressure_pa,altitude_m\n");
    for (uint32_t i = 0; i < meta.count; i++) {
        const bmp_log_sample_t *s = &bmp_log_buffer[i];
        printf("BMPLOG,%.3f,%.2f,%.2f,%.2f\n",
               s->t_ms / 1000.0f,
               s->temp_c_x100 / 100.0f,
               s->pressure_pa_x100 / 100.0f,
               s->altitude_cm / 100.0f);
    }
    printf("BMPLOG_END,count=%lu\n", (unsigned long)meta.count);
}

static void bmp_log_capture_if_armed(void)
{
    if (!has_bmp || !nvs_ready) {
        return;
    }

    nvs_handle_t nvs;
    uint8_t armed = 0;
    uint32_t duration_ms = BMP_LOG_DEFAULT_SECONDS * 1000;
    uint32_t sequence = 0;
    if (bmp_log_open(NVS_READWRITE, &nvs) != ESP_OK) {
        return;
    }
    (void)nvs_get_u8(nvs, "armed", &armed);
    (void)nvs_get_u32(nvs, "duration", &duration_ms);
    (void)nvs_get_u32(nvs, "sequence", &sequence);
    if (!armed) {
        nvs_close(nvs);
        return;
    }

    duration_ms = bmp_log_clamp_seconds(duration_ms / 1000) * 1000;
    uint32_t target_count = duration_ms / BMP_LOG_INTERVAL_MS;
    if (target_count > BMP_LOG_MAX_SAMPLES) {
        target_count = BMP_LOG_MAX_SAMPLES;
    }

    printf("BMPLOG_CAPTURE_BEGIN,seconds=%lu,interval_ms=%lu,max_rows=%lu\n",
           (unsigned long)(duration_ms / 1000), (unsigned long)BMP_LOG_INTERVAL_MS,
           (unsigned long)target_count);
    printf("BMPLOG_HINT,keep the board at the low reference for first 10 seconds, then move to the target height.\n");

    bmp_set_initial_p0();
    int64_t start_us = esp_timer_get_time();
    uint32_t count = 0;
    while (count < target_count) {
        baro_data_t sample;
        if (bmp_read_sample(&sample) == ESP_OK && isfinite(sample.pressure_pa) && isfinite(sample.altitude_m)) {
            bmp_log_sample_t *dst = &bmp_log_buffer[count];
            dst->t_ms = (uint32_t)((sample.timestamp_us - start_us) / 1000);
            dst->pressure_pa_x100 = (int32_t)lroundf(sample.pressure_pa * 100.0f);
            dst->temp_c_x100 = (int16_t)lroundf(sample.temperature_c * 100.0f);
            dst->altitude_cm = (int16_t)lroundf(sample.altitude_m * 100.0f);
            count++;
            if (count % 20 == 0) {
                printf("BMPLOG_CAPTURE_PROGRESS,count=%lu,remaining_s=%lu,altitude_m=%.2f\n",
                       (unsigned long)count,
                       (unsigned long)((target_count - count) * BMP_LOG_INTERVAL_MS / 1000),
                       sample.altitude_m);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(BMP_LOG_INTERVAL_MS));
    }

    bmp_log_meta_t meta = {
        .magic = BMP_LOG_MAGIC,
        .sequence = sequence,
        .count = count,
        .interval_ms = BMP_LOG_INTERVAL_MS,
        .duration_ms = duration_ms,
        .p0_pa = p0_pa,
    };

    esp_err_t err = nvs_set_blob(nvs, "samples", bmp_log_buffer, count * sizeof(bmp_log_sample_t));
    if (err == ESP_OK) {
        err = nvs_set_blob(nvs, "meta", &meta, sizeof(meta));
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "armed", 0);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);

    printf("BMPLOG_CAPTURE_END,ok=%u,count=%lu\n", err == ESP_OK ? 1 : 0, (unsigned long)count);
}

static void bmp_log_handle_command(const char *cmd)
{
    if (gps_log_handle_command(cmd)) {
        return;
    }
    if (strncmp(cmd, "BMPLOG_ARM", 10) == 0) {
        uint32_t seconds = BMP_LOG_DEFAULT_SECONDS;
        const char *comma = strchr(cmd, ',');
        if (comma != NULL) {
            seconds = (uint32_t)atoi(comma + 1);
        }
        if (bmp_log_arm(seconds) != ESP_OK) {
            printf("BMPLOG_ARMED,ok=0\n");
        }
    } else if (strcmp(cmd, "BMPLOG_DUMP") == 0) {
        bmp_log_dump();
    } else if (strcmp(cmd, "BMPLOG_CLEAR") == 0) {
        bmp_log_clear();
    } else if (strcmp(cmd, "BMPLOG_STATUS") == 0) {
        bmp_log_print_status();
    }
}

static void log_command_feed_char(char ch)
{
    if (ch == '\r' || ch == '\n') {
        if (bmp_log_cmd_len > 0) {
            bmp_log_cmd_buf[bmp_log_cmd_len] = '\0';
            bmp_log_handle_command(bmp_log_cmd_buf);
            bmp_log_cmd_len = 0;
        }
    } else if (bmp_log_cmd_len + 1 < sizeof(bmp_log_cmd_buf)) {
        bmp_log_cmd_buf[bmp_log_cmd_len++] = ch;
    } else {
        bmp_log_cmd_len = 0;
    }
}

static void bmp_log_poll_uart_commands(void)
{
    uint8_t data[64];
    int len = uart_read_bytes(UART_NUM_0, data, sizeof(data), 0);
    for (int i = 0; i < len; i++) {
        log_command_feed_char((char)data[i]);
    }

    len = usb_serial_jtag_read_bytes(data, sizeof(data), 0);
    for (int i = 0; i < len; i++) {
        log_command_feed_char((char)data[i]);
    }
}

static esp_err_t gps_log_open(nvs_open_mode_t mode, nvs_handle_t *handle)
{
    if (!nvs_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    return nvs_open("gpslog", mode, handle);
}

static uint32_t gps_log_clamp_seconds(uint32_t seconds)
{
    if (seconds == 0) {
        return GPS_LOG_DEFAULT_SECONDS;
    }
    if (seconds > GPS_LOG_MAX_SAMPLES) {
        return GPS_LOG_MAX_SAMPLES;
    }
    return seconds;
}

static uint32_t gps_parse_utc_ms(const char *utc)
{
    if (!utc || strlen(utc) < 6) {
        return 0;
    }
    int hh = (utc[0] - '0') * 10 + (utc[1] - '0');
    int mm = (utc[2] - '0') * 10 + (utc[3] - '0');
    int ss = (utc[4] - '0') * 10 + (utc[5] - '0');
    int ms = 0;
    const char *dot = strchr(utc, '.');
    if (dot) {
        int scale = 100;
        for (const char *p = dot + 1; *p >= '0' && *p <= '9' && scale > 0; p++, scale /= 10) {
            ms += (*p - '0') * scale;
        }
    }
    return (uint32_t)(((hh * 60 + mm) * 60 + ss) * 1000 + ms);
}

static void gps_log_store_gga(const char *utc, int quality, float lat, float lon,
                              float altitude_m, int sats, float hdop, bool checksum_ok)
{
    if (!gps_log_capture_active || gps_log_capture_count >= GPS_LOG_MAX_SAMPLES) {
        return;
    }
    if (quality <= 0 || !isfinite(lat) || !isfinite(lon)) {
        return;
    }

    gps_log_sample_t *dst = &gps_log_buffer[gps_log_capture_count++];
    dst->t_ms = (uint32_t)((esp_timer_get_time() - gps_log_capture_start_us) / 1000);
    dst->utc_ms = gps_parse_utc_ms(utc);
    dst->lat_e7 = (int32_t)lroundf(lat * 10000000.0f);
    dst->lon_e7 = (int32_t)lroundf(lon * 10000000.0f);
    dst->altitude_cm = isfinite(altitude_m) ? (int32_t)lroundf(altitude_m * 100.0f) : 0;
    dst->baro_pressure_pa_x100 = 0;
    dst->baro_temp_c_x100 = 0;
    dst->baro_altitude_cm = 0;
    if (has_bmp) {
        baro_data_t baro;
        if (bmp_read_sample(&baro) == ESP_OK) {
            if (isfinite(baro.pressure_pa)) {
                dst->baro_pressure_pa_x100 = (int32_t)lroundf(baro.pressure_pa * 100.0f);
            }
            if (isfinite(baro.temperature_c)) {
                dst->baro_temp_c_x100 = (int16_t)lroundf(baro.temperature_c * 100.0f);
            }
            if (isfinite(baro.altitude_m)) {
                dst->baro_altitude_cm = (int16_t)lroundf(baro.altitude_m * 100.0f);
            }
        }
    }
    dst->speed_kn_x100 = gps_latest_speed_kn_x100;
    dst->course_deg_x100 = gps_latest_course_deg_x100;
    dst->hdop_x100 = isfinite(hdop) ? (uint16_t)lroundf(hdop * 100.0f) : 0;
    dst->satellites = sats > 0 ? (uint8_t)sats : 0;
    dst->quality = (uint8_t)quality;
    dst->checksum_ok = checksum_ok ? 1 : 0;
}

static void gps_log_print_status(void)
{
    nvs_handle_t nvs;
    uint8_t armed = 0;
    gps_log_meta_t meta = {0};
    size_t meta_len = sizeof(meta);
    if (gps_log_open(NVS_READONLY, &nvs) != ESP_OK) {
        printf("GPSLOG_STATUS,nvs=0,armed=0,stored=0,count=0\n");
        return;
    }
    (void)nvs_get_u8(nvs, "armed", &armed);
    esp_err_t meta_err = nvs_get_blob(nvs, "meta", &meta, &meta_len);
    nvs_close(nvs);

    bool stored = (meta_err == ESP_OK && meta_len == sizeof(meta) && meta.magic == GPS_LOG_MAGIC && meta.count > 0);
    printf("GPSLOG_STATUS,nvs=1,armed=%u,stored=%u,count=%lu,duration_s=%lu\n",
           armed, stored ? 1 : 0, stored ? (unsigned long)meta.count : 0UL,
           stored ? (unsigned long)(meta.duration_ms / 1000) : 0UL);
}

static esp_err_t gps_log_arm(uint32_t seconds)
{
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(gps_log_open(NVS_READWRITE, &nvs), TAG, "open GPS log NVS failed");

    uint32_t clamped = gps_log_clamp_seconds(seconds);
    uint32_t sequence = 0;
    (void)nvs_get_u32(nvs, "sequence", &sequence);
    sequence++;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_u8(nvs, "armed", 1));
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_u32(nvs, "duration", clamped * 1000));
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_u32(nvs, "sequence", sequence));
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_erase_key(nvs, "meta"));
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_erase_key(nvs, "samples"));
    esp_err_t err = nvs_commit(nvs);
    nvs_close(nvs);

    printf("GPSLOG_ARMED,seconds=%lu,sequence=%lu,next_boot=1\n",
           (unsigned long)clamped, (unsigned long)sequence);
    return err;
}

static void gps_log_clear(void)
{
    nvs_handle_t nvs;
    if (gps_log_open(NVS_READWRITE, &nvs) != ESP_OK) {
        printf("GPSLOG_CLEAR,nvs=0,ok=0\n");
        return;
    }
    (void)nvs_erase_key(nvs, "armed");
    (void)nvs_erase_key(nvs, "duration");
    (void)nvs_erase_key(nvs, "meta");
    (void)nvs_erase_key(nvs, "samples");
    esp_err_t err = nvs_commit(nvs);
    nvs_close(nvs);
    printf("GPSLOG_CLEAR,ok=%u\n", err == ESP_OK ? 1 : 0);
}

static void gps_log_dump(void)
{
    nvs_handle_t nvs;
    gps_log_meta_t meta = {0};
    size_t meta_len = sizeof(meta);
    size_t sample_len = sizeof(gps_log_buffer);
    if (gps_log_open(NVS_READONLY, &nvs) != ESP_OK) {
        printf("GPSLOG_DUMP,nvs=0,ok=0\n");
        return;
    }
    esp_err_t meta_err = nvs_get_blob(nvs, "meta", &meta, &meta_len);
    esp_err_t sample_err = nvs_get_blob(nvs, "samples", gps_log_buffer, &sample_len);
    nvs_close(nvs);

    if (meta_err != ESP_OK || sample_err != ESP_OK || meta.magic != GPS_LOG_MAGIC ||
        meta_len != sizeof(meta) || sample_len < meta.count * sizeof(gps_log_sample_t)) {
        printf("GPSLOG_DUMP,ok=0,count=0\n");
        return;
    }

    printf("GPSLOG_HEADER,sequence,count,duration_ms\n");
    printf("GPSLOG_META,%lu,%lu,%lu\n",
           (unsigned long)meta.sequence, (unsigned long)meta.count, (unsigned long)meta.duration_ms);
    printf("GPSLOG_CSV_HEADER,t_s,utc_ms,lat_deg,lon_deg,altitude_m,baro_altitude_m,pressure_pa,bmp_temp_c,speed_kn,course_deg,satellites,hdop,quality,checksum_ok\n");
    for (uint32_t i = 0; i < meta.count; i++) {
        const gps_log_sample_t *s = &gps_log_buffer[i];
        printf("GPSLOG,%.3f,%lu,%.8f,%.8f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%.2f,%u,%u\n",
               s->t_ms / 1000.0f,
               (unsigned long)s->utc_ms,
               s->lat_e7 / 10000000.0f,
               s->lon_e7 / 10000000.0f,
               s->altitude_cm / 100.0f,
               s->baro_altitude_cm / 100.0f,
               s->baro_pressure_pa_x100 / 100.0f,
               s->baro_temp_c_x100 / 100.0f,
               s->speed_kn_x100 / 100.0f,
               s->course_deg_x100 / 100.0f,
               s->satellites,
               s->hdop_x100 / 100.0f,
               s->quality,
               s->checksum_ok);
    }
    printf("GPSLOG_END,count=%lu\n", (unsigned long)meta.count);
}

static void gps_log_capture_if_armed(void)
{
    if (!has_gps_uart || !nvs_ready) {
        return;
    }

    nvs_handle_t nvs;
    uint8_t armed = 0;
    uint32_t duration_ms = GPS_LOG_DEFAULT_SECONDS * 1000;
    uint32_t sequence = 0;
    if (gps_log_open(NVS_READWRITE, &nvs) != ESP_OK) {
        return;
    }
    (void)nvs_get_u8(nvs, "armed", &armed);
    (void)nvs_get_u32(nvs, "duration", &duration_ms);
    (void)nvs_get_u32(nvs, "sequence", &sequence);
    if (!armed) {
        nvs_close(nvs);
        return;
    }

    nvs_close(nvs);
    printf("GPSLOG_CAPTURE_PENDING,seconds=%lu,cancel_window_s=10\n",
           (unsigned long)gps_log_clamp_seconds(duration_ms / 1000));
    int64_t wait_until_us = esp_timer_get_time() + 10000000LL;
    while (esp_timer_get_time() < wait_until_us) {
        bmp_log_poll_uart_commands();
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    armed = 0;
    if (gps_log_open(NVS_READWRITE, &nvs) != ESP_OK) {
        return;
    }
    (void)nvs_get_u8(nvs, "armed", &armed);
    (void)nvs_get_u32(nvs, "duration", &duration_ms);
    (void)nvs_get_u32(nvs, "sequence", &sequence);
    if (!armed) {
        nvs_close(nvs);
        printf("GPSLOG_CAPTURE_CANCELLED\n");
        return;
    }

    duration_ms = gps_log_clamp_seconds(duration_ms / 1000) * 1000;
    memset(gps_log_buffer, 0, sizeof(gps_log_buffer));
    gps_log_capture_count = 0;
    gps_log_capture_active = true;
    gps_log_capture_start_us = esp_timer_get_time();

    printf("GPSLOG_CAPTURE_BEGIN,seconds=%lu,max_rows=%lu\n",
           (unsigned long)(duration_ms / 1000), (unsigned long)GPS_LOG_MAX_SAMPLES);
    printf("GPSLOG_HINT,move outdoors; keep GPS antenna facing up; wait until capture ends before unplugging power.\n");

    int64_t end_us = gps_log_capture_start_us + (int64_t)duration_ms * 1000;
    uint32_t last_progress = 0;
    while (esp_timer_get_time() < end_us && gps_log_capture_count < GPS_LOG_MAX_SAMPLES) {
        gps_poll_uart();
        if (gps_log_capture_count >= last_progress + 20) {
            last_progress = gps_log_capture_count;
            int64_t remaining_us = end_us - esp_timer_get_time();
            printf("GPSLOG_CAPTURE_PROGRESS,count=%lu,remaining_s=%lld\n",
                   (unsigned long)gps_log_capture_count,
                   (long long)(remaining_us > 0 ? remaining_us / 1000000 : 0));
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    gps_log_capture_active = false;

    gps_log_meta_t meta = {
        .magic = GPS_LOG_MAGIC,
        .sequence = sequence,
        .count = gps_log_capture_count,
        .duration_ms = duration_ms,
    };
    esp_err_t err = nvs_set_blob(nvs, "samples", gps_log_buffer, gps_log_capture_count * sizeof(gps_log_sample_t));
    if (err == ESP_OK) {
        err = nvs_set_blob(nvs, "meta", &meta, sizeof(meta));
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "armed", 0);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);

    printf("GPSLOG_CAPTURE_END,ok=%u,count=%lu\n", err == ESP_OK ? 1 : 0, (unsigned long)gps_log_capture_count);
}

static uint32_t parse_command_seconds(const char *cmd, uint32_t fallback)
{
    const char *arg = strchr(cmd, ',');
    if (!arg) {
        arg = strchr(cmd, ' ');
    }
    return arg ? (uint32_t)atoi(arg + 1) : fallback;
}

static bool gps_log_handle_command(const char *cmd)
{
    if (strncmp(cmd, "GPSLOG_ARM", 10) == 0) {
        uint32_t seconds = parse_command_seconds(cmd, GPS_LOG_DEFAULT_SECONDS);
        if (gps_log_arm(seconds) != ESP_OK) {
            printf("GPSLOG_ARMED,ok=0\n");
        }
        return true;
    }
    if (strcmp(cmd, "GPSLOG_DUMP") == 0) {
        gps_log_dump();
        return true;
    }
    if (strcmp(cmd, "GPSLOG_CLEAR") == 0) {
        gps_log_clear();
        return true;
    }
    if (strcmp(cmd, "GPSLOG_STATUS") == 0) {
        gps_log_print_status();
        return true;
    }
    return false;
}

static void gps_sync_task(void *arg)
{
    (void)arg;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint32_t pps_count = 0;
        int64_t pps_us = 0;
        portENTER_CRITICAL(&gps_pps_mux);
        pps_count = gps_pps_count;
        pps_us = gps_last_pps_us;
        portEXIT_CRITICAL(&gps_pps_mux);

        imu_data_t imu = {0};
        mag_data_t mag = {0};
        int64_t imu_start_us = esp_timer_get_time();
        esp_err_t imu_err = has_mpu ? mpu_read_sample(&imu) : ESP_ERR_INVALID_STATE;
        int64_t imu_done_us = esp_timer_get_time();

        int64_t mag_start_us = esp_timer_get_time();
        esp_err_t mag_err = has_hmc ? hmc_read_sample(&mag) : ESP_ERR_INVALID_STATE;
        int64_t mag_done_us = esp_timer_get_time();

        printf("SYNC_SAMPLE,%lu,%lld,%lld,%lld,%lld,%d,%lld,%lld,%lld,%d\n",
               (unsigned long)pps_count,
               (long long)pps_us,
               (long long)imu_start_us,
               (long long)imu_done_us,
               (long long)(imu_start_us - pps_us),
               imu_err == ESP_OK ? 1 : 0,
               (long long)mag_start_us,
               (long long)mag_done_us,
               (long long)(mag_start_us - pps_us),
               mag_err == ESP_OK ? 1 : 0);
    }
}

static void IRAM_ATTR gps_pps_isr_handler(void *arg)
{
    (void)arg;
    int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&gps_pps_mux);
    gps_last_pps_us = now_us;
    gps_pps_count++;
    portEXIT_CRITICAL_ISR(&gps_pps_mux);
    if (gps_sync_task_handle) {
        BaseType_t high_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(gps_sync_task_handle, &high_task_woken);
        if (high_task_woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

static esp_err_t gps_uart_init(void)
{
    const uart_config_t config = {
        .baud_rate = GPS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(GPS_UART_PORT, GPS_UART_BUF_SIZE, 0, 0, NULL, 0),
                        TAG, "GPS UART driver install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(GPS_UART_PORT, &config), TAG, "GPS UART config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(GPS_UART_PORT, GPS_UART_TX_IO, GPS_UART_RX_IO,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "GPS UART pin config failed");

    gpio_config_t pps_config = {
        .pin_bit_mask = 1ULL << GPS_PPS_IO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&pps_config));
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "GPS PPS ISR service install failed: %s", esp_err_to_name(isr_err));
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_isr_handler_add(GPS_PPS_IO, gps_pps_isr_handler, NULL));

    has_gps_uart = true;
    printf("GPS_UART_READY,port=%d,rx_gpio=%d,tx_gpio=%d,baud=%d,pps_gpio=%d\n",
           GPS_UART_PORT, GPS_UART_RX_IO, GPS_UART_TX_IO, GPS_UART_BAUD, GPS_PPS_IO);
    return ESP_OK;
}

static bool gps_is_time_sentence(const char *line)
{
    return strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0 ||
           strncmp(line, "$GNRMC", 6) == 0 || strncmp(line, "$GPRMC", 6) == 0 ||
           strncmp(line, "$GNZDA", 6) == 0 || strncmp(line, "$GPZDA", 6) == 0;
}

static int gps_hex_value(char ch)
{
    if (ch >= '0' && ch <= '9') {
        return ch - '0';
    }
    if (ch >= 'A' && ch <= 'F') {
        return ch - 'A' + 10;
    }
    if (ch >= 'a' && ch <= 'f') {
        return ch - 'a' + 10;
    }
    return -1;
}

static bool gps_checksum_ok(const char *line)
{
    if (!line || line[0] != '$') {
        return false;
    }
    const char *star = strchr(line, '*');
    if (!star || gps_hex_value(star[1]) < 0 || gps_hex_value(star[2]) < 0) {
        return false;
    }
    uint8_t calc = 0;
    for (const char *p = line + 1; p < star; p++) {
        calc ^= (uint8_t)*p;
    }
    uint8_t got = (uint8_t)((gps_hex_value(star[1]) << 4) | gps_hex_value(star[2]));
    return calc == got;
}

static float gps_parse_float_field(const char *text)
{
    if (!text || text[0] == '\0') {
        return NAN;
    }
    return strtof(text, NULL);
}

static int gps_parse_int_field(const char *text)
{
    if (!text || text[0] == '\0') {
        return -1;
    }
    return atoi(text);
}

static float gps_parse_latlon(const char *value, const char *hemi)
{
    if (!value || !hemi || value[0] == '\0' || hemi[0] == '\0') {
        return NAN;
    }
    const char *dot = strchr(value, '.');
    int deg_digits = (dot && dot - value == 4) ? 2 : 3;
    if ((int)strlen(value) <= deg_digits) {
        return NAN;
    }

    char deg_buf[4] = {0};
    memcpy(deg_buf, value, deg_digits);
    float degrees = strtof(deg_buf, NULL);
    float minutes = strtof(value + deg_digits, NULL);
    float coord = degrees + minutes / 60.0f;
    if (hemi[0] == 'S' || hemi[0] == 'W') {
        coord = -coord;
    }
    return coord;
}

static size_t gps_split_fields(char *buf, char *fields[], size_t max_fields)
{
    char *star = strchr(buf, '*');
    if (star) {
        *star = '\0';
    }

    size_t count = 0;
    char *p = buf;
    if (*p == '$') {
        p++;
    }
    while (count < max_fields) {
        fields[count++] = p;
        char *comma = strchr(p, ',');
        if (!comma) {
            break;
        }
        *comma = '\0';
        p = comma + 1;
    }
    return count;
}

static void gps_parse_and_print(const char *line, int64_t rx_done_us)
{
    if (!gps_is_time_sentence(line)) {
        return;
    }

    bool checksum_ok = gps_checksum_ok(line);
    char buf[GPS_NMEA_LINE_MAX];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *fields[20] = {0};
    size_t n = gps_split_fields(buf, fields, 20);
    if (n == 0 || strlen(fields[0]) < 5) {
        return;
    }

    const char *type = fields[0] + strlen(fields[0]) - 3;
    if (strcmp(type, "GGA") == 0 && n >= 10) {
        float lat = gps_parse_latlon(fields[2], fields[3]);
        float lon = gps_parse_latlon(fields[4], fields[5]);
        int quality = gps_parse_int_field(fields[6]);
        int sats = gps_parse_int_field(fields[7]);
        float hdop = gps_parse_float_field(fields[8]);
        float alt = gps_parse_float_field(fields[9]);
        gps_log_store_gga(fields[1], quality, lat, lon, alt, sats, hdop, checksum_ok);
        printf("GPS_GGA,%lld,%s,%s,%d,%.8f,%.8f,%.3f,%d,%.2f,%d\n",
               (long long)rx_done_us, fields[0], fields[1], quality,
               lat, lon, alt, sats, hdop, checksum_ok ? 1 : 0);
    } else if (strcmp(type, "RMC") == 0 && n >= 10) {
        float lat = gps_parse_latlon(fields[3], fields[4]);
        float lon = gps_parse_latlon(fields[5], fields[6]);
        float speed = gps_parse_float_field(fields[7]);
        float course = gps_parse_float_field(fields[8]);
        if (isfinite(speed) && speed >= 0.0f && speed <= 655.0f) {
            gps_latest_speed_kn_x100 = (uint16_t)lroundf(speed * 100.0f);
        }
        if (isfinite(course) && course >= 0.0f && course < 360.0f) {
            gps_latest_course_deg_x100 = (uint16_t)lroundf(course * 100.0f);
        }
        char status = fields[2] && fields[2][0] ? fields[2][0] : 'V';
        printf("GPS_RMC,%lld,%s,%s,%c,%.8f,%.8f,%.3f,%.3f,%s,%d\n",
               (long long)rx_done_us, fields[0], fields[1], status,
               lat, lon, speed, course, fields[9], checksum_ok ? 1 : 0);
    }
}

static void gps_print_line(const char *line, int64_t rx_done_us)
{
    if (line[0] == '$') {
        printf("GPS_RAW,%s\n", line);
        if (gps_is_time_sentence(line)) {
            printf("GPS_NMEA_TS,%lld,%s\n", (long long)rx_done_us, line);
            gps_parse_and_print(line, rx_done_us);
        }
    } else if (line[0] != '\0') {
        printf("GPS_TEXT,%s\n", line);
    }
}

static void gps_print_pps_if_new(void)
{
    uint32_t count;
    int64_t last_us;
    portENTER_CRITICAL(&gps_pps_mux);
    count = gps_pps_count;
    last_us = gps_last_pps_us;
    portEXIT_CRITICAL(&gps_pps_mux);

    if (count == 0 || count == gps_reported_pps_count) {
        return;
    }

    int64_t interval_us = gps_prev_reported_pps_us > 0 ? last_us - gps_prev_reported_pps_us : 0;
    gps_reported_pps_count = count;
    gps_prev_reported_pps_us = last_us;
    printf("GPS_PPS,%lu,%lld,%lld\n",
           (unsigned long)count, (long long)last_us, (long long)interval_us);
}

static void gps_poll_uart(void)
{
    if (!has_gps_uart) {
        return;
    }

    uint8_t data[128];
    int len = uart_read_bytes(GPS_UART_PORT, data, sizeof(data), 0);
    if (len > 0) {
        gps_nmea_bytes += (uint32_t)len;
    }

    for (int i = 0; i < len; i++) {
        char ch = (char)data[i];
        if (ch == '\r' || ch == '\n') {
            if (gps_nmea_len > 0) {
                gps_nmea_line[gps_nmea_len] = '\0';
                gps_print_line(gps_nmea_line, esp_timer_get_time());
                gps_nmea_lines++;
                gps_nmea_len = 0;
            }
        } else if (gps_nmea_len + 1 < sizeof(gps_nmea_line)) {
            gps_nmea_line[gps_nmea_len++] = ch;
        } else {
            gps_nmea_len = 0;
            printf("GPS_WARN,line_too_long\n");
        }
    }

    gps_print_pps_if_new();

    int64_t now_us = esp_timer_get_time();
    if (gps_last_status_us == 0 || now_us - gps_last_status_us >= GPS_STATUS_PERIOD_US) {
        gps_last_status_us = now_us;
        uint32_t pps_count;
        int64_t pps_us;
        portENTER_CRITICAL(&gps_pps_mux);
        pps_count = gps_pps_count;
        pps_us = gps_last_pps_us;
        portEXIT_CRITICAL(&gps_pps_mux);
        printf("GPS_STATUS,uart=%d,bytes=%lu,lines=%lu,pps_level=%d,pps_count=%lu,last_pps_age_us=%lld\n",
               has_gps_uart ? 1 : 0,
               (unsigned long)gps_nmea_bytes,
               (unsigned long)gps_nmea_lines,
               gpio_get_level(GPS_PPS_IO),
               (unsigned long)pps_count,
               pps_us > 0 ? (long long)(now_us - pps_us) : -1LL);
    }
}

static void init_all_sensors(void)
{
    if (mpu_init() != ESP_OK) {
        has_mpu = false;
    }
    if (hmc_init() != ESP_OK) {
        has_hmc = false;
    }
    if (bmp_init() != ESP_OK) {
        has_bmp = false;
    }
    bmp_set_initial_p0();

    printf("SENSOR_STATUS,MPU=%d,HMC=%d,BMP=%d,BMP_ADDR=0x%02X\n",
           has_mpu ? 1 : 0, has_hmc ? 1 : 0, has_bmp ? 1 : 0, bmp_addr);
}

static void print_header(void)
{
    // CSV 表头必须与 printf("SENSOR,...") 字段顺序一致，PC 端脚本按这个表头解析。
    printf("SENSOR_HEADER,"
           "t_s,"
           "ax_raw_g,ay_raw_g,az_raw_g,gx_raw_dps,gy_raw_dps,gz_raw_dps,"
           "ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,mpu_temp_c,"
           "roll_deg,pitch_deg,"
           "bx_uT,by_uT,bz_uT,mag_uT,"
           "bx_cal_uT,by_cal_uT,bz_cal_uT,mag_cal_uT,"
           "yaw_flat_deg,yaw_tilt_deg,"
           "mahony_roll_deg,mahony_pitch_deg,mahony_yaw_deg,"
           "bmp_temp_c,pressure_pa,altitude_m,"
           "imu_dt_us,mag_dt_us,bmp_dt_us,imu_mag_dt_us,imu_bmp_dt_us\n");
    printf("PERF_HEADER,"
           "samples,update_hz,imu_update_hz,comp_update_hz,mahony_update_hz,sensor_print_hz,"
           "loop_avg_us,loop_max_us,loop_cpu_pct,"
           "mpu_avg_us,mpu_max_us,"
           "comp_avg_us,comp_max_us,comp_cpu_pct,"
           "hmc_avg_us,hmc_max_us,"
           "yaw_avg_us,yaw_max_us,yaw_cpu_pct,"
           "mahony_avg_us,mahony_max_us,mahony_cpu_pct,"
           "bmp_avg_us,bmp_max_us,"
           "measured_cpu_pct\n");
    printf("SYNC_HEADER,"
           "pps_count,pps_us,"
           "imu_start_us,imu_done_us,imu_start_delta_us,imu_ok,"
           "mag_start_us,mag_done_us,mag_start_delta_us,mag_ok\n");
}

static void perf_add_pair(int64_t value_us, int64_t *sum_us, int64_t *max_us)
{
    if (value_us < 0) {
        return;
    }
    *sum_us += value_us;
    if (value_us > *max_us) {
        *max_us = value_us;
    }
}

static void perf_record(int64_t loop_us, int64_t mpu_us, int64_t comp_us,
                        int64_t hmc_us, int64_t yaw_us, int64_t mahony_us,
                        int64_t bmp_us, bool imu_updated, bool comp_updated,
                        bool mahony_updated, bool sensor_printed)
{
    int64_t now_us = esp_timer_get_time();
    if (perf_stats.count == 0) {
        perf_stats.window_start_us = now_us;
    }
    perf_stats.window_end_us = now_us;
    perf_stats.count++;
    if (imu_updated) {
        perf_stats.imu_update_count++;
    }
    if (comp_updated) {
        perf_stats.comp_update_count++;
    }
    if (mahony_updated) {
        perf_stats.mahony_update_count++;
    }
    if (sensor_printed) {
        perf_stats.sensor_print_count++;
    }
    perf_add_pair(loop_us, &perf_stats.loop_sum_us, &perf_stats.loop_max_us);
    perf_add_pair(mpu_us, &perf_stats.mpu_sum_us, &perf_stats.mpu_max_us);
    perf_add_pair(comp_us, &perf_stats.comp_sum_us, &perf_stats.comp_max_us);
    perf_add_pair(hmc_us, &perf_stats.hmc_sum_us, &perf_stats.hmc_max_us);
    perf_add_pair(yaw_us, &perf_stats.yaw_sum_us, &perf_stats.yaw_max_us);
    perf_add_pair(mahony_us, &perf_stats.mahony_sum_us, &perf_stats.mahony_max_us);
    perf_add_pair(bmp_us, &perf_stats.bmp_sum_us, &perf_stats.bmp_max_us);
}

static void perf_print_if_ready(void)
{
    if (perf_stats.count < PERF_REPORT_SAMPLES) {
        return;
    }

    float n = (float)perf_stats.count;
    float loop_avg = perf_stats.loop_sum_us / n;
    float comp_avg = perf_stats.comp_sum_us / n;
    float yaw_avg = perf_stats.yaw_sum_us / n;
    float mahony_avg = perf_stats.mahony_sum_us / n;
    float period_us = LOOP_PERIOD_MS * 1000.0f;
    float window_s = (perf_stats.window_end_us - perf_stats.window_start_us) / 1000000.0f;
    if (window_s <= 0.0f) {
        window_s = n * LOOP_PERIOD_MS / 1000.0f;
    }
    float update_hz = n / window_s;
    float imu_update_hz = perf_stats.imu_update_count / window_s;
    float comp_update_hz = perf_stats.comp_update_count / window_s;
    float mahony_update_hz = perf_stats.mahony_update_count / window_s;
    float sensor_print_hz = perf_stats.sensor_print_count / window_s;

    printf("PERF,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,"
           "%.2f,%lld,%.4f,"
           "%.2f,%lld,"
           "%.2f,%lld,%.6f,"
           "%.2f,%lld,"
           "%.2f,%lld,%.6f,"
           "%.2f,%lld,%.6f,"
           "%.2f,%lld,"
           "%.4f\n",
            (unsigned long)perf_stats.count,
            update_hz, imu_update_hz, comp_update_hz, mahony_update_hz, sensor_print_hz,
            loop_avg, (long long)perf_stats.loop_max_us, loop_avg / period_us * 100.0f,
           perf_stats.mpu_sum_us / n, (long long)perf_stats.mpu_max_us,
           comp_avg, (long long)perf_stats.comp_max_us, comp_avg / period_us * 100.0f,
           perf_stats.hmc_sum_us / n, (long long)perf_stats.hmc_max_us,
           yaw_avg, (long long)perf_stats.yaw_max_us, yaw_avg / period_us * 100.0f,
           mahony_avg, (long long)perf_stats.mahony_max_us, mahony_avg / period_us * 100.0f,
           perf_stats.bmp_sum_us / n, (long long)perf_stats.bmp_max_us,
           (comp_avg + yaw_avg + mahony_avg) / period_us * 100.0f);

    memset(&perf_stats, 0, sizeof(perf_stats));
}

void app_main(void)
{
    nvs_init_for_logs();
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0));
    usb_serial_jtag_driver_config_t usb_jtag_config = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 1024,
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(usb_serial_jtag_driver_install(&usb_jtag_config));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gps_uart_init());
    ESP_ERROR_CHECK(i2c_bus_init());
    i2c_scan();
    init_all_sensors();
    if (has_gps_uart && (has_mpu || has_hmc)) {
        BaseType_t ok = xTaskCreate(gps_sync_task, "gps_sync", GPS_SYNC_TASK_STACK, NULL,
                                    GPS_SYNC_TASK_PRIORITY, &gps_sync_task_handle);
        if (ok != pdPASS) {
            gps_sync_task_handle = NULL;
            ESP_LOGW(TAG, "GPS sync task create failed");
        }
    }
    bmp_log_print_status();
    gps_log_print_status();
    bmp_log_capture_if_armed();
    gps_log_capture_if_armed();
    print_header();

    mag_data_t mag = {
        .timestamp_us = 0,
        .bx = NAN,
        .by = NAN,
        .bz = NAN,
        .magnitude = NAN,
    };
    baro_data_t baro = {
        .timestamp_us = 0,
        .temperature_c = NAN,
        .pressure_pa = NAN,
        .altitude_m = NAN,
    };
    float bx_cal = NAN;
    float by_cal = NAN;
    float bz_cal = NAN;
    float mag_cal = NAN;
    float yaw_flat = NAN;
    float yaw_tilt = NAN;
    int64_t last_hmc_read_us = -HMC_READ_PERIOD_MS * 1000LL;
    int64_t last_bmp_read_us = -BMP_READ_PERIOD_MS * 1000LL;
    int64_t last_sensor_print_us = -SENSOR_PRINT_PERIOD_MS * 1000LL;

    while (1) {
        bmp_log_poll_uart_commands();
        gps_poll_uart();
        int64_t loop_start_us = esp_timer_get_time();
        int64_t now_us = loop_start_us;
        int64_t mpu_us = 0;
        int64_t comp_us = 0;
        int64_t hmc_us = 0;
        int64_t yaw_us = 0;
        int64_t mahony_us = 0;
        int64_t bmp_us = 0;
        bool imu_ok = false;
        bool comp_updated = false;
        bool mahony_updated = false;
        bool sensor_printed = false;

        imu_data_t imu = {
            .timestamp_us = now_us,
            .ax_raw = NAN,
            .ay_raw = NAN,
            .az_raw = NAN,
            .gx_raw = NAN,
            .gy_raw = NAN,
            .gz_raw = NAN,
            .ax = NAN,
            .ay = NAN,
            .az = NAN,
            .gx = NAN,
            .gy = NAN,
            .gz = NAN,
            .temp = NAN,
        };

        int64_t stage_start_us = esp_timer_get_time();
        if (has_mpu && mpu_read_sample(&imu) == ESP_OK) {
            mpu_us = esp_timer_get_time() - stage_start_us;
            stage_start_us = esp_timer_get_time();
            complementary_update(&imu);
            comp_us = esp_timer_get_time() - stage_start_us;
            imu_ok = true;
            comp_updated = true;
        } else {
            mpu_us = esp_timer_get_time() - stage_start_us;
        }

        if (has_hmc && (now_us - last_hmc_read_us) >= HMC_READ_PERIOD_MS * 1000LL) {
            stage_start_us = esp_timer_get_time();
            last_hmc_read_us = now_us;
            if (hmc_read_sample(&mag) == ESP_OK) {
                hmc_us = esp_timer_get_time() - stage_start_us;
                mag_calibrate(mag.bx, mag.by, mag.bz, &bx_cal, &by_cal, &bz_cal);
                mag_cal = sqrtf(bx_cal * bx_cal + by_cal * by_cal + bz_cal * bz_cal);
            } else {
                hmc_us = esp_timer_get_time() - stage_start_us;
            }
        }

        if (isfinite(bx_cal) && isfinite(by_cal) && isfinite(bz_cal)) {
            stage_start_us = esp_timer_get_time();
            yaw_flat = compute_yaw(bx_cal, by_cal);
            if (attitude_initialized) {
                yaw_tilt = compute_tilt_compensated_yaw(bx_cal, by_cal, bz_cal);
            }
            yaw_us = esp_timer_get_time() - stage_start_us;
            if (imu_ok && isfinite(yaw_tilt)) {
                stage_start_us = esp_timer_get_time();
                if (mahony_update(&imu, bx_cal, by_cal, bz_cal, yaw_tilt)) {
                    mahony_updated = true;
                }
                mahony_us = esp_timer_get_time() - stage_start_us;
            }
        }

        if (has_bmp && (now_us - last_bmp_read_us) >= BMP_READ_PERIOD_MS * 1000LL) {
            stage_start_us = esp_timer_get_time();
            last_bmp_read_us = now_us;
            (void)bmp_read_sample(&baro);
            bmp_us = esp_timer_get_time() - stage_start_us;
        }

        if ((now_us - last_sensor_print_us) >= SENSOR_PRINT_PERIOD_MS * 1000LL) {
            last_sensor_print_us = now_us;
            sensor_printed = true;
            int64_t imu_dt_us = imu.timestamp_us - loop_start_us;
            int64_t mag_dt_us = mag.timestamp_us - loop_start_us;
            int64_t bmp_dt_us = baro.timestamp_us - loop_start_us;
            int64_t imu_mag_dt_us = mag.timestamp_us - imu.timestamp_us;
            int64_t imu_bmp_dt_us = baro.timestamp_us - imu.timestamp_us;

            printf("SENSOR,%.3f,"
                   "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                   "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,"
                   "%.3f,%.3f,"
                   "%.4f,%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,%.4f,"
                   "%.3f,%.3f,"
                   "%.3f,%.3f,%.3f,"
                   "%.3f,%.2f,%.3f,"
                   "%lld,%lld,%lld,%lld,%lld\n",
                   now_us / 1000000.0,
                   imu.ax_raw, imu.ay_raw, imu.az_raw, imu.gx_raw, imu.gy_raw, imu.gz_raw,
                   imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, imu.temp,
                   roll_deg, pitch_deg,
                   mag.bx, mag.by, mag.bz, mag.magnitude,
                   bx_cal, by_cal, bz_cal, mag_cal,
                   yaw_flat, yaw_tilt,
                   mahony_roll_deg, mahony_pitch_deg, mahony_yaw_deg,
                   baro.temperature_c, baro.pressure_pa, baro.altitude_m,
                   (long long)imu_dt_us, (long long)mag_dt_us, (long long)bmp_dt_us,
                   (long long)imu_mag_dt_us, (long long)imu_bmp_dt_us);
        }

        int64_t loop_process_us = esp_timer_get_time() - loop_start_us;
        int64_t remaining_us = LOOP_PERIOD_MS * 1000LL - loop_process_us;
        if (remaining_us > 0) {
            esp_rom_delay_us((uint32_t)remaining_us);
        }

        perf_record(loop_process_us, mpu_us, comp_us, hmc_us, yaw_us, mahony_us, bmp_us,
                    imu_ok, comp_updated, mahony_updated, sensor_printed);
        perf_print_if_ready();

    }
}
