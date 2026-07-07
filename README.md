# sensor-final-project

ESP32-S3 多传感器融合扩展板课程设计项目。项目实现了 MPU6050/MPU6500 兼容 IMU、HMC5883L 磁力计、BMP280/BME280 气压计和 GNSS 模块的数据采集、标定、融合与实验分析。

## 目录说明

```text
sensor-final-project/
├── hardware/                 # 硬件设计文件
│   ├── schematic.pdf          # 原理图 PDF
│   ├── pcb.pdf                # PCB 布局/制板文件说明 PDF
│   ├── gerber/                # Gerber 生产文件
│   └── BOM.csv                # 物料清单
├── firmware/                 # 固件与算法代码
│   ├── drivers/               # 串口记录、离线读取和驱动辅助脚本
│   ├── calibration/           # 加速度计、磁力计、Allan 方差和航向残差标定
│   ├── fusion/                # 互补滤波、Mahony、GNSS/INS、实时姿态显示分析脚本
│   ├── ai_enhance/            # 1D-CNN/卷积去噪与 Kalman 对比
│   └── main.c                 # ESP32-S3 主固件
├── data/                     # 测试数据
│   ├── calibration/           # 标定数据
│   ├── fusion_comparison/     # 融合与算法对比数据
│   └── performance/           # 更新频率、CPU 占用和时间同步数据
├── docs/                     # 文档
│   ├── spec.md                # 系统设计要求
│   ├── test_report.md         # 测试报告
│   ├── final_report.pdf       # 最终课程论文
│   └── final_report.docx      # 可编辑最终课程论文
├── figures/                  # 论文和实验图表
└── README.md                 # 仓库说明
```

## 已完成内容

- 硬件：完成 ESP32-S3 扩展板设计，I2C 总线连接 MPU、HMC5883L 和 BMP280，GNSS 使用 UART 与 PPS 引脚。
- 固件：完成 I2C 扫描、传感器初始化、CSV 串口输出、GPS NMEA 解析、PPS 时间同步记录、BMP/GPS 离线记录。
- 标定：完成加速度计六位置 12 参数标定、磁力计椭球拟合、陀螺仪 Allan 方差分析和温度补偿实验。
- 融合：实现互补滤波和 Mahony 姿态融合，完成实时网页姿态显示。
- AI 增强：完成 IMU 静态数据 1D 卷积去噪，并与传统低通、一阶 Kalman 混合结构进行 SNR 对比。
- 实验：完成标定精度、磁力计 3D 散点、Allan 曲线、BMP 楼梯高度、GNSS NMEA 解析、GNSS/INS 松耦合静态分析和更新频率测试。

## 构建与运行

固件基于 ESP-IDF，目标芯片为 ESP32-S3。

```powershell
cd firmware
idf.py set-target esp32s3
idf.py -p COM7 flash monitor
```

Python 分析脚本位于 `firmware/calibration`、`firmware/fusion`、`firmware/ai_enhance` 和 `firmware/drivers`。原始实验 CSV 位于 `data/`。

## 说明

本仓库是课程提交用整理版。原始 ESP-IDF 工程采用单文件主固件 `main.c`，传感器驱动、标定参数、融合算法和日志命令均集中在该文件中；离线标定和可视化脚本按老师要求拆分到对应目录。
