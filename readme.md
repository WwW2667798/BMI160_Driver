# BMI160 Driver for STM32F1

这是一个基于 STM32F1（如 STM32F103）平台的 BMI160 传感器驱动与示例工程，包含 Bosch 原厂驱动文件、示例硬件接口代码以及简单的滤波工具（如 Mahony滤波/互补滤波 可扩展实现）。

## 项目结构
- `Hardware/`：板级封装与上层封装（例如 `bmi160.c`, `bmi160.h`）。
- `Driver/`：驱动接口实现（`bmi160_driver.c` / `.h`）。
- `Tools/`：工具代码，例如滤波器实现（`filter.c`）。
- `Library/`：STM32 外设库文件（ADC、SPI、I2C 等）。
- `Project.*` / `Start/` / `Objects/`：Keil MDK 工程文件与编译产物。

## 功能
- 与 BMI160 传感器通信（I2C/SPI，可在 `bmi160` 驱动中配置）
- 读取加速度计、陀螺仪、（可选）磁力计数据
- 提供示例滤波接口用于姿态估计

## 硬件连接
- VCC -> 3.3V
- GND -> GND
- SDA -> PB11
- SCL -> PB10
- INT/CS 等按模块需要连接

若使用 SPI，请将对应的 SCK/MISO/MOSI/CS 连接到 MCU 的 SPI 引脚，并在驱动中启用 SPI 接口。

## 快速上手（Keil uVision）
1. 使用 Keil 打开工程文件：`Project.uvprojx`。
2. 配置目标板与时钟，确认 `bmi160` 的总线回调（I2C/SPI）在 `bmi160` 实例中正确设置。
3. 编译并下载到目标板。运行后查看串口或日志输出以验证传感器数据。

## 代码要点
- 驱动入口：`bmi160_init()`（在 `Hardware/bmi160.c`）
- 低层总线需要在项目中实现：`BMI160_BUS_READ_FUNC` / `BMI160_BUS_WRITE_FUNC`（详见 `bmi160.h`）
- 滤波与姿态：`Tools/filter.c` 提供示例卡尔曼滤波函数，可拓展为三轴或替换为 Madgwick/AHRS 实现。

## 使用示例（调用流程，伪代码）
1. 初始化 I2C/SPI 总线和时钟。
2. 初始化传感器：`bmi160_init(&bmi_instance);`。
3. 在定时器或主循环中读取原始数据并送入滤波器计算角度（pitch/roll/yaw）。
