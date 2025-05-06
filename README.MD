# GUETfly Mini F1 Flight Controller
# GUETfly Mini F1 飞控系统

## Project Overview | 项目概述
GUETfly Mini F1 is an open-source flight controller based on STM32/GD32 microcontroller, derived from the MultiWii project. It provides comprehensive flight control capabilities for multi-rotor aircraft.

GUETfly Mini F1 是一个基于STM32/GD32微控制器的开源飞控系统，源自MultiWii项目。它为多旋翼飞行器提供全面的飞行控制功能。

## Features | 主要特性
- GPS Navigation Support | GPS导航支持
- Waypoint Storage (up to 254 waypoints) | 航点存储（最多254个航点）
- Barometer Support (BMP5611) | 气压计支持（BMP5611）
- Magnetometer Support (AK8963) | 磁力计支持（AK8963）
- Inertial Navigation | 惯性导航
- Multiple UART Support (UART3,4,5) | 多串口支持（串口3、4、5）
- PWM Output (TIME3 for motors, TIME4 for servos) | PWM输出（TIME3用于电机，TIME4用于舵机）

## Hardware | 硬件规格
- Main Controller: STM32/GD32 | 主控制器：STM32/GD32
- PCB Design: GUETflyminiV1.2 | PCB设计：GUETflyminiV1.2
- Pin Assignment: See "guetflymini引脚分配.xlsx" | 引脚分配：详见"guetflymini引脚分配.xlsx"

## Development Environment | 开发环境

### Ubuntu 20.04
```bash
sudo apt-get install gcc-arm-none-eabi make -y
export CROSS_COMPILE=arm-none-eabi-
make STM32
```

### MAC
```bash
brew install arm-none-eabi-gcc
export CROSS_COMPILE=arm-none-eabi-
make STM32
```

### Windows
```
Just double click "MDK-ARM/Project.uvprojx"
```

## Project Structure | 项目结构
```
├── USER/              # User code | 用户代码
├── Libraries/         # Library files | 库文件
├── MDK-ARM/          # Keil MDK project files | Keil MDK工程文件
├── LL_Drievers/      # Low-level drivers | 底层驱动
├── Public/           # Public files | 公共文件
├── USB/              # USB related code | USB相关代码
├── multiwii_2.4/     # MultiWii firmware | MultiWii固件代码
└── Makefile          # Build configuration | 编译配置
```

## Recent Updates | 最近更新
- 2021/03/02: Initial testing | 初始测试
- 2018/09/22: Modified abs to ABS, adjusted navigation stick angle | 修改abs为ABS，调整导航拉杆角度
- 2018/06/26: Fixed IIC driver bug | 修复IIC驱动BUG
- 2018/06/10: Inertial navigation implementation | 惯性导航移植
- 2018/04/02: Fixed waypoint issues | 修复航点问题
- 2018/03/28: Fixed GPS positioning | 修复GPS定点

## License | 许可证
See LICENSE file for details | 详见LICENSE文件

## Contact | 联系方式
For any questions or suggestions, please open an issue in this repository.
如有任何问题或建议，请在仓库中提交issue。
