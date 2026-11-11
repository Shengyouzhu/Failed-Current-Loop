# STM32 FOC 电流环实验备份

这是 `STM32project` 原版工程的存档，用于保留当前电流环实验状态。工程使用 STM32F407VETx、AS5600 磁编码器和 DRV8316 驱动板。

## 当前状态

- 电流环目标 `Iq = 0.2 A`，软件电压限幅为 `1.5 V`。
- 目前仅用电流环控制，没有速度外环。实际转速和电流仍有较大波动，尚未验证为稳定可用的电流控制方案。
- 本仓库保存的是原版工程；用于尝试商家算法和速度环的 `STM32project验证` 不在此仓库内。

## 编译

使用 Keil MDK 5 打开 `MDK-ARM/STM32project.uvprojx`，选择 `STM32project` 目标并编译。CubeMX 配置保存在 `STM32project.ioc`。

编译产物、个人 Keil 设置和调试会话文件已加入 `.gitignore`。首次存档前，本机使用 Arm Compiler 5.06 update 6 编译，结果为 0 error、0 warning。
