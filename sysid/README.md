# H7_BSP 云台系统辨识数据索引

本目录集中保存 yaw 云台双环系统辨识数据、脚本和结果。原则是保留原始数据、不覆盖旧实验、按控制环路归档。

## 目录结构

- `speed_loop/`
  - `data/`：速度环激励、RTT 解析数据、Ozone 采样数据
  - `scripts/`：速度环采集与辨识脚本
  - `results/`：速度环辨识和 PID 对比结果图
  - `logs/`：后续当前参数速度环 RTT 原始日志输出目录
- `position_loop/`
  - `logs/`：位置环 RTT 原始日志，含多组参数实测
  - `scripts/`：位置环 Python/MATLAB 辨识与参数搜索脚本
  - `results/`：位置环模型、参数对比图、JSON/MAT 结果
- `reports/`
  - 系统辨识经验、实验结论和后续计划文档

## 关键数据

- 速度环旧数据：`speed_loop/data/speed_loop_legacy_sysid_data.csv`
- 速度环旧 MATLAB 数据：`speed_loop/data/speed_loop_legacy_sysid_data.mat`
- 速度环旧激励：`speed_loop/data/speed_loop_legacy_excitation.csv`
- Ozone 速度目标/IMU 数据：`speed_loop/data/ozone_yaw_speed_sampling_260701.csv`
- 位置环旧双算日志：`position_loop/logs/rtt_angle_sysid.log`
- 位置环修复双算后日志：`position_loop/logs/rtt_angle_sysid_after_single_calc.log`
- 位置环 `Kp=8.5, Ki=0` 日志：`position_loop/logs/rtt_angle_sysid_kp8p5_ki0.log`
- 位置环 `Kp=10, Ki=0` 日志：`position_loop/logs/rtt_angle_sysid_kp10_ki0.log`

## 历史实验结论

- 速度环旧模型：`G_spd(s) = 1.051 * exp(-30ms*s) / (0.108s + 1)`，闭环带宽约 `1.47Hz`。
- 速度环旧数据不代表当前固件参数。旧脚本/数据的参数语境与后续历史实验 `Kp=0.15, Ki=0.63` 不完全一致，不能直接用于当前达妙 MIT 云台的整定。
- 历史位置环曾存在外环重复计算问题，后续实验记录采用单次计算；这不是当前达妙云台的接口或验证结论。
- 历史实验参数：速度环 `Kp=0.15, Ki=0.63`；位置环 `Kp=10.00, Ki=0.00`。当前云台改用达妙 MIT，两者不可直接等同；现有 Yaw 转矩环增益默认为零，以 [Gimbal_Config.h](../User_File/Application/Gimbal/Gimbal_Config.h) 为准。
- 位置环 `Kp=10, Ki=0` 三组实测中最好：RMSE 约 `0.374rad`、MAE 约 `0.149rad`、90% 到达中位约 `0.2s`、P90 超调约 `2.35%`。

## 使用建议

采集脚本属于历史实验工具，不是当前闭环已接入激励的证明。`sysid_capture.py` 在本机生成激励文件，但不会自动把序列写入 Gimbal；它构建 Debug 固件后等待用户用 EmberProbe 烧录，再用 probe-rs attach 采集 RTT。修改数据解释或控制参数前，先核对固件实际输出的日志格式、单位与激励路径。

- 复跑旧速度环辨识：运行 `python sysid/speed_loop/scripts/identify_speed.py`。
- 后续继续优化速度环时，不要用 legacy 数据直接改 PID。应重新采当前固件参数下的 `±5 / ±10 / ±20rad/s` 速度环数据。
- 复跑位置环 MATLAB 参数分析：运行 `h7_yaw_matlab_rate_model_opt.m` 或 `h7_yaw_matlab_sysid_opt.m`。
- 所有新采集数据请使用 `current` 或日期后缀命名，避免覆盖 `legacy` 文件。
