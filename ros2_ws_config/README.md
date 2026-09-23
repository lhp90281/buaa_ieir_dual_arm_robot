# Machine Configuration

此目录只发布电机/理论限位输入模板、可复用摩擦参数和合并脚本。
**实测方向、零偏和已确认限位属于每台机器人，不提交 Git、不随克隆分发。**
它们仍保存在此目录供本机读取；更换电机、机械装配或烧录零位后必须重新标定。
首次克隆先完成方向、限位确认、手动标定和合并，再启动真机控制器，不能创建全零文件绕过标定。

| 文件 | 用途 | 更新时机 |
|---|---|---|
| `joint_calibration_dual_{left,right}.yaml` | 电机型号、slot、原始限位参考 | 核对硬件时 |
| `joint_calibration_dual_{left,right}_reviewed.yaml` | 用户在模型中确认的硬限位参考 | limit-preview 完成时 |
| `joint_directions_{left,right}.yaml` | 实物运动与模型一致的 axis_sign | 方向阶段全部完成时 |
| `joint_offsets_{left,right}.yaml` | 单臂 7 个关节的零偏和方向 | 手动每关节 Enter 后立即更新 |
| `joint_offsets_dual.yaml` | 14 关节控制器默认读取文件 | 手动运行 merge_offsets.py |
| `friction_model.yaml` | 按电机型号划分的摩擦参数 | 摩擦辨识/人工复核后 |

计算为 `q_urdf = axis_sign * (q_raw - zero_offset)`。
从参考点得到 `zero_offset = raw_at_reference - axis_sign * urdf_pos_at_reference`。
不能只改 axis_sign 而保留旧零偏。W3 的电机 YAML 保持 offset=0、signs=+1，
避免 bridge 和 ros2_control 重复变换。

在工作区执行 `python3 src/ros2_ws_config/merge_offsets.py`，同时读取左右两臂，
要求每臂正好 7 个唯一名字/slot，输出合并文件，不会驱动机器人。
单臂控制可直接指定对应单臂文件。重标后需重新合并，再重启控制端；无需重新编译。
合并会覆盖旧合并文件，标定前建议备份整个本目录。

完整命令、操作键和安全前提见 [主 README](../README.md#calibration)。

Git 忽略 `joint_offsets*.yaml*`、`joint_directions*.yaml*` 和
`joint_calibration_*_reviewed.yaml*`（包括同名备份）。不要使用 `git add -f` 强行提交。
未带 `_reviewed` 的输入文件只是候选限位，不代表当前实物已经确认。
摩擦参数按电机类型复用，不属于本次排除的零位标定数据。
