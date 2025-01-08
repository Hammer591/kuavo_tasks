# 文件说明

## 运动控制

### `move_robot.py`
对应任务：“跑通项目的双足机器人的例子，如下图，并且编写一个脚本来向话题 `/cmd_vel` 发送数据控制机器人以指定的速度移动”。

启动该脚本后，可通过键盘按键向 `/cmd_vel` 话题发送消息控制 humanoid 仿真的运动方向：

- **s**: 前进  
- **w**: 后退  
- **a**: 向左  
- **d**: 向右  
- **q**: 左转  
- **e**: 右转  

代码使用了非阻塞输入，持续按下指定按键即可维持运动状态，无需回车。

---

### `move_vel.py`
对应任务：“自动切换步态的功能，即机器人在站立状态下向 `/cmd_vel` 发送不为 0 的速度可以自动向前走，同时发送为 0 的速度可以自动停止”。

启动脚本后，通过键盘输入速度数值可控制 humanoid 仿真的运动速度和步态：

- 输入任意非零速度：切换步态为 `walk`，设置 x 方向速度为输入值（`v > 0` 前进，`v < 0` 后退）。  
- 输入速度为 `0`：切换步态为 `stance`，速度归零。  

初始化状态下，步态为 `stance`。

---

## 视觉感知

### `yolo_test_train1.py`
模型训练脚本，采用的 YOLO 版本为 YOLOv8，数据集为 `handler`（训练集为 `image_1~91.jpg`，验证集为 `image_92~182.jpg`）。

---

### `test_pic.py`
通过图片检验 `yolo_test_train1.py` 训练所得模型的测试程序，测试对象为 `image_92.jpg`。

---

### `handler_test_vedio.py`
检验 `yolo_test_train1.py` 训练所得模型对于视频内容的识别效果的测试程序，视频来源是电脑摄像头。程序会基于 YOLO 和卡尔曼滤波预测框选出 `handler`，并用绿点标注预测中心；当检测目标被遮挡时，会基于卡尔曼滤波的预测，用红点继续标注预测的中心位置。

---

## 注意事项
- 所有代码内对于文件的调用全部采用的是绝对路径。

---

# 项目中遇到的问题及收获

## 问题 1：`humanoid_target_trajectories_publisher` 的启动问题
### 解决：
- `humanoid_target_trajectories_publisher` 控制 `/cmd_vel` 话题的开启，是控制机器人行动速度必须开启的节点。
- 通过全局检索发现，开启 `humanoid_target_trajectories_publisher` 的可执行文件位于 `humanoid_controllers` 路径下，可通过开启新终端，`source` 之后运行 `$ rosrun humanoid_controllers humanoid_target_trajectories_publisher` 手动开启该节点。

### 收获：
- 理解了话题和节点之间的关系，掌握了查询和开启话题对应节点的方法。

---

## 问题 2：使用 `catkin` 工具编译工作区域时出现的编译失败问题
### 解决：
- 由于安装了 `anaconda` 环境导致出现大量依赖或包无法检测的问题，可以通过退出 `conda` 环境解决大多数问题。
- 使用 `catkin` 工具进行 `catkin build` 编译时一定不要忘记 `$ catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo`。
- 添加 `cpp` 和创建文件夹都需要重新编译，`python` 文件不需要编译。

### 收获：
- 初步了解 `catkin` 工具的使用，对编译工作空间的原理有了一定了解。

---

## 问题 3：自动切换步态任务中遇到的问题
### 解决：
- 通过 `legged_robot_sqp.launch` 文件内容定位到开启步态控制终端的可执行文件，再根据其内容定位到发布消息的可执行程序 `GaitKeyboardPublisher.cpp`，通过阅读代码内容和关联查找确定步态消息发布的话题 `/humanoid_mpc_mode_schedule` 和向该话题发布的消息格式，最后编写脚本根据输入直接向该话题发布消息即可。
- 期间遇到成功向 `/humanoid_mpc_mode_schedule` 话题发送步态信息并被接收，但仿真无反应，不进行步态切换的问题。通过 `rostopic list` 查看发现 `/humanoid_mpc_mode_schedule` 话题正常开启，通过 `echo` 监控该话题发现消息被话题正常接收说明消息发布正常；通过查看 `legged_robot_sqp.launch` 主程序运行终端发现消息未被仿真接收，说明接收异常，通过 `gpt` 排查问题定位到接收消息的可执行文件 `GaitReceiver.cpp`，删除 `nodeHandle.subscribe()` 内 `'this'` 后的 `udp` 部分后消息被仿真正常接收。

### 收获：
- 学会了如何定位达成某一控制目标需要的对应话题以及如何正确的向该话题发布消息，学习了 `rostopic` 相关指令的使用，积累了排查问题的经验和方法。

---

## 问题 4：`commit` 历史分支的问题
### 解决：
- 在 `GitHub` 上通过 `/code/commit` 可以找到项目的历史分支，但是 `git clone` 后存储到本地的版本仍为最新版本，需要在本地通过 `git commit` 指令回退到指定历史版本。
- 同时推测问题 3 可能由未回退到指定历史版本导致。

### 收获：
- 初步了解了 `git` 的使用。

---

## 问题 5：YOLO 训练中出现找不到 `labels` 或找不到 `images` 的问题
### 解决：
- 包装 `dataset` 的时候需要注意 `label` 和 `image` 的存放，`images` 和对应 `labels` 需要放在相同的路径下，这样在 `.yaml` 中提供 `images` 路径后 YOLO 才可自动定位并匹配 `labels`，同时注意一定不要打错单词。

### 收获：
- 学习了如何使用 YOLO 训练自己的模型，了解了数据集、标注以及如何包装数据集。

---

## 问题 6：卡尔曼滤波在检测对象被遮挡后就不再检测的问题
### 解决：
- 在最开始版本的程序当检测对象脱离摄像头，其 `_id` 就会被 YOLO 检测自动忽略，导致即便添加了卡尔曼滤波也不会继续跟踪预测，只需单独处理，重新找到被忽略的对象并通过卡尔曼滤波对其位置持续预测即可。

### 收获：
- 学习了卡尔曼滤波的相关原理和程序实现。

---

## 问题 7：将文件提交到 `GitHub` 上时出现的无法提交的问题
### 解决：
- 尚未解决。

### 收获：
- 无。
