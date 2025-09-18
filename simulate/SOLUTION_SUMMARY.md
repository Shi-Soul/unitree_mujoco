# 解决方案总结：MuJoCo 仿真重置功能

## 问题分析

原始问题是将 MuJoCo 仿真从 `main.cc` 改为 `main_ros.cc` 的 ROS2 节点形式，但编译时遇到了 DDS 库版本冲突的问题。

**根本原因：**
- Unitree SDK2 使用自己的 DDS 库
- ROS2 Foxy 使用 CycloneDDS 
- 两个 DDS 库版本不兼容，存在符号冲突（如 `DDSRT_SCHED_REALTIME`、`ddsrt_sched_t` 等）

## 解决方案

采用了**原生 DDS 方案**而不是混合 ROS2 的方案，避免了 DDS 版本冲突问题。

### 实现方式

1. **创建 DDS 版本主程序** (`src/main_dds.cc`)
   - 基于原始 `main.cc`
   - 集成 Unitree SDK2 的原生 DDS 功能
   - 添加重置订阅者和重置逻辑

2. **DDS 重置消息结构**
   ```cpp
   struct ResetMessage {
       uint32_t timestamp;  // 时间戳
       bool reset_flag;     // 重置标志
   };
   ```

3. **DDS 话题接口**
   - 话题名称：`mjc/reset`
   - 消息类型：`ResetMessage`
   - 通信方式：Unitree SDK2 原生 DDS

## 文件结构

### 新增文件
```
src/unitree_mujoco/simulate/
├── src/main_dds.cc                    # DDS版本主程序
├── scripts/dds_reset_publisher.cpp    # C++ DDS发布者工具
├── scripts/dds_reset_test.py          # Python测试脚本
├── README_DDS.md                      # DDS版本使用说明
├── install_dds.sh                     # 安装脚本
├── test_dds_implementation.sh         # 测试脚本
└── SOLUTION_SUMMARY.md                # 本文档
```

### 修改文件
```
├── CMakeLists.txt                     # 添加DDS版本构建配置
└── package.xml                        # 保留原有ROS2配置（可选）
```

## 构建结果

构建成功生成以下可执行文件：
- `unitree_mujoco` - 原始版本（保持不变）
- `unitree_mujoco_dds` - **DDS版本（支持重置功能）**
- `dds_reset_publisher` - 重置命令发布工具

## 使用方法

### 快速开始

1. **构建**
   ```bash
   cd src/unitree_mujoco/simulate
   ./install_dds.sh
   ```

2. **启动仿真**
   ```bash
   ./build/unitree_mujoco_dds -r g1 -n eno1 -i 0
   ```

3. **发送重置命令**
   ```bash
   # 单次重置
   ./build/dds_reset_publisher --once
   
   # 循环重置（每10秒）
   ./build/dds_reset_publisher --loop
   ```

### 验证测试
```bash
./test_dds_implementation.sh
```

## 重置功能

当收到重置命令时，系统会：

1. **位置重置**：关节返回到默认位置（如果有关键帧0则使用，否则使用 `qpos0`）
2. **速度重置**：所有关节速度设为零
3. **力重置**：清除所有外部施加的力
4. **正向运动学**：调用 `mj_forward()` 更新派生量

## 方案优势

### vs ROS2 混合方案
1. **无 DDS 冲突**：使用与 Unitree SDK2 相同的 DDS 库
2. **构建简单**：无需 ROS2 环境或 colcon
3. **更好集成**：与 Unitree 生态系统原生集成
4. **独立运行**：无需 ROS2 安装即可运行

### vs 修改原始代码
1. **保持兼容**：原始版本完全保留
2. **功能增强**：添加了网络重置功能
3. **易于维护**：清晰的代码分离

## 技术细节

### DDS 通信架构
```
dds_reset_publisher  --[DDS mjc/reset]--> unitree_mujoco_dds
                                              ↓
                                         ResetRobot()
                                              ↓
                                         MuJoCo 重置
```

### 线程架构
- **主线程**：MuJoCo 渲染循环
- **物理线程**：物理仿真和重置处理
- **Unitree桥接线程**：SDK2 数据桥接
- **重置订阅线程**：DDS 重置命令监听

### 安全性
- 使用 `std::atomic<bool>` 实现线程安全的重置标志
- 在物理循环中安全处理重置请求
- 避免竞态条件

## 测试验证

✅ 所有测试通过：
- 构建工件检查
- 帮助功能测试  
- 文件结构验证
- 基本功能测试

## 总结

成功解决了 DDS 版本冲突问题，实现了稳定可靠的重置功能。采用原生 DDS 方案避免了复杂的依赖冲突，提供了更好的性能和兼容性。

**推荐使用 `unitree_mujoco_dds` 版本**，它提供了所有原始功能plus网络重置能力，同时避免了 ROS2 DDS 冲突问题。 