# LIO-SAM 代码逻辑分析

本文档整理当前工作区内 `src/LIO-SAM` 的核心代码逻辑，重点说明 4 个主要可执行节点的职责分工，以及 `mapOptmization.cpp` 的主流程与关键函数。

## 1. 工作空间与主要功能包

当前目录是一个 ROS1 Catkin 工作空间，功能包位于：

- `/home/john/lio-sam/src/LIO-SAM`

该包编译后生成 4 个核心可执行文件：

- `lio_sam_imageProjection`
- `lio_sam_featureExtraction`
- `lio_sam_mapOptmization`
- `lio_sam_imuPreintegration`

它们构成了 LIO-SAM 的主处理流水线：

1. 原始点云与 IMU 输入
2. 点云去畸变与结构化整理
3. 角点/面点特征提取
4. scan-to-map 建图优化
5. IMU 高频外推与最终里程计融合输出

## 2. 四个节点的功能分工

### 2.1 `lio_sam_imageProjection`

源码：

- `/home/john/lio-sam/src/LIO-SAM/src/imageProjection.cpp`

主要职责：

- 接收原始激光点云、IMU、增量里程计
- 基于 IMU 和里程计信息对一帧点云做 deskew
- 将点云投影成二维 range image
- 抽取有效点并生成 `cloud_info`

主要输入：

- `pointCloudTopic`
- `imuTopic`
- `odomTopic + "_incremental"`

主要输出：

- `lio_sam/deskew/cloud_deskewed`
- `lio_sam/deskew/cloud_info`

逻辑概括：

1. 缓存点云并检查点字段中是否有 `ring` 和 `time/t`
2. 从 IMU 队列中提取当前扫描时间范围内的姿态变化
3. 从里程计队列中提取扫描起止时刻的位姿，用于给后端提供初值
4. 将每个点按采样时刻回算到扫描起始时刻
5. 将点投影到 range image，并记录每个点所在列号、距离、扫描线索引
6. 发布供后续特征提取使用的结构化结果

### 2.2 `lio_sam_featureExtraction`

源码：

- `/home/john/lio-sam/src/LIO-SAM/src/featureExtraction.cpp`

主要职责：

- 从去畸变后的点云中提取边缘特征和面特征
- 为后端 scan-to-map 配准提供稀疏而稳定的匹配点

主要输入：

- `lio_sam/deskew/cloud_info`

主要输出：

- `lio_sam/feature/cloud_corner`
- `lio_sam/feature/cloud_surface`
- `lio_sam/feature/cloud_info`

逻辑概括：

1. 对每个点计算局部曲率
2. 标记遮挡点和不稳定点
3. 每条扫描线分段选取角点，避免特征过于集中
4. 提取平面点，并对其做体素降采样
5. 将角点、面点及更新后的 `cloud_info` 发布给后端

### 2.3 `lio_sam_mapOptmization`

源码：

- `/home/john/lio-sam/src/LIO-SAM/src/mapOptmization.cpp`

主要职责：

- 接收当前帧角点/面点特征
- 从历史关键帧构造局部地图
- 执行 scan-to-map 优化求当前帧位姿
- 用因子图维护全局一致轨迹
- 融合 GPS 和回环约束
- 发布全局里程计、增量里程计、路径和地图

主要输入：

- `lio_sam/feature/cloud_info`
- `gpsTopic`
- `lio_loop/loop_closure_detection`

主要输出：

- `lio_sam/mapping/odometry`
- `lio_sam/mapping/odometry_incremental`
- `lio_sam/mapping/path`
- `lio_sam/mapping/map_global`
- `lio_sam/mapping/map_local`
- `lio_sam/mapping/trajectory`
- `lio_sam/mapping/slam_info`
- 服务 `lio_sam/save_map`

它是整个系统的后端核心。

### 2.4 `lio_sam_imuPreintegration`

源码：

- `/home/john/lio-sam/src/LIO-SAM/src/imuPreintegration.cpp`

该可执行文件内部实际上包含两个模块：

- `IMUPreintegration`
- `TransformFusion`

主要职责：

- 利用 IMU 做高频状态预测
- 接收 mapping 输出的低频激光校正结果
- 用 GTSAM 做 IMU bias 和状态优化
- 将激光里程计与 IMU 高频预测融合成连续平滑的最终 odom

主要输入：

- `imuTopic`
- `lio_sam/mapping/odometry_incremental`
- `lio_sam/mapping/odometry`

主要输出：

- `odomTopic + "_incremental"`
- `odomTopic`
- `lio_sam/imu/path`

## 3. 系统总体数据流

整体数据流可以概括为：

1. `imageProjection`
   原始点云去畸变，生成结构化扫描
2. `featureExtraction`
   从扫描中提取角点和面点
3. `mapOptmization`
   用特征和局部地图做配准，并更新因子图
4. `imuPreintegration`
   用 IMU 高频外推，再与建图结果融合输出最终里程计

从话题关系理解：

- `imageProjection` 输出给 `featureExtraction`
- `featureExtraction` 输出给 `mapOptmization`
- `mapOptmization` 输出给 `imuPreintegration`
- `imuPreintegration` 再反向给 `imageProjection` 提供增量里程计辅助去畸变

这形成了一个前端-后端-高频预测相互协作的闭环。

## 4. `mapOptmization.cpp` 详细逻辑分析

### 4.1 文件定位

`mapOptmization.cpp` 是 LIO-SAM 的后端主控文件，负责：

- 当前帧对局部地图配准
- 关键帧管理
- 因子图优化
- GPS 融合
- 回环检测与回环约束加入
- 地图发布与保存

如果把整个系统类比成一个 SLAM 后端，这个文件就是“地图状态与轨迹状态”的主维护者。

### 4.2 主要内部数据

几个最关键的数据结构：

- `cloudKeyPoses3D`
  保存每个关键帧的 3D 平移位置

- `cloudKeyPoses6D`
  保存每个关键帧的完整 6DoF 位姿和时间戳

- `cornerCloudKeyFrames`
  保存每个关键帧的角点云

- `surfCloudKeyFrames`
  保存每个关键帧的面点云

- `laserCloudCornerLast / laserCloudSurfLast`
  当前帧的输入特征

- `laserCloudCornerFromMap / laserCloudSurfFromMap`
  从附近关键帧拼接出来的局部地图

- `gtSAMgraph / initialEstimate / isam`
  GTSAM 因子图与增量优化器

- `transformTobeMapped[6]`
  当前帧待优化位姿，是 scan-to-map 的核心状态变量

### 4.3 主线程处理流程

入口函数是：

- `laserCloudInfoHandler()`

每来一帧 `lio_sam/feature/cloud_info`，后端执行一次主流程：

1. 更新时间戳并读入当前帧角点/面点
2. `updateInitialGuess()`
   生成当前帧位姿初值
3. `extractSurroundingKeyFrames()`
   构建局部地图
4. `downsampleCurrentScan()`
   对当前帧特征做降采样
5. `scan2MapOptimization()`
   当前帧对局部地图做迭代优化
6. `saveKeyFramesAndFactor()`
   满足条件时写成关键帧并加入因子图
7. `correctPoses()`
   若发生回环，回写全部历史关键帧位姿
8. `publishOdometry()`
   发布全局和增量里程计
9. `publishFrames()`
   发布路径、关键帧、局部图与调试信息

### 4.4 初值更新 `updateInitialGuess()`

作用：

- 为当前帧 scan-to-map 提供较好的初始位姿

策略分三层：

1. 如果系统刚启动且没有关键帧
   直接用 IMU 的 roll/pitch/yaw 作为姿态初值

2. 如果前端提供了 `odomAvailable`
   优先使用由 `imageProjection` 提供的增量里程计初值

3. 如果没有可用平移初值
   则退化为只使用 IMU 旋转增量来更新姿态初值

这样做的目的，是尽量让当前帧优化从“接近正确”的位置起步，减少 LM 迭代次数并降低发散风险。

### 4.5 局部地图构建 `extractSurroundingKeyFrames()`

核心调用链：

- `extractSurroundingKeyFrames()`
- `extractNearby()`
- `extractCloud()`

逻辑：

1. 用当前轨迹末端位置在关键帧轨迹上做半径搜索
2. 取附近关键帧作为局部地图候选
3. 对关键帧位置本身做一次降采样，控制局部地图规模
4. 额外补入最近若干秒的关键帧，避免机器人原地转圈时局部地图不完整
5. 将这些关键帧对应的角点和面点变换到世界坐标系
6. 融合成 `laserCloudCornerFromMap` 和 `laserCloudSurfFromMap`
7. 再对局部地图特征做一次降采样

这里还引入了缓存：

- `laserCloudMapContainer`

已经变换过的关键帧点云会缓存下来，避免每帧反复重复坐标变换。

### 4.6 当前帧降采样 `downsampleCurrentScan()`

作用：

- 降低当前帧特征数量，控制 scan-to-map 匹配和优化开销

结果保存到：

- `laserCloudCornerLastDS`
- `laserCloudSurfLastDS`

后续所有配准计算都基于它们进行，而不是直接使用原始特征。

### 4.7 scan-to-map 优化 `scan2MapOptimization()`

这是 `mapOptmization.cpp` 最核心的计算阶段。

主循环如下：

1. 建立局部地图角点/面点的 KD-Tree
2. 最多迭代 30 次
3. 每轮迭代中：
   - `cornerOptimization()`
   - `surfOptimization()`
   - `combineOptimizationCoeffs()`
   - `LMOptimization()`
4. 若 LM 收敛，则提前结束
5. 最后执行 `transformUpdate()`

#### 4.7.1 `cornerOptimization()`

对当前帧每个角点：

1. 先用当前估计位姿把点变换到地图坐标系
2. 在局部地图角点中找最近 5 个点
3. 用这 5 个点估计一条主方向明显的直线
4. 构造“点到线距离”残差

得到的残差和雅可比信息会存入：

- `laserCloudOriCornerVec`
- `coeffSelCornerVec`

#### 4.7.2 `surfOptimization()`

对当前帧每个面点：

1. 变换到地图坐标系
2. 在局部地图面点中找最近 5 个点
3. 拟合一个平面
4. 检查这 5 个点是否真的近似共面
5. 构造“点到平面距离”残差

结果存入：

- `laserCloudOriSurfVec`
- `coeffSelSurfVec`

#### 4.7.3 `combineOptimizationCoeffs()`

作用：

- 合并角点残差和面点残差
- 组成一个统一的最小二乘问题输入

#### 4.7.4 `LMOptimization()`

作用：

- 用 LOAM 风格的 LM 最小二乘更新 `transformTobeMapped`

逻辑：

1. 构造正规方程
2. 第一轮时做退化检测
3. 计算本轮位姿增量
4. 若旋转和平移增量都足够小，则判定收敛

#### 4.7.5 `transformUpdate()`

作用：

- 在 LM 优化结果基础上做后处理

内容包括：

1. 用 IMU 对 roll/pitch 做轻度约束
2. 对旋转和 z 方向位移做上限限制
3. 记录优化后的位姿，供增量里程计输出使用

### 4.8 关键帧判断 `saveFrame()`

不是每一帧都会成为关键帧。

判断原则：

- 如果当前帧相对上一关键帧的旋转变化和位移变化都很小
- 则不保存为关键帧

这样做可以减少：

- 地图冗余
- 因子图规模
- 配准计算量

### 4.9 因子图写入

关键函数：

- `addOdomFactor()`
- `addGPSFactor()`
- `addLoopFactor()`
- `saveKeyFramesAndFactor()`

#### 4.9.1 `addOdomFactor()`

作用：

- 给因子图添加相邻关键帧之间的相对位姿约束

规则：

- 第一帧使用先验因子
- 后续关键帧使用 BetweenFactor

这是轨迹连续性的基础约束。

#### 4.9.2 `addGPSFactor()`

作用：

- 在需要时引入 GPS 绝对位置约束

加入条件比较严格：

1. 系统已经有足够运动
2. 当前位姿协方差较大，说明后端不够确定
3. GPS 时间戳与当前帧足够接近
4. GPS 协方差不能太大
5. 与上一条有效 GPS 位置要相隔一定距离

这样做是为了避免频繁使用噪声 GPS 破坏局部几何一致性。

#### 4.9.3 `addLoopFactor()`

作用：

- 把回环线程提前准备好的 pose-pose 约束写入因子图

这些约束来自：

- `performLoopClosure()` 中 ICP 验证通过的回环结果

#### 4.9.4 `saveKeyFramesAndFactor()`

这是关键帧写入和因子图更新的总入口：

1. 判断是否保存当前帧为关键帧
2. 追加 odom/GPS/loop 因子
3. 调用 iSAM2 增量更新
4. 若有强回环/GPS 约束，额外多次 update 让结果充分传播
5. 从最新优化结果中取出当前关键帧位姿
6. 保存关键帧点云与路径
7. 更新 `transformTobeMapped`

### 4.10 回环处理

关键线程与函数：

- `loopClosureThread()`
- `performLoopClosure()`
- `detectLoopClosureDistance()`
- `detectLoopClosureExternal()`
- `loopFindNearKeyframes()`
- `visualizeLoopClosure()`

处理逻辑：

1. 后台线程周期性尝试寻找回环候选
2. 候选可来自外部检测结果，或内部距离搜索
3. 对候选关键帧附近子图做 ICP 验证
4. 若 ICP 收敛且 fitness 足够好，则生成回环约束
5. 把约束压入队列，等待主线程写入因子图

回环本身不直接在后台线程里改图，而是通过队列与主线程同步，这样结构更安全。

### 4.11 位姿回写 `correctPoses()`

当回环或某些强约束导致全局图被重新优化后，需要把最新位姿结果回写到历史关键帧中。

函数工作：

1. 清空局部地图缓存
2. 清空可视化路径
3. 用最新 `isamCurrentEstimate` 覆盖历史关键帧位姿
4. 重新生成可视化路径

这样后续局部地图提取和地图发布都会基于新的全局一致轨迹。

### 4.12 发布逻辑

关键函数：

- `publishOdometry()`
- `publishFrames()`
- `publishGlobalMap()`

#### 4.12.1 `publishOdometry()`

发布两类里程计：

1. `lio_sam/mapping/odometry`
   表示后端优化后的全局位姿

2. `lio_sam/mapping/odometry_incremental`
   表示连续平滑的增量里程计

其中增量里程计会：

- 利用本帧前后优化位姿差值累积
- 对 roll/pitch 做少量 IMU 融合

它的主要用途是供 `imuPreintegration` 做高频外推和融合。

#### 4.12.2 `publishFrames()`

该函数将内部状态拆成多种调试与可视化输出：

- 关键帧轨迹
- 局部地图
- 当前帧配准后的点云
- 原始去畸变点云的全局投影
- 路径
- `slam_info`

`slam_info` 可被第三方模块消费，用于外部显示或二次处理。

#### 4.12.3 `publishGlobalMap()`

作用：

- 导出或可视化全局地图

逻辑：

1. 取当前位姿附近的关键帧
2. 将其点云变换到全局坐标
3. 融合并降采样
4. 发布地图点云

### 4.13 地图保存 `saveMapService()`

服务名：

- `lio_sam/save_map`

它会把所有关键帧重新投影到世界坐标后导出成 PCD：

- `trajectory.pcd`
- `transformations.pcd`
- `CornerMap.pcd`
- `SurfMap.pcd`
- `GlobalMap.pcd`

如果请求中给了分辨率，还会先做下采样。

## 5. 如何理解 `mapOptmization.cpp` 的代码结构

可以把这个文件按 5 个层次来读：

1. 数据输入层
   `laserCloudInfoHandler()`、`gpsHandler()`、`loopInfoHandler()`

2. 局部地图层
   `extractNearby()`、`extractCloud()`、`extractSurroundingKeyFrames()`

3. 当前帧优化层
   `cornerOptimization()`、`surfOptimization()`、`LMOptimization()`、`scan2MapOptimization()`

4. 全局图优化层
   `addOdomFactor()`、`addGPSFactor()`、`addLoopFactor()`、`saveKeyFramesAndFactor()`

5. 结果发布层
   `correctPoses()`、`publishOdometry()`、`publishFrames()`、`publishGlobalMap()`

按这个顺序理解，会比直接从头读到尾更清晰。

## 6. 当前代码阅读建议

如果继续深入理解，建议按以下顺序看源码：

1. `imageProjection.cpp`
   先理解点云是如何被 deskew 和结构化的
2. `featureExtraction.cpp`
   再理解后端到底在用什么特征
3. `mapOptmization.cpp`
   重点理解 scan-to-map 和因子图
4. `imuPreintegration.cpp`
   最后理解高频预测和最终 odom 融合

如果只想抓住后端主线，则建议直接按下列函数顺序阅读 `mapOptmization.cpp`：

1. `laserCloudInfoHandler()`
2. `updateInitialGuess()`
3. `extractSurroundingKeyFrames()`
4. `downsampleCurrentScan()`
5. `scan2MapOptimization()`
6. `saveKeyFramesAndFactor()`
7. `correctPoses()`
8. `publishOdometry()`
9. `publishFrames()`

## 7. 当前代码状态说明

当前工作区内已做过以下兼容性修正，代码已可成功编译：

1. OpenCV 旧头文件替换为 OpenCV 4 可用头
2. 构建标准从 C++11 提升为 C++14
3. 在项目内补充了 `flann` 对 `std::unordered_map` 的序列化特化

当前仍存在一些非阻塞告警，例如：

- `tf::TransformException` 按值捕获
- `unused` 变量未使用
- 部分有符号/无符号比较
- PCL 旧类型别名废弃告警

这些不会阻止编译，但后续可以进一步清理。
