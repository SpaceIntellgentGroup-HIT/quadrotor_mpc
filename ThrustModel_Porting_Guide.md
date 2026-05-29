# 无人机推力模型在线估计 (Thrust Model Estimation) 移植说明

本文档旨在说明如何将 `estimateThrustModel` 功能移植到其他无人机飞控或伴控系统中。

## 1. 功能的作用 (Purpose)
在多旋翼无人机飞行过程中，随着电池电量的消耗，系统电压会逐渐下降。这意味着**相同的电机油门指令（百分比），在满电和低电时产生的实际物理推力（加速度）是不同的**。

为了实现高精度的轨迹追踪，系统需要知道“期望加速度”与“实际下发的油门量”之间的精确数学对应关系。
`estimateThrustModel` 的作用是：**利用带有遗忘因子的递归最小二乘法 (RLS, Recursive Least Squares)，在线实时估计这个比例系数，从而自适应电池电压的变化。**

---

## 2. 输入与输出 (Inputs & Outputs)

**输入数据 (Inputs):**
1. **当前系统时间 (`current_time`)**: 用于计算延迟，对齐数据。
2. **实际 Z 轴加速度 (`real_acc_z`)**: 来源于飞控或 IMU 传感器测得的当前 Z 轴机体加速度。
   - *数据来源说明*：在 ROS 系统中，通常通过订阅飞控节点（如 MAVROS 的 `/mavros/imu/data` 话题）获取 `sensor_msgs/Imu` 消息，并直接提取其中的 `linear_acceleration.z`。
3. **历史油门指令队列 (`timed_thrust_`)**: 记录了过去一段时间下发的油门百分比及其对应的时间戳。

**输出数据 (Outputs):**
1. **推力-加速度映射系数 (`thr2acc_`)**: 算法的核心输出。其物理意义是“100% 油门能够产生的 Z 轴加速度”。
2. **期望油门指令 (`throttle_percentage`)**: 在控制环中，利用估计出的 `thr2acc_`，将算法规划出的期望 Z 轴加速度转换为发送给底层飞控的油门百分比（`throttle = desired_acc_z / thr2acc_`）。

---

## 3. 实现方式 (Implementation Method)

移植本功能的核心在于两部分：**系统延迟补偿** 和 **递归最小二乘法(RLS)参数更新**。

### 3.1 延迟补偿 (Delay Compensation)
指令下发到电机响应，再到 IMU 测量出加速度，存在物理和通信上的时间延迟（通常在 30~50ms 之间）。因此，不能用“当前的油门”对应“当前的加速度”。
- **机制**：每次下发油门指令时，将 `(时间戳, 油门量)` 压入历史队列。在估计参数时，只取距今 **35ms ~ 45ms** 之间的历史油门数据与当前的加速度进行匹配。太旧的数据直接丢弃，太新的数据则等待下一次循环。

### 3.2 递归最小二乘法 (RLS with Forgetting Factor)
采用了一阶线性无截距模型：$a_z = thr2acc\_ \times thr$
- $a_z$ 是实测 Z 轴加速度， $thr$ 是历史油门百分比。
- 引入了**遗忘因子 $\rho^2 = 0.998$**：由于电池电压下降是缓慢的动态过程，引入遗忘因子可以使算法赋予近期数据更大的权重，“遗忘”很久以前的旧模型，从而实现参数的动态跟踪。

---

## 4. 移植示例代码 (C++ Example Code)

为了方便移植，这里去除了原工程中的 ROS 依赖（例如 `ros::Time`），使用了标准的 `double` 作为时间戳（单位：秒）。您可以直接将此类封装到您的 C++ 控制系统中。

```cpp
#include <queue>
#include <utility>

class ThrustEstimator {
private:
    // 历史油门队列，存储 <时间戳(秒), 油门百分比(0~1)>
    std::queue<std::pair<double, double>> timed_thrust_; 
    
    // RLS 算法状态参数
    const double rho2_ = 0.998; // 遗忘因子 (通常无需修改)
    double thr2acc_;            // 待估计的推力映射系数
    double P_;                  // 协方差矩阵 (一阶系统中为标量)
    
    // 延迟补偿时间窗口 (根据具体硬件响应速度调整)
    const double delay_max_ = 0.045; // 45ms
    const double delay_min_ = 0.035; // 35ms

public:
    /**
     * @brief 构造函数，初始化估计器
     * @param hover_percentage 经验悬停油门百分比 (例如 0.35 表示 35% 油门悬停)
     * @param gravity 当地重力加速度 (例如 9.81)
     */
    ThrustEstimator(double hover_percentage, double gravity) {
        thr2acc_ = gravity / hover_percentage; // 初始猜测值
        P_ = 1e6;                              // 初始协方差设置很大，表示初始状态不确定
    }

    /**
     * @brief 在每次向底层下发油门指令时调用，记录历史数据
     * @param current_time_s 当前时间(秒)
     * @param thrust         下发的油门百分比(0~1)
     */
    void pushThrustRecord(double current_time_s, double thrust) {
        timed_thrust_.push({current_time_s, thrust});
        
        // 限制队列长度，防止内存无限增长
        while (timed_thrust_.size() > 100) {
            timed_thrust_.pop();
        }
    }

    /**
     * @brief 在控制器的循环中高频调用，在线更新映射系数
     * @param current_time_s 当前时间(秒)
     * @param real_acc_z     IMU当前测量的 Z 轴实际加速度 (去除重力影响后的体轴加速度)
     * @return true表示本次更新了参数，false表示无合适数据更新
     */
    bool estimateThrustModel(double current_time_s, double real_acc_z) {
        while (timed_thrust_.size() >= 1) {
            auto t_t = timed_thrust_.front();
            double time_passed = current_time_s - t_t.first;

            if (time_passed > delay_max_) {
                // 数据太旧，已错过匹配窗口，丢弃
                timed_thrust_.pop();
                continue;
            }
            if (time_passed < delay_min_) {
                // 队首数据还太新，电机的响应还没完全体现在当前加速度上，退出等待
                return false;
            }

            // 找到了延迟窗口内匹配的油门数据
            double thr = t_t.second;
            timed_thrust_.pop();

            // ---------------------------------------------------------
            // 带有遗忘因子的 RLS (递归最小二乘) 算法核心
            // ---------------------------------------------------------
            // 1. 计算卡尔曼增益 K
            double gamma = 1.0 / (rho2_ + thr * P_ * thr);
            double K = gamma * P_ * thr;
            
            // 2. 根据预测误差更新映射系数
            // 预测误差 = 实际加速度 - 预测加速度(thr * thr2acc_)
            thr2acc_ = thr2acc_ + K * (real_acc_z - thr * thr2acc_);
            
            // 3. 更新协方差
            P_ = (1.0 - K * thr) * P_ / rho2_;

            return true;
        }
        return false;
    }

    /**
     * @brief 利用估计出的系数，计算出为了产生期望加速度所需的油门量
     * @param desired_acc_z 期望的Z轴加速度
     * @return 实际应该下发给底层的油门百分比
     */
    double computeDesiredThrust(double desired_acc_z) {
        return desired_acc_z / thr2acc_;
    }
    
    // 获取当前估计出的映射系数 (用于调试或日志记录)
    double getThr2Acc() const { return thr2acc_; }
};
```
