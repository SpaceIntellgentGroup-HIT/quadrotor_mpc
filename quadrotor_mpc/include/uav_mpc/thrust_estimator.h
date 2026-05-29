#ifndef THRUST_ESTIMATOR_H
#define THRUST_ESTIMATOR_H

#include <queue>
#include <utility>

class ThrustEstimator {
private:
    // 历史油门队列，存储 <时间戳(秒), 油门百分比(0~1)>
    std::queue<std::pair<double, double>> timed_thrust_; 
    
    // RLS 算法状态参数
    const double rho2_ = 0.998; // 遗忘因子
    double thr2acc_;            // 待估计的推力映射系数
    double P_;                  // 协方差矩阵 (一阶系统中为标量)
    
    // 延迟补偿时间窗口
    const double delay_max_ = 0.045; // 45ms
    const double delay_min_ = 0.035; // 35ms

public:
    /**
     * @brief 构造函数，初始化估计器
     * @param hover_percentage 经验悬停油门百分比 (例如 0.35 表示 35% 油门悬停)
     * @param gravity 当地重力加速度 (例如 9.8066)
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
     * @param real_acc_z     IMU当前测量的 Z 轴实际加速度
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

#endif
