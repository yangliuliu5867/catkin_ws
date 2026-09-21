#pragma once

#include "unified_trt.hpp"

#include <memory>
#include <mutex>
#include <string>

namespace trt
{

class TrtManager
{
public:
    // Meyers 单例：全局唯一管理器
    static TrtManager &instance();

    // 显式初始化：加载 engine 并构造需要的 Predictor（传空字符串可跳过某个模型）
    // 支持部分初始化：如果某个 engine 路径为空，则对应 Predictor 不会被创建。
    // 如果至少创建了一个 Predictor 并成功 warmup 则返回 true。重复调用为幂等。
    bool init(const std::string &engine_collision_bs36,
              const std::string &engine_collision_bs1,
              const std::string &engine_student,
              bool use_cuda_graph = true,
              bool verbose_timing = false);

    // 是否已经成功完成初始化（包括构造和 warmup）
    bool isReady() const;

    // 访问已预热好的模型接口（需先确保 isReady()==true）
    CollisionPredictor &collisionBs36();
    CollisionPredictor &collisionBs1();
    StudentPredictor &student();

private:
    TrtManager() = default;
    ~TrtManager() = default;

    TrtManager(const TrtManager &) = delete;
    TrtManager &operator=(const TrtManager &) = delete;

    mutable std::mutex mutex_;
    bool ready_ = false;

    std::unique_ptr<CollisionPredictor> collision_bs36_;
    std::unique_ptr<CollisionPredictor> collision_bs1_;
    std::unique_ptr<StudentPredictor> student_;
};

} // namespace trt
