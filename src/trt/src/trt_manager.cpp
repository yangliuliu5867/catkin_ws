#include "trt_manager.hpp"

#include <iostream>

namespace trt
{

TrtManager &TrtManager::instance()
{
    static TrtManager inst;
    return inst;
}

bool TrtManager::init(const std::string &engine_collision_bs36,
                      const std::string &engine_collision_bs1,
                      const std::string &engine_student,
                      bool use_cuda_graph,
                      bool verbose_timing)
{
    std::lock_guard<std::mutex> lk(mutex_);
    if (ready_)
    {
        return true;
    }

    try
    {
        bool any_ok = false;

        if (!engine_collision_bs36.empty()) {
            collision_bs36_.reset(new CollisionPredictor(
                engine_collision_bs36,
                /*max_batch_size=*/36,
                /*use_cuda_graph=*/use_cuda_graph,
                /*verbose_timing=*/verbose_timing));
            any_ok = true;
            std::cout << "[TrtManager] collision_bs36 initialized." << std::endl;
        }

        if (!engine_collision_bs1.empty()) {
            collision_bs1_.reset(new CollisionPredictor(
                engine_collision_bs1,
                /*max_batch_size=*/1,
                /*use_cuda_graph=*/use_cuda_graph,
                /*verbose_timing=*/verbose_timing));
            any_ok = true;
            std::cout << "[TrtManager] collision_bs1 initialized." << std::endl;
        }

        if (!engine_student.empty()) {
            student_.reset(new StudentPredictor(
                engine_student,
                /*max_batch_size=*/72,
                /*use_cuda_graph=*/use_cuda_graph,
                /*verbose_timing=*/verbose_timing));
            any_ok = true;
            std::cout << "[TrtManager] student initialized." << std::endl;
        }

        if (!any_ok) {
            std::cerr << "[TrtManager] No engine paths provided - nothing to initialize." << std::endl;
            ready_ = false;
            return false;
        }

        // 至此，至少一个 Predictor 构造并已在其构造函数内完成 warmup
        ready_ = true;
        std::cout << "[TrtManager] Models initialized and warmed up (partial or full)." << std::endl;
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[TrtManager] Exception during init: " << e.what() << std::endl;
        ready_ = false;
        collision_bs36_.reset();
        collision_bs1_.reset();
        student_.reset();
        return false;
    }
}

bool TrtManager::isReady() const
{
    std::lock_guard<std::mutex> lk(mutex_);
    return ready_;
}

CollisionPredictor &TrtManager::collisionBs36()
{
    std::lock_guard<std::mutex> lk(mutex_);
    if (!ready_ || !collision_bs36_)
    {
        throw std::runtime_error("[TrtManager] collisionBs36 called before successful init()");
    }
    return *collision_bs36_;
}

CollisionPredictor &TrtManager::collisionBs1()
{
    std::lock_guard<std::mutex> lk(mutex_);
    if (!ready_ || !collision_bs1_)
    {
        throw std::runtime_error("[TrtManager] collisionBs1 called before successful init()");
    }
    return *collision_bs1_;
}

StudentPredictor &TrtManager::student()
{
    std::lock_guard<std::mutex> lk(mutex_);
    if (!ready_ || !student_)
    {
        throw std::runtime_error("[TrtManager] student called before successful init()");
    }
    return *student_;
}

} // namespace trt
