/**
 * @file unified_trt.h
 * @brief 统一的 TensorRT 推理头文件，包含三种模型的部署接口
 * 
 * 本文件合并了 trt_collision 和 trt_small 两个项目，提供：
 *   1. CollisionPredictor - 碰撞检测模型 (支持 bs=1 和 bs=36)
 *   2. StudentPredictor   - Student 轨迹生成模型 (默认 bs=72)
 * 
 * 使用方式：
 *   #include "unified_trt.h"
 *   
 *   // 初始化三个模型（构造时自动加载引擎并预热）
 *   CollisionPredictor collision_bs36("best_model_multimat.engine", 36);
 *   CollisionPredictor collision_bs1("best_model_multimat_bs1.engine", 1);
 *   StudentPredictor   student("student_distilled.engine", 72);
 * 
 * @author Auto-generated from trt_collision and trt_small
 * @date 2024
 */

#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <numeric>
#include <cstring>
#include <array>
#include <algorithm>
#include <mutex>
#include <cuda_runtime_api.h>
#include <NvInfer.h>
#include <cmath>
#include <chrono>
#include <Eigen/Dense>

/**
 * @brief CUDA 错误检查宏
 */
#define CHECK(status) \
    do { \
        const cudaError_t _err = static_cast<cudaError_t>((status)); \
        if (_err != cudaSuccess) { \
            std::cerr \
                << "Cuda failure: " << static_cast<int>(_err) \
                << " (" << cudaGetErrorString(_err) << ")" \
                << " at " << __FILE__ << ":" << __LINE__ \
                << " in call: " << #status \
                << std::endl; \
            abort(); \
        } \
    } while (0)

/**
 * @brief 全局 TensorRT Logger（三种模型共用）
 */
class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            std::cout << "[TRT] " << msg << std::endl;
    }
};

extern Logger gLogger;

// ============================================================================
//                        CollisionPredictor
// ============================================================================
/**
 * @class CollisionPredictor
 * @brief 碰撞检测模型的 TensorRT 推理类
 * 
 * 用于预测张拉整体结构的碰撞响应。支持批量推理 (bs=1 或 bs=36)。
 * 
 * ## 输入格式 (来自 collision2onnx.py)
 * 
 * 每个样本包含 12 个 float，格式如下：
 * ```
 * [state(6), quat(4), phys(2)]
 * ```
 * 
 * | 字段        | 索引     | 维度 | 描述                                      |
 * |-------------|----------|------|-------------------------------------------|
 * | state       | [0:6]    | 6    | 状态向量 (线速度/角速度)           |
 * | quat        | [6:10]   | 4    | 四元数 (w, x, y, z) 顺序，表示姿态       |
 * | phys        | [10:12]  | 2    | 物理参数 (如材料属性)                     |
 * 
 * **注意**: 四元数采用 (w, x, y, z) 顺序，w 在前！
 * 
 * ## 输出格式
 * 
 * 每个样本输出 6 个 float：
 * ```
 * [output(6)]
 * ```
 * 输出已经过反归一化处理，为物理单位下的预测结果。
 * 
 * ## 示例代码
 * ```cpp
 * // 创建 bs=36 的碰撞检测器
 * CollisionPredictor predictor("best_model_multimat.engine", 36, true, false);
 * 
 * // 准备输入数据
 * int batch_size = 36;
 * std::vector<float> inputs(batch_size * 12);
 * // 填充 inputs: [state0(6), quat0(4), phys0(2), state1(6), quat1(4), phys1(2), ...]
 * 
 * // 推理
 * std::vector<float> outputs;
 * predictor.predict(inputs, outputs, batch_size);
 * // outputs 大小为 batch_size * 6
 * ```
 */
class CollisionPredictor {
public:
    /**
     * @brief 构造函数，加载引擎并自动预热
     * 
     * @param engine_path    TRT engine 文件路径 (.engine)
     * @param max_batch_size 最大批次大小（bs=1 或 bs=36）
     * @param use_cuda_graph 是否启用 CUDA Graphs 加速（默认 true）
     * @param verbose_timing 是否启用详细计时事件（默认 false）
     * 
     * @note 构造时会自动进行 10 次预热推理
     */
    CollisionPredictor(const std::string& engine_path, int max_batch_size = 36, 
                      bool use_cuda_graph = true, bool verbose_timing = false)
        : mMaxBatchSize(max_batch_size), mEnableCudaGraph(use_cuda_graph), mVerboseTiming(verbose_timing) {
        loadEngine(engine_path);
        allocateMemory();
        warmup();
    }

    /**
     * @brief 析构函数，释放所有 CUDA 资源
     */
    ~CollisionPredictor() {
        if (mEventH2DStart) cudaEventDestroy(mEventH2DStart);
        if (mEventH2DEnd) cudaEventDestroy(mEventH2DEnd);
        if (mEventExecStart) cudaEventDestroy(mEventExecStart);
        if (mEventExecEnd) cudaEventDestroy(mEventExecEnd);
        if (mEventD2HStart) cudaEventDestroy(mEventD2HStart);
        if (mEventD2HEnd) cudaEventDestroy(mEventD2HEnd);

        if (mGraphExec) CHECK(cudaGraphExecDestroy(mGraphExec));
        if (mGraph) CHECK(cudaGraphDestroy(mGraph));
        if (mStream) CHECK(cudaStreamDestroy(mStream));

        if (mDeviceInput) CHECK(cudaFree(mDeviceInput));
        if (mDeviceOutput) CHECK(cudaFree(mDeviceOutput));

        if (mHostInput) CHECK(cudaFreeHost(mHostInput));
        if (mHostOutput) CHECK(cudaFreeHost(mHostOutput));
    }

    /**
     * @brief 执行碰撞检测推理
     * 
     * @param[in]  input_raw   输入数据，大小为 batch_size * 12 (floats)
     *                         格式: [state(6), quat(4), phys(2)] 每样本
     *                         - state[0:6]:  状态向量
     *                         - quat[6:10]:  四元数 (w, x, y, z)
     *                         - phys[10:12]: 物理参数
     * @param[out] output_phys 输出向量，会被 resize 为 batch_size * 6
     *                         每样本 6 个 float，为反归一化后的物理输出
     * @param[in]  batch_size  当前批次大小，不能超过 max_batch_size
     * 
     * @warning batch_size 超过 max_batch_size 时会打印错误并直接返回
     */
    void predict(const std::vector<float>& input_raw, std::vector<float>& output_phys, int batch_size) {
        std::lock_guard<std::mutex> lk(mInferMutex);
        if (batch_size > mMaxBatchSize) {
            std::cerr << "[CollisionPredictor] Batch size " << batch_size << " exceeds max " << mMaxBatchSize << std::endl;
            return;
        }

        preprocess_copy_raw(input_raw, batch_size);

        if (mEnableCudaGraph) {
            if (!mGraphExec) {
                CHECK(cudaStreamSynchronize(mStream));
                CHECK(cudaStreamBeginCapture(mStream, cudaStreamCaptureModeGlobal));
                enqueueInference(batch_size);
                CHECK(cudaStreamEndCapture(mStream, &mGraph));
#if defined(CUDART_VERSION) && (CUDART_VERSION >= 11030)
                CHECK(cudaGraphInstantiate(&mGraphExec, mGraph, nullptr, nullptr, 0));
#else
                CHECK(cudaGraphInstantiate(&mGraphExec, mGraph, nullptr, nullptr, 0));
#endif
            }
            CHECK(cudaGraphLaunch(mGraphExec, mStream));
        } else {
            enqueueInference(batch_size);
        }

        CHECK(cudaStreamSynchronize(mStream));
        postprocess_copy_raw(output_phys, batch_size);
    }

    /**
     * @brief 获取输入维度
     * @return 每个样本的输入维度 (12)
     */
    int getInputDim() const { return mInputDim; }

    /**
     * @brief 获取输出维度
     * @return 每个样本的输出维度 (6)
     */
    int getOutputDim() const { return mOutputDim; }

    /**
     * @brief 获取最大批次大小
     * @return 最大批次大小
     */
    int getMaxBatchSize() const { return mMaxBatchSize; }

private:
    int mMaxBatchSize;
    static const int mInputDim = 12;   ///< 输入维度: state(6) + quat(4) + phys(2)
    static const int mOutputDim = 6;   ///< 输出维度: 6
    bool mEnableCudaGraph = true;

    const std::string mInputName = "inputs";
    const std::string mOutputName = "outputs";
    std::unique_ptr<nvinfer1::IRuntime> mRuntime;
    std::unique_ptr<nvinfer1::ICudaEngine> mEngine;
    std::unique_ptr<nvinfer1::IExecutionContext> mContext;
    cudaStream_t mStream = nullptr;

    cudaGraph_t mGraph = nullptr;
    cudaGraphExec_t mGraphExec = nullptr;

    bool mVerboseTiming = false;
    cudaEvent_t mEventH2DStart = nullptr;
    cudaEvent_t mEventH2DEnd = nullptr;
    cudaEvent_t mEventExecStart = nullptr;
    cudaEvent_t mEventExecEnd = nullptr;
    cudaEvent_t mEventD2HStart = nullptr;
    cudaEvent_t mEventD2HEnd = nullptr;

    float* mHostInput = nullptr;
    float* mHostOutput = nullptr;

    std::mutex mInferMutex;
    void* mDeviceInput = nullptr;
    void* mDeviceOutput = nullptr;

    double mLastCpuMemcpyMs = 0.0;

    void loadEngine(const std::string& path) {
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file.good()) {
            std::cerr << "[CollisionPredictor] Failed to open engine: " << path << std::endl;
            abort();
        }
        std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);
        std::vector<char> buffer(size);
        file.read(buffer.data(), size);

        mRuntime.reset(nvinfer1::createInferRuntime(gLogger));
        mEngine.reset(mRuntime->deserializeCudaEngine(buffer.data(), size));
        mContext.reset(mEngine->createExecutionContext());
        CHECK(cudaStreamCreate(&mStream));
        std::cout << "[CollisionPredictor] Engine loaded: " << path << std::endl;
    }

    void allocateMemory() {
        size_t devInBytes = (size_t)mMaxBatchSize * mInputDim * sizeof(float);
        size_t devOutBytes = (size_t)mMaxBatchSize * mOutputDim * sizeof(float);

        CHECK(cudaMalloc(&mDeviceInput, devInBytes));
        CHECK(cudaMalloc(&mDeviceOutput, devOutBytes));

        CHECK(cudaMallocHost((void**)&mHostInput, devInBytes));
        CHECK(cudaMallocHost((void**)&mHostOutput, devOutBytes));

        if (mVerboseTiming) {
            cudaEventCreate(&mEventH2DStart);
            cudaEventCreate(&mEventH2DEnd);
            cudaEventCreate(&mEventExecStart);
            cudaEventCreate(&mEventExecEnd);
            cudaEventCreate(&mEventD2HStart);
            cudaEventCreate(&mEventD2HEnd);
        }
    }

    void warmup() {
        bool backup = mEnableCudaGraph;
        mEnableCudaGraph = false;
        std::vector<float> dummy_in(mMaxBatchSize * mInputDim, 0.1f);
        std::vector<float> dummy_out;
        for (int i = 0; i < 10; ++i) predict(dummy_in, dummy_out, mMaxBatchSize);
        mEnableCudaGraph = backup;
    }

    void enqueueInference(int batch_size) {
        size_t inBytes = (size_t)batch_size * mInputDim * sizeof(float);
        size_t outBytes = (size_t)batch_size * mOutputDim * sizeof(float);

        if (mVerboseTiming && mEventH2DStart) cudaEventRecord(mEventH2DStart, mStream);
        CHECK(cudaMemcpyAsync(mDeviceInput, mHostInput, inBytes, cudaMemcpyHostToDevice, mStream));
        if (mVerboseTiming && mEventH2DEnd) cudaEventRecord(mEventH2DEnd, mStream);

        if (mVerboseTiming && mEventExecStart) cudaEventRecord(mEventExecStart, mStream);
        mContext->setInputShape(mInputName.c_str(), nvinfer1::Dims2(batch_size, mInputDim));
        mContext->setTensorAddress(mInputName.c_str(), mDeviceInput);
        mContext->setTensorAddress(mOutputName.c_str(), mDeviceOutput);
        bool status = mContext->enqueueV3(mStream);
        if (!status) {
            throw std::runtime_error("[CollisionPredictor] TRT enqueue failed");
        }
        if (mVerboseTiming && mEventExecEnd) cudaEventRecord(mEventExecEnd, mStream);

        if (mVerboseTiming && mEventD2HStart) cudaEventRecord(mEventD2HStart, mStream);
        CHECK(cudaMemcpyAsync(mHostOutput, mDeviceOutput, outBytes, cudaMemcpyDeviceToHost, mStream));
        if (mVerboseTiming && mEventD2HEnd) cudaEventRecord(mEventD2HEnd, mStream);
    }

    void preprocess_copy_raw(const std::vector<float>& raw, int batch_size) {
        size_t bytes = (size_t)batch_size * mInputDim * sizeof(float);
        auto t0 = std::chrono::high_resolution_clock::now();
        std::memcpy(mHostInput, raw.data(), bytes);
        auto t1 = std::chrono::high_resolution_clock::now();
        mLastCpuMemcpyMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    }

    void postprocess_copy_raw(std::vector<float>& result, int batch_size) {
        result.resize(batch_size * mOutputDim);
        size_t bytes = (size_t)batch_size * mOutputDim * sizeof(float);
        std::memcpy(result.data(), mHostOutput, bytes);
    }
};

// ============================================================================
//                        StudentPredictor
// ============================================================================
/**
 * @class StudentPredictor
 * @brief Student 蒸馏模型的 TensorRT 推理类
 * 
 * 用于生成 B-spline 控制点轨迹。基于 Transformer 架构的学生网络。
 * 
 * ## 输入格式 (来自 small2onnx.py)
 * 
 * 每个样本包含 14 个 float，格式如下（统一顺序）：
 * ```
 * [task_mask(2), start_pos(2), start_vel(2), start_acc(2), yaw_vector(2), geometry_offset(2), guide_vector(2)]
 * ```
 * 
 * | 字段            | 索引     | 维度 | 描述                                    |
 * |-----------------|----------|------|-----------------------------------------|
 * | task_mask       | [0:2]    | 2    | 任务掩码，用于区分任务类型             |
 * | start_pos       | [2:4]    | 2    | 起始位置 (x, y)                        |
 * | start_vel       | [4:6]    | 2    | 起始速度 (vx, vy)                      |
 * | start_acc       | [6:8]    | 2    | 起始加速度 (ax, ay)                    |
 * | yaw_vector      | [8:10]   | 2    | 偏航向量 (cos, sin)                    |
 * | geometry_offset | [10:12]  | 2    | 几何偏移（目标位置）                   |
 * | guide_vector    | [12:14]  | 2    | 引导向量（目标方向）                   |
 * 
 * ## 输出格式
 * 
 * 每个样本输出 24 个 float (12 个控制点，每点 2D):
 * ```
 * [cp0_x, cp0_y, cp1_x, cp1_y, ..., cp11_x, cp11_y]
 * ```
 * 
 * 输出为物理单位下的控制点坐标，已完成反归一化。
 * - 控制点数量: 12
 * - 每控制点维度: 2 (x, y)
 * - 总输出维度: 12 * 2 = 24
 * 
 * ## 任务掩码说明
 * 
 * task_mask 用于区分不同类型的规划任务：
 * - `[0.0, 1.0]` - 避障任务：最后一个控制点 (cp15) 会被替换为 geometry_offset
 * - `[1.0, 0.0]` - 碰撞任务：第 11 个控制点 (cp11) 会被替换为 geometry_offset
 * - 首个控制点 (cp0) 始终被替换为 start_pos
 * 
 * ## 示例代码
 * ```cpp
 * // 创建 Student 预测器
 * StudentPredictor predictor("student_distilled.engine", 72, true, false);
 * 
 * // 准备输入数据
 * int batch_size = 72;
 * std::vector<float> inputs(batch_size * 14);
 * // 填充每个样本的 14 个 float:
 * // [start_pos(2), start_vel(2), start_acc(2), guide_vector(2), 
 * //  yaw_vector(2), task_mask(2), geometry_offset(2)]
 * 
 * // 推理
 * std::vector<float> outputs;
 * predictor.predict(inputs, outputs, batch_size);
 * // outputs 大小为 batch_size * 24 (12 控制点 * 2 坐标)
 * 
 * // 访问第 i 个样本的第 j 个控制点
 * int i = 0, j = 5;  // 第 0 个样本的第 5 个控制点
 * float cp_x = outputs[i * 24 + j * 2];
 * float cp_y = outputs[i * 24 + j * 2 + 1];
 * ```
 */
class StudentPredictor {
public:
    /**
     * @brief 构造函数，加载引擎并自动预热
     * 
     * @param engine_path    TRT engine 文件路径 (student_distilled.engine)
     * @param max_batch_size 最大批次大小（默认 72）
     * @param use_cuda_graph 是否启用 CUDA Graphs 加速（默认 true）
     * @param verbose_timing 是否启用详细计时事件（默认 false）
     * 
     * @note 构造时会自动进行 8 次预热推理
     */
    StudentPredictor(const std::string& engine_path, int max_batch_size = 72, 
                    bool use_cuda_graph = true, bool verbose_timing = false)
        : mMaxBatchSize(max_batch_size), mEnableCudaGraph(use_cuda_graph), mVerboseTiming(verbose_timing) {
        loadEngine(engine_path);
        allocateMemory();
        warmup();
    }

    /**
     * @brief 析构函数，释放所有 CUDA 资源
     */
    ~StudentPredictor() {
        if (mEventH2DStart) cudaEventDestroy(mEventH2DStart);
        if (mEventH2DEnd) cudaEventDestroy(mEventH2DEnd);
        if (mEventExecStart) cudaEventDestroy(mEventExecStart);
        if (mEventExecEnd) cudaEventDestroy(mEventExecEnd);
        if (mEventD2HStart) cudaEventDestroy(mEventD2HStart);
        if (mEventD2HEnd) cudaEventDestroy(mEventD2HEnd);

        if (mGraphExec) CHECK(cudaGraphExecDestroy(mGraphExec));
        if (mGraph) CHECK(cudaGraphDestroy(mGraph));
        if (mStream) CHECK(cudaStreamDestroy(mStream));

        if (mDeviceInput) CHECK(cudaFree(mDeviceInput));
        if (mDeviceOutput) CHECK(cudaFree(mDeviceOutput));

        if (mHostInput) CHECK(cudaFreeHost(mHostInput));
        if (mHostOutput) CHECK(cudaFreeHost(mHostOutput));
    }

    struct RemappedBatch {
        // Padded output control points in physical units: (B, 21, 2) flattened.
        std::vector<float> control_points_padded;
        // Per-sample valid length: avoidance=18, collision=21.
        std::vector<int> lengths;
        // Collision pre-impact velocity vector (physical units): (B,2) flattened. Avoidance rows are 0.
        std::vector<float> v_pre;
        // Collision post segment time (seconds): (B,) . Avoidance rows are 0.
        std::vector<float> t_post;
    };

    /**
     * @brief 执行 Student 模型推理
     * 
    * @param[in]  input_raw 输入数据，大小为 batch_size * 14 (floats)
    *                       格式: [task_mask(2), start_pos(2), start_vel(2), 
    *                              start_acc(2), yaw_vector(2), geometry_offset(2), guide_vector(2)] 每样本
    *                       详细字段说明：
    *                       - task_mask[0:2]:       任务掩码 [0,1]=避障, [1,0]=碰撞
    *                       - start_pos[2:4]:       起始位置 (x, y)
    *                       - start_vel[4:6]:       起始速度 (vx, vy)
    *                       - start_acc[6:8]:       起始加速度 (ax, ay)
    *                       - yaw_vector[8:10]:     偏航向量 (cos, sin)
    *                       - geometry_offset[10:12]: 几何偏移（目标位置）
    *                       - guide_vector[12:14]:  引导向量
    * @param[out] output_cp 输出向量，会被 resize 为 batch_size * 24
    *                       每样本 24 个 float (12 控制点 * 2D 坐标)
    *                       数据布局: [cp0_x, cp0_y, cp1_x, cp1_y, ..., cp11_x, cp11_y]
     * @param[in]  batch_size 当前批次大小，不能超过 max_batch_size
     * 
     * @warning batch_size 超过 max_batch_size 时会打印错误并直接返回
     */
    void predict(const std::vector<float>& input_raw, std::vector<float>& output_cp, int batch_size) {
        std::lock_guard<std::mutex> lk(mInferMutex);
        predictNoLock(input_raw, output_cp, batch_size);
    }

    // Runs TRT inference then remaps to avoidance(18) / collision(21) control points and computes v_pre.
    // Outputs are padded to (B,21,2) with per-sample lengths.
    void predict_remap(const std::vector<float>& input_raw, RemappedBatch& out, int batch_size, int degree = 5, float T_duration = 4.0f) {
        std::lock_guard<std::mutex> lk(mInferMutex);
        std::vector<float> cp12;
        predictNoLock(input_raw, cp12, batch_size);
        out = remap_batch(input_raw, cp12, batch_size, degree, T_duration);
    }

private:
    void predictNoLock(const std::vector<float>& input_raw, std::vector<float>& output_cp, int batch_size) {
        if (batch_size > mMaxBatchSize) {
            std::cerr << "[StudentPredictor] Batch size " << batch_size << " > max " << mMaxBatchSize << std::endl;
            return;
        }

        preprocess_copy_raw(input_raw, batch_size);

        if (mEnableCudaGraph) {
            if (!mGraphExec) {
                CHECK(cudaStreamSynchronize(mStream));
                CHECK(cudaStreamBeginCapture(mStream, cudaStreamCaptureModeGlobal));
                enqueueInference(batch_size);
                CHECK(cudaStreamEndCapture(mStream, &mGraph));
#if defined(CUDART_VERSION) && (CUDART_VERSION >= 11030)
                CHECK(cudaGraphInstantiate(&mGraphExec, mGraph, nullptr, nullptr, 0));
#else
                CHECK(cudaGraphInstantiate(&mGraphExec, mGraph, nullptr, nullptr, 0));
#endif
            }
            CHECK(cudaGraphLaunch(mGraphExec, mStream));
        } else {
            enqueueInference(batch_size);
        }

        CHECK(cudaStreamSynchronize(mStream));
        postprocess_copy_raw(output_cp, batch_size);
    }

public:
    /**
     * @brief 获取输入维度
     * @return 每个样本的输入维度 (14)
     */
    int getInputDim() const { return mInputDim; }

    /**
     * @brief 获取输出维度
        * @return 每个样本的输出维度 (24 = 12 控制点 * 2)
     */
    int getOutputDim() const { return mOutputDim; }

    /**
     * @brief 获取控制点数量
        * @return 控制点数量 (12)
     */
    int getNumControlPoints() const { return mOutPoints; }

    /**
     * @brief 获取最大批次大小
     * @return 最大批次大小
     */
    int getMaxBatchSize() const { return mMaxBatchSize; }

private:
    int mMaxBatchSize;
    static const int mInputDim = 14;         ///< 输入维度: 14 (context)
    static const int mOutPoints = 12;        ///< 控制点数量
    static const int mOutDimPerPoint = 2;    ///< 每控制点维度 (x, y)
    static const int mOutputDim = mOutPoints * mOutDimPerPoint; ///< 总输出维度: 24

    bool mEnableCudaGraph = true;
    const std::string mInputName = "inputs";
    const std::string mOutputName = "control_points";

    static RemappedBatch remap_batch(const std::vector<float>& input_raw, const std::vector<float>& cp12_raw, int batch_size, int degree, float T_duration) {
        RemappedBatch out;
        if (batch_size <= 0) return out;
        if ((int)input_raw.size() < batch_size * 14) {
            std::cerr << "[StudentPredictor::remap_batch] input_raw too small" << std::endl;
            return out;
        }
        if ((int)cp12_raw.size() < batch_size * 12 * 2) {
            std::cerr << "[StudentPredictor::remap_batch] cp12_raw too small" << std::endl;
            return out;
        }

        using MatB2 = Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor>;
        using MatB42 = Eigen::Matrix<float, Eigen::Dynamic, 42, Eigen::RowMajor>; // 21*2

        Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, 14, Eigen::RowMajor>> ctx(input_raw.data(), batch_size, 14);
        Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, 24, Eigen::RowMajor>> cp(cp12_raw.data(), batch_size, 24);

        const int FULL_HORIZON = 21;
        const float eps = 1e-6f;
        const float k = static_cast<float>(degree);

        // New canonical order: tm, sp, sv, sa, yv, go, gv
        const Eigen::VectorXf task0 = ctx.col(0);                          // tm
        const MatB2 sp = ctx.block(0, 2, batch_size, 2);                   // start_pos
        const MatB2 sv = ctx.block(0, 4, batch_size, 2);                   // start_vel
        const MatB2 sa = ctx.block(0, 6, batch_size, 2);                   // start_acc
        const MatB2 yv = ctx.block(0, 8, batch_size, 2);                   // yaw_vector (cos,sin)
        const MatB2 go = ctx.block(0, 10, batch_size, 2);                  // geometry_offset
        const MatB2 guide = ctx.block(0, 12, batch_size, 2);               // guide_vector

        const MatB2 collision_point = sp + go;

        auto cp_pt = [&](int idx) {
            return cp.block(0, 2 * idx, batch_size, 2);
        };

        // Knot-consistent endpoint reconstruction (matches mpd-splines-public/unified and small/small_deploy.py):
        // Uses derivative-control-point relations under clamped open-uniform knots with u=t/T.
        auto knot_val = [&](int idx, int n_control_points) -> float {
            const int p = degree;
            const float h = 1.0f / static_cast<float>(n_control_points - p);
            if (idx < p) return 0.0f;
            if (idx <= n_control_points) return static_cast<float>(idx - p) * h;
            return 1.0f;
        };

        auto reconstruct_start = [&](const MatB2& p0, const MatB2& v0, const MatB2& a0, const Eigen::ArrayXf& T, int n_control_points) {
            const int p = degree;
            const float pf = static_cast<float>(p);
            const float den_Q0 = std::max(1e-12f, knot_val(p + 1, n_control_points) - knot_val(1, n_control_points));
            const float den_R0 = std::max(1e-12f, knot_val(p + 1, n_control_points) - knot_val(2, n_control_points));
            const float den_Q1 = std::max(1e-12f, knot_val(p + 2, n_control_points) - knot_val(2, n_control_points));

            MatB2 C0 = p0;
            MatB2 Q0 = (v0.array().colwise() * T).matrix();
            MatB2 R0 = (a0.array().colwise() * T.square()).matrix();

            MatB2 C1 = (C0.array() + Q0.array() * (den_Q0 / pf)).matrix();
            MatB2 Q1 = (Q0.array() + R0.array() * (den_R0 / (pf - 1.0f))).matrix();
            MatB2 C2 = (C1.array() + Q1.array() * (den_Q1 / pf)).matrix();

            std::array<MatB2, 3> Cs{C0, C1, C2};
            return Cs;
        };

        auto reconstruct_end = [&](const MatB2& p1, const MatB2& v1, const MatB2& a1, const Eigen::ArrayXf& T, int n_control_points) {
            const int p = degree;
            const float pf = static_cast<float>(p);
            const float den_Q_last = std::max(1e-12f, knot_val(n_control_points + p - 1, n_control_points) - knot_val(n_control_points - 1, n_control_points));
            const float den_R_last = std::max(1e-12f, knot_val(n_control_points + p - 2, n_control_points) - knot_val(n_control_points - 1, n_control_points));
            const float den_Q_prev = std::max(1e-12f, knot_val(n_control_points + p - 2, n_control_points) - knot_val(n_control_points - 2, n_control_points));

            MatB2 Cn_1 = p1;
            MatB2 Q_last = (v1.array().colwise() * T).matrix();
            MatB2 R_last = (a1.array().colwise() * T.square()).matrix();

            MatB2 Cn_2 = (Cn_1.array() - Q_last.array() * (den_Q_last / pf)).matrix();
            MatB2 Q_prev = (Q_last.array() - R_last.array() * (den_R_last / (pf - 1.0f))).matrix();
            MatB2 Cn_3 = (Cn_2.array() - Q_prev.array() * (den_Q_prev / pf)).matrix();

            std::array<MatB2, 3> Ce{Cn_3, Cn_2, Cn_1};
            return Ce;
        };

        const Eigen::ArrayXf T_full = Eigen::ArrayXf::Constant(batch_size, T_duration);
        const MatB2 zeros = MatB2::Zero(batch_size, 2);

        // ---------------- avoidance branch (padded to 21) ----------------
        MatB42 out_avoid = MatB42::Zero(batch_size, 42);
        {
            const MatB2 ep = sp + go;
            const auto Cstart = reconstruct_start(sp, sv, sa, T_full, FULL_HORIZON - 3);  // n=18
            const auto Cend = reconstruct_end(ep, zeros, zeros, T_full, FULL_HORIZON - 3);

            // indices: 0..2 Cstart, 3..14 cp(12), 15..17 Cend, 18..20 zeros
            out_avoid.block(0, 0, batch_size, 2) = Cstart[0];
            out_avoid.block(0, 2, batch_size, 2) = Cstart[1];
            out_avoid.block(0, 4, batch_size, 2) = Cstart[2];
            out_avoid.block(0, 6, batch_size, 24) = cp;
            out_avoid.block(0, 30, batch_size, 2) = Cend[0];
            out_avoid.block(0, 32, batch_size, 2) = Cend[1];
            out_avoid.block(0, 34, batch_size, 2) = Cend[2];
        }

        // ---------------- collision branch ----------------
        MatB42 out_coll = MatB42::Zero(batch_size, 42);
        MatB2 v_pre = MatB2::Zero(batch_size, 2);
        Eigen::ArrayXf t_post = Eigen::ArrayXf::Zero(batch_size);
        {
            const MatB2 last_pred = cp_pt(11);
            const MatB2 ctrl_pre_prev_init = cp_pt(10);

            const Eigen::ArrayXf L_post = (last_pred - collision_point).rowwise().norm().array();
            const Eigen::ArrayXf v_post_mag = guide.rowwise().norm().array();
            t_post = (2.0f * L_post) / (v_post_mag + eps);

            const int n_pre = std::max(FULL_HORIZON - 5, degree + 1); // 16
            const float delta_u_pre = 1.0f / static_cast<float>(n_pre - degree);
            const Eigen::ArrayXf T_pre = (T_full - t_post).max(eps);
            const Eigen::ArrayXf factor_v_pre = (T_pre * delta_u_pre) / k;

            const MatB2 d_pre = collision_point - ctrl_pre_prev_init;
            // v_der0: instantaneous endpoint velocity estimate (keeps legacy behavior)
            const MatB2 v_der0 = (d_pre.array().colwise() / (factor_v_pre + eps)).matrix();

            const auto Cstart_collision = reconstruct_start(sp, sv, sa, T_pre, FULL_HORIZON - 5); // n=16

            // v_avg: local-average endpoint velocity over last `spans` knot spans (no SciPy).
            // Matches recons_average.py:
            //   h = 1/(N-p), delta_u = spans*h
            //   v_avg = (C(1) - C(1-delta_u)) / (T_pre * delta_u)
            const int N_pre = 16;
            const int spans = 2;
            const float w = 0.001f;
            const float h = 1.0f / static_cast<float>(N_pre - degree);
            const float delta_u_avg = std::min(1.0f - 1e-6f, spans * h);
            const float u0 = std::max(0.0f, 1.0f - delta_u_avg);

            // Precompute basis at u0 for clamped open-uniform knots (N_pre=16, degree=5).
            auto basis_open_uniform = [&](int n_ctrl, int p, float u) -> std::vector<float> {
                const int n = n_ctrl;
                const float hh = 1.0f / static_cast<float>(n - p);
                auto U = [&](int idx) -> float {
                    if (idx < p) return 0.0f;
                    if (idx <= n) return static_cast<float>(idx - p) * hh;
                    return 1.0f;
                };

                std::vector<float> prev(n, 0.0f), curr(n, 0.0f);
                for (int i = 0; i < n; ++i) {
                    const float t0 = U(i);
                    const float t1 = U(i + 1);
                    if ((u >= t0 && u < t1) || (u == 1.0f && t1 == 1.0f && i == n - 1)) {
                        prev[i] = 1.0f;
                    }
                }
                for (int d = 1; d <= p; ++d) {
                    std::fill(curr.begin(), curr.end(), 0.0f);
                    for (int i = 0; i < n; ++i) {
                        const float left_denom = U(i + d) - U(i);
                        const float right_denom = U(i + d + 1) - U(i + 1);

                        float left = 0.0f;
                        if (left_denom > 1e-12f) {
                            left = (u - U(i)) / left_denom * prev[i];
                        }
                        float right = 0.0f;
                        if (right_denom > 1e-12f && (i + 1) < n) {
                            right = (U(i + d + 1) - u) / right_denom * prev[i + 1];
                        }
                        curr[i] = left + right;
                    }
                    prev.swap(curr);
                }
                return prev;
            };

            const std::vector<float> basis_u0 = basis_open_uniform(N_pre, degree, u0);

            // Fixed-point iteration for self-consistent v_pre_vec since pre-tail depends on v_pre_vec.
            MatB2 v_pre_vec = v_der0;
            std::array<MatB2, 3> tail_pre{zeros, zeros, zeros};
            for (int it = 0; it < 3; ++it) {
                tail_pre = reconstruct_end(collision_point, v_pre_vec, zeros, T_pre, n_pre); // (Cn_3, Cn_2, Cn_1)

                MatB2 pos0 = MatB2::Zero(batch_size, 2);
                for (int i = 0; i < N_pre; ++i) {
                    const float bi = basis_u0[i];
                    if (bi == 0.0f) continue;
                    if (i < 3) {
                        pos0.noalias() += bi * Cstart_collision[i];
                    } else if (i < 13) {
                        pos0.noalias() += bi * cp_pt(i - 3);
                    } else {
                        pos0.noalias() += bi * tail_pre[i - 13];
                    }
                }

                const MatB2 v_avg = ((collision_point - pos0).array().colwise() / (T_pre * delta_u_avg + eps)).matrix();
                v_pre_vec = (w * v_der0 + (1.0f - w) * v_avg);
            }

            tail_pre = reconstruct_end(collision_point, v_pre_vec, zeros, T_pre, n_pre);
            const MatB2 ctrl_pre_pre_prev = tail_pre[0];
            const MatB2 ctrl_pre_prev = tail_pre[1];
            v_pre = v_pre_vec;

            const auto Cpost_start = reconstruct_start(collision_point, guide, zeros, t_post, 6);
            const auto Cpost_end = reconstruct_end(last_pred, zeros, zeros, t_post, 6);

            out_coll.block(0, 0, batch_size, 2) = Cstart_collision[0];
            out_coll.block(0, 2, batch_size, 2) = Cstart_collision[1];
            out_coll.block(0, 4, batch_size, 2) = Cstart_collision[2];

            // indices 3..12: first 10 points from cp -> 20 floats
            out_coll.block(0, 6, batch_size, 20) = cp.block(0, 0, batch_size, 20);

            // indices 13..14
            out_coll.block(0, 26, batch_size, 2) = ctrl_pre_pre_prev;
            out_coll.block(0, 28, batch_size, 2) = ctrl_pre_prev;

            // indices 15..20: Cpost (6 points)
            out_coll.block(0, 30, batch_size, 2) = Cpost_start[0];
            out_coll.block(0, 32, batch_size, 2) = Cpost_start[1];
            out_coll.block(0, 34, batch_size, 2) = Cpost_start[2];
            out_coll.block(0, 36, batch_size, 2) = Cpost_end[0];
            out_coll.block(0, 38, batch_size, 2) = Cpost_end[1];
            out_coll.block(0, 40, batch_size, 2) = Cpost_end[2];
        }

        // ---------------- select per-sample (vectorized blend) ----------------
        const Eigen::ArrayXf is_collision = (task0.array() >= 0.5f).cast<float>();
        const Eigen::ArrayXf is_avoid = 1.0f - is_collision;

        MatB42 out_padded = (out_avoid.array().colwise() * is_avoid + out_coll.array().colwise() * is_collision).matrix();
        MatB2 v_pre_sel = (v_pre.array().colwise() * is_collision).matrix();

        out.control_points_padded.resize((size_t)batch_size * 21 * 2);
        std::memcpy(out.control_points_padded.data(), out_padded.data(), (size_t)batch_size * 21 * 2 * sizeof(float));

        out.v_pre.resize((size_t)batch_size * 2);
        std::memcpy(out.v_pre.data(), v_pre_sel.data(), (size_t)batch_size * 2 * sizeof(float));

        out.t_post.resize((size_t)batch_size);
        for (int b = 0; b < batch_size; ++b) {
            out.t_post[(size_t)b] = (task0(b) >= 0.5f) ? t_post(b) : 0.0f;
        }

        out.lengths.resize(batch_size);
        for (int b = 0; b < batch_size; ++b) {
            out.lengths[b] = (task0(b) >= 0.5f) ? 21 : 18;
        }
        return out;
    }

    std::unique_ptr<nvinfer1::IRuntime> mRuntime;
    std::unique_ptr<nvinfer1::ICudaEngine> mEngine;
    std::unique_ptr<nvinfer1::IExecutionContext> mContext;
    cudaStream_t mStream = nullptr;

    cudaGraph_t mGraph = nullptr;
    cudaGraphExec_t mGraphExec = nullptr;

    bool mVerboseTiming = false;
    cudaEvent_t mEventH2DStart = nullptr;
    cudaEvent_t mEventH2DEnd = nullptr;
    cudaEvent_t mEventExecStart = nullptr;
    cudaEvent_t mEventExecEnd = nullptr;
    cudaEvent_t mEventD2HStart = nullptr;
    cudaEvent_t mEventD2HEnd = nullptr;

    float* mHostInput = nullptr;
    float* mHostOutput = nullptr;

    std::mutex mInferMutex;
    void* mDeviceInput = nullptr;
    void* mDeviceOutput = nullptr;

    double mLastCpuMemcpyMs = 0.0;

    void loadEngine(const std::string& path) {
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file.good()) {
            std::cerr << "[StudentPredictor] Failed to open engine: " << path << std::endl;
            abort();
        }
        std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);
        std::vector<char> buffer(size);
        file.read(buffer.data(), size);

        mRuntime.reset(nvinfer1::createInferRuntime(gLogger));
        mEngine.reset(mRuntime->deserializeCudaEngine(buffer.data(), size));
        mContext.reset(mEngine->createExecutionContext());
        CHECK(cudaStreamCreate(&mStream));
        std::cout << "[StudentPredictor] Engine loaded: " << path << std::endl;
    }

    void allocateMemory() {
        size_t devInBytes = (size_t)mMaxBatchSize * mInputDim * sizeof(float);
        size_t devOutBytes = (size_t)mMaxBatchSize * mOutputDim * sizeof(float);

        CHECK(cudaMalloc(&mDeviceInput, devInBytes));
        CHECK(cudaMalloc(&mDeviceOutput, devOutBytes));

        CHECK(cudaMallocHost((void**)&mHostInput, devInBytes));
        CHECK(cudaMallocHost((void**)&mHostOutput, devOutBytes));

        if (mVerboseTiming) {
            cudaEventCreate(&mEventH2DStart);
            cudaEventCreate(&mEventH2DEnd);
            cudaEventCreate(&mEventExecStart);
            cudaEventCreate(&mEventExecEnd);
            cudaEventCreate(&mEventD2HStart);
            cudaEventCreate(&mEventD2HEnd);
        }
    }

    void warmup() {
        bool backup = mEnableCudaGraph;
        mEnableCudaGraph = false;
        std::vector<float> dummy_in(mMaxBatchSize * mInputDim, 0.1f);
        std::vector<float> dummy_out;
        for (int i = 0; i < 8; ++i) predict(dummy_in, dummy_out, mMaxBatchSize);
        mEnableCudaGraph = backup;
    }

    void enqueueInference(int batch_size) {
        size_t inBytes = (size_t)batch_size * mInputDim * sizeof(float);
        size_t outBytes = (size_t)batch_size * mOutputDim * sizeof(float);

        if (mVerboseTiming && mEventH2DStart) cudaEventRecord(mEventH2DStart, mStream);
        CHECK(cudaMemcpyAsync(mDeviceInput, mHostInput, inBytes, cudaMemcpyHostToDevice, mStream));
        if (mVerboseTiming && mEventH2DEnd) cudaEventRecord(mEventH2DEnd, mStream);

        if (mVerboseTiming && mEventExecStart) cudaEventRecord(mEventExecStart, mStream);
        mContext->setInputShape(mInputName.c_str(), nvinfer1::Dims2(batch_size, mInputDim));
        mContext->setTensorAddress(mInputName.c_str(), mDeviceInput);
        mContext->setTensorAddress(mOutputName.c_str(), mDeviceOutput);
        bool status = mContext->enqueueV3(mStream);
        if (!status) {
            throw std::runtime_error("[StudentPredictor] TRT enqueue failed");
        }
        if (mVerboseTiming && mEventExecEnd) cudaEventRecord(mEventExecEnd, mStream);

        if (mVerboseTiming && mEventD2HStart) cudaEventRecord(mEventD2HStart, mStream);
        CHECK(cudaMemcpyAsync(mHostOutput, mDeviceOutput, outBytes, cudaMemcpyDeviceToHost, mStream));
        if (mVerboseTiming && mEventD2HEnd) cudaEventRecord(mEventD2HEnd, mStream);
    }

    void preprocess_copy_raw(const std::vector<float>& raw, int batch_size) {
        size_t bytes = (size_t)batch_size * mInputDim * sizeof(float);
        auto t0 = std::chrono::high_resolution_clock::now();
        std::memcpy(mHostInput, raw.data(), bytes);
        auto t1 = std::chrono::high_resolution_clock::now();
        mLastCpuMemcpyMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    }

    void postprocess_copy_raw(std::vector<float>& out, int batch_size) {
        out.resize(batch_size * mOutputDim);
        size_t bytes = (size_t)batch_size * mOutputDim * sizeof(float);
        std::memcpy(out.data(), mHostOutput, bytes);
    }
};