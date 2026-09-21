#pragma once

#include <Eigen/Core>
#include <fstream>
#include <sstream>

namespace utils {

template<typename T, size_t N>
Eigen::Matrix<T, N, 1> Array2Eigen(std::array<T, N> &array) {
    return Eigen::Map<Eigen::Matrix<T, N, 1>>(array.data());
}

template<typename T, size_t N>
Eigen::Matrix<T, N, 1> Vector2Eigen(std::vector<T> vector) {
    return Eigen::Map<Eigen::Matrix<T, N, 1>>(vector.data());
}

template<typename T, size_t N>
std::array<T, N> Vector2Array(std::vector<T> vector) {
    std::array<T, N> array;
    for (size_t i = 0; i < N; ++i) {
        array[i] = vector[i];
    }
    return array;
}

const static Eigen::IOFormat _CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");

template<typename T, size_t N>
const Eigen::WithFormat<Eigen::Transpose<Eigen::Matrix<T, N, 1>>> Format2CSV(Eigen::Matrix<T, N, 1> &data) {
    return data.transpose().format(_CSVFormat);
}

template<typename T, size_t N>
std::string Format2CSV(std::array<T, N> &data) {
    // return Array2Eigen(data).transpose().format(_CSVFormat); // 有问题
    std::stringstream stream;
    for (size_t i = 0; i < N; ++i) {
        stream << data[i];
        if (i + 1 < N) {
            stream << ", ";
        }
    }
    return stream.str();
}

template<typename T, size_t N>
std::string Format2CSV(T &data) {
    std::stringstream stream;
    stream << data;
    return stream.str();
}

// void DebugMat(Eigen::MatrixXd mat, const char *path) {
//     const static Eigen::IOFormat _CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
//     std::ofstream outFile;

//     outFile.open(path, std::ios::out);
//     outFile << mat.format(_CSVFormat) << std::endl;
//     outFile.close();
// }

}  // namespace utils
