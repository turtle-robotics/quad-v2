#pragma once

#include <Eigen/Dense>

#ifdef DEBUG
#define QUAD_ASSERT(cnd, msg)                                                         \
    do {                                                                              \
        static_assert(                                                                \
                !std::is_pointer_v<decltype(cnd)>,                                    \
                "Do not use QUAD_ASSERT with raw pointers"                            \
                "and instead do QUAD_ASSERT(cnd != nullptr) "                         \
                "to avoid implicit pointer-to-bool conversion.");                     \
        if (bool(cnd) == false) {                                                     \
            std::cerr << "QUAD_ASSERT failed: " << #cnd << std::endl;                 \
            std::cerr << "QUAD_ASSERT message: " << msg << std::endl;                 \
            std::cerr << "QUAD_ASSERT failed at: ";                                   \
            std::cerr << __func__ << " " << __FILE__ << " " << __LINE__ << std::endl; \
            std::exit(1);                                                             \
        }                                                                             \
    } while(0)
#else
#define QUAD_ASSERT(cnd, msg)
#endif


namespace turtle::quad {
    using namespace Eigen;
    int example(const int a, const int b) {
        return a + b;
    }
}
