// Ceres solver
#include <ceres/ceres.h>

// Ceres cost functors
struct ConeCostFunctor {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ConeCostFunctor(Eigen::Vector3d SurfacePoint) : SurfacePoint(SurfacePoint) {}

    template <class T>
    bool operator()(T const* const sAxisPoint_a,
                    T const* const sAxisPoint_b,
                    T const* const sAngle,
                    T* sResiduals) const
    {
        using Vector3T = Eigen::Matrix<T, 3, 1>;
        Eigen::Map<Vector3T const> const AxisPoint_a(sAxisPoint_a);
        Eigen::Map<Vector3T const> const AxisPoint_b(sAxisPoint_b);

        sResiduals[0] = (AxisPoint_b - AxisPoint_a).cross(AxisPoint_a - SurfacePoint.cast<T>() ).squaredNorm() /
                        (AxisPoint_b - AxisPoint_a).squaredNorm() -
                        (SurfacePoint.cast<T>() - AxisPoint_b).squaredNorm() *
                        ceres::sin(T(sAngle[0]) )*ceres::sin(T(sAngle[0]) );

                        // (SurfacePoint.cast<T>() - AxisPoint_b).dot((AxisPoint_a - AxisPoint_b) /
                        // (AxisPoint_a - AxisPoint_b).norm() ) *
                        // (SurfacePoint.cast<T>() - AxisPoint_b).dot((AxisPoint_a - AxisPoint_b) /
                        // (AxisPoint_a - AxisPoint_b).norm() ) * T(ceres::tan(sAngle[0]) )*T(ceres::tan(sAngle[0]) );

        return true;
    }

    Eigen::Vector3d SurfacePoint;
};

struct CylinderCostFunctor {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CylinderCostFunctor(Eigen::Vector3d SurfacePoint) : SurfacePoint(SurfacePoint) {}

    template <class T>
    bool operator()(T const* const sAxisPoint_a,
                    T const* const sAxisPoint_b,
                    T const* const sRadius,
                    T* sResiduals) const
    {
        using Vector3T = Eigen::Matrix<T, 3, 1>;
        Eigen::Map<Vector3T const> const AxisPoint_a(sAxisPoint_a);
        Eigen::Map<Vector3T const> const AxisPoint_b(sAxisPoint_b);

        sResiduals[0] = (AxisPoint_b - AxisPoint_a).cross(AxisPoint_a - SurfacePoint.cast<T>() ).squaredNorm() /
                        (AxisPoint_b - AxisPoint_a).squaredNorm() - T(sRadius[0])*T(sRadius[0]);

        return true;
    }

    Eigen::Vector3d SurfacePoint;
};

struct SphereCostFunctor {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SphereCostFunctor(Eigen::Vector3d SurfacePoint) : SurfacePoint(SurfacePoint) {}

    template <class T>
    bool operator()(T const* const sCenterPoint,
                    T const* const sRadius,
                    T* sResiduals) const
    {
        using Vector3T = Eigen::Matrix<T, 3, 1>;
        Eigen::Map<Vector3T const> const CenterPoint(sCenterPoint);

        sResiduals[0] = (SurfacePoint.cast<T>() - CenterPoint).squaredNorm() - T(sRadius[0])*T(sRadius[0]);

        return true;
    }

    Eigen::Vector3d SurfacePoint;
};


// struct CylinderCostFunctor {
//     CylinderCostFunctor(double p[3]) : p_{p[0], p[1], p[2]} {}

//     template <class T>
//     bool operator()(const T* const x, T* residual) const
//     {
//         residual[0] = (((T(p_[0]) - x[0])*(x[1] - x[4]) - (T(p_[1]) - x[1])*(x[0] - x[3]) ) *
//                        ((T(p_[0]) - x[0])*(x[1] - x[4]) - (T(p_[1]) - x[1])*(x[0] - x[3]) ) +

//                        ((T(p_[0]) - x[0])*(x[2] - x[5]) - (T(p_[2]) - x[2])*(x[0] - x[3]) ) *
//                        ((T(p_[0]) - x[0])*(x[2] - x[5]) - (T(p_[2]) - x[2])*(x[0] - x[3]) ) +

//                        ((T(p_[1]) - x[1])*(x[2] - x[5]) - (T(p_[2]) - x[2])*(x[1] - x[4]) ) *
//                        ((T(p_[1]) - x[1])*(x[2] - x[5]) - (T(p_[2]) - x[2])*(x[1] - x[4]) ) ) /

//                        ((x[0] - x[3])*(x[0] - x[3]) + 
//                         (x[1] - x[4])*(x[1] - x[4]) +
//                         (x[2] - x[5])*(x[2] - x[5]) ) - x[6]*x[6];

//         return true;
//     }

//     double p_[3];
// };