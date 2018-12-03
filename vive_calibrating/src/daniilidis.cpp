#include "vive_robot_calibrating_node.h"

// Eigen::Matrix3d AntisymmetricMatrix(const Eigen::Vector3d &eigen_v_);
// Eigen::Matrix<double, 6, 8> ConstructSMatrix(const Eigen::Quaterniond &eigen_qrA_,
//                                              const Eigen::Quaterniond &eigen_qdA_,
//                                              const Eigen::Quaterniond &eigen_qrB_,
//                                              const Eigen::Quaterniond &eigen_qdB_);

// void TransformToDualQuaternion(const geometry_msgs::Transform &tf_msg_,
//                                Eigen::Quaterniond &eigen_qr_,
//                                Eigen::Quaterniond &eigen_qd_);
// Eigen::MatrixXd ConstructTMatrix(const geometry_msgs::Transform tf_Ta_[],
//                                  const geometry_msgs::Transform tf_Tb_[],
//                                  const int &size);

void TransformToDualQuaternion(const geometry_msgs::Transform &tf_msg_,
                                                Eigen::Quaterniond &eigen_qr_,
                                                Eigen::Quaterniond &eigen_qd_)
{
     /**
      * Compute the screw representation of a transform as a dual quaternion.
      * 
      * qr - real quaternion (rotation)
      * qd - dual quaternion (translation)
      */
    
    // The real quaternion is simply the rotation part of the transform
    tf2::fromMsg(tf_msg_.rotation, eigen_qr_);

    Eigen::Vector3d eigen_t_;
    tf2::fromMsg(tf_msg_.translation, eigen_t_);

    eigen_qd_.w() = -0.5*(eigen_qr_.vec() ).transpose()*eigen_t_;
    eigen_qd_.vec() = 0.5*(eigen_qr_.w()*eigen_t_ + eigen_t_.cross(eigen_qr_.vec() ) );
}

Eigen::Matrix3d AntisymmetricMatrix(const Eigen::Vector3d &eigen_v_) {
     /**
      * Compute the antisymmetric matrix corresponding to the 
      * cross-product with the provided vector.
      */
    
    Eigen::Matrix3d eigen_asmat_;
    eigen_asmat_ <<             0 , -eigen_v_(2),  eigen_v_(1),
                     eigen_v_(2),             0 , -eigen_v_(0),
                    -eigen_v_(1),  eigen_v_(0),             0 ;
    
    return eigen_asmat_;
}

Eigen::Matrix<double, 6, 8> ConstructSMatrix(const Eigen::Quaterniond &eigen_qrA_,
                                                              const Eigen::Quaterniond &eigen_qdA_,
                                                              const Eigen::Quaterniond &eigen_qrB_,
                                                              const Eigen::Quaterniond &eigen_qdB_)
{
     /**
      * Construct a 6x8 matrix corresponding to a single instance 
      * of the hand-eye calibration problem with dual quaternions.
      * 
      * The S matrix is given by equation 31 in Daniilidis (1999):
      * Hand-Eye Calibration Using Dual Quaternions
      */
    
    Eigen::Matrix<double, 3, 4> eigen_Sdiag_;
    eigen_Sdiag_ << eigen_qrA_.vec() - eigen_qrB_.vec(),
                    AntisymmetricMatrix(eigen_qrA_.vec() + eigen_qrB_.vec() );
    
    Eigen::Matrix<double, 6, 8> eigen_S_;
    eigen_S_ << eigen_Sdiag_, Eigen::Matrix<double, 3, 4>::Constant(0.),
                eigen_qdA_.vec() - eigen_qdB_.vec(),
                AntisymmetricMatrix(eigen_qdA_.vec() + eigen_qdB_.vec() ),
                eigen_Sdiag_;
    
    return eigen_S_;
}

Eigen::MatrixXd ConstructTMatrix(const geometry_msgs::Transform tf_Ta_[],
                                                  const geometry_msgs::Transform tf_Tb_[],
                                                  const int &size)
{
     /**
      * Construct a 6*nx8 matrix T corresponding to the complete
      * hand-eye calibration problem with dual quaternions.
      * 
      * The T matrix is given as equation 33 in Daniilidis (1999):
      * Hand-Eye Calibration Using Dual Quaternions
      */
    
    Eigen::Quaterniond eigen_qrA_, eigen_qdA_, eigen_qrB_, eigen_qdB_;
    Eigen::MatrixXd eigen_T_(6*size, 8);

    for (int i = 0; i < size; i++) {
        TransformToDualQuaternion(tf_Ta_[i], eigen_qrA_, eigen_qdA_);
        TransformToDualQuaternion(tf_Tb_[i], eigen_qrB_, eigen_qdB_);

        eigen_T_.block<6, 8>(6*i, 0) = ConstructSMatrix(eigen_qrA_, eigen_qdA_,
                                                        eigen_qrB_, eigen_qdB_);
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd_(eigen_T_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector4d eigen_u_[2] = {svd_.matrixV().matrix().block<4, 1>(0, 6),
                                   svd_.matrixV().matrix().block<4, 1>(0, 7) };
    Eigen::Vector4d eigen_v_[2] = {svd_.matrixV().matrix().block<4, 1>(4, 6),
                                   svd_.matrixV().matrix().block<4, 1>(4, 7) };

    ROS_INFO_STREAM(svd_.singularValues() );

    double a = eigen_u_[0].dot(eigen_v_[0]);
    double b = eigen_u_[0].dot(eigen_v_[1]) + eigen_u_[1].dot(eigen_v_[0]);
    double c = eigen_u_[1].dot(eigen_v_[1]);

    double s[2] = {(-b + std::sqrt(b*b - 4*a*c) )/(2*a),
                   (-b - std::sqrt(b*b - 4*a*c) )/(2*a) };

    a = eigen_u_[0].dot(eigen_u_[0]);
    b = 2*eigen_u_[0].dot(eigen_u_[1]);
    c = eigen_u_[1].dot(eigen_u_[1]);

    double d[2] = {s[0]*s[0]*a + 2*b*s[0] + c,
                   s[1]*s[1]*a + 2*b*s[1] + c};
    
    double lambda[2];
    // Pick solution s corresponding to the smallest lambda value
    if (d[0] >= d[1]) {
        lambda[1] = 1/std::sqrt(d[0]);
        lambda[0] = s[0]*lambda[1];
    } else {
        lambda[1] = 1/std::sqrt(d[1]);
        lambda[0] = s[1]*lambda[1];
    }

    Eigen::Quaterniond eigen_qrX_(lambda[0]*eigen_u_[0] + lambda[1]*eigen_u_[1]);
    eigen_qrX_.normalize();

    Eigen::Matrix3d eigen_rotmat_;
    eigen_rotmat_ << 0, 1, 0, 0, 0, 1, -1, 0, 0; 

    Eigen::Quaterniond eigen_qdX_(lambda[0]*eigen_v_[0] + lambda[1]*eigen_v_[1]);
    Eigen::Vector3d eigen_t_ = 2.*(eigen_qdX_*eigen_qrX_.conjugate() ).vec();

    ROS_INFO_STREAM(std::endl << eigen_qrX_.toRotationMatrix()*eigen_rotmat_ << std::endl << eigen_rotmat_*eigen_t_);
}