#ifndef G2O_ISOMETRY3D_MAPPINGS_H_
#define G2O_ISOMETRY3D_MAPPINGS_H_

#include <Eigen/Core>
#include <Eigen/Dense>  // Ensure access to Eigen::Vector6d and Eigen::Vector7d
#include <Eigen/Geometry>  // Required for Eigen::Quaternion

#include "se3quat.h"
#include "../core/eigen_types.h"
#include "g2o_types_slam3d_api.h"

namespace g2o
{
    namespace internal
    {
        template <typename Derived>
        bool writeVector(std::ostream &os, const Eigen::DenseBase<Derived> &b)
        {
            for (int i = 0; i < b.size(); i++)
                os << b(i) << " ";
            return os.good();
        }

        template <typename Derived>
        bool readVector(std::istream &is, Eigen::DenseBase<Derived> &b)
        {
            for (int i = 0; i < b.size() && is.good(); i++)
                is >> b(i);
            return is.good() || is.eof();
        }

        /**
         * Extract the rotation matrix from an Isometry3 matrix
         */
        inline Isometry3D::ConstLinearPart extractRotation(const Isometry3D &A)
        {
            return A.matrix().topLeftCorner<3, 3>();
        }

        /**
         * Normalize the quaternion, such that ||q|| == 1 and q.w() > 0
         */
        template<typename Scalar>
        G2O_TYPES_SLAM3D_API Eigen::Quaternion<Scalar> normalized(const Eigen::Quaternion<Scalar> &q)
        {
            Eigen::Quaternion<Scalar> result = q;
            result.normalize();
            if (result.w() < Scalar(0))
                result.coeffs() = -result.coeffs();  // Ensure w() > 0
            return result;
        }

        /**
         * Normalize the quaternion in-place
         */
        G2O_TYPES_SLAM3D_API Eigen::Quaterniond &normalize(Eigen::Quaterniond &q)
        {
            q.normalize();
            if (q.w() < 0)
                q.coeffs() = -q.coeffs();  // Ensure w() > 0
            return q;
        }

        /**
         * Converts a Rotation Matrix to (qx, qy, qz)
         */
        G2O_TYPES_SLAM3D_API Eigen::VectorXd toCompactQuaternion(const Eigen::Matrix3d &R);

        /**
         * Converts an Isometry3D to (x, y, z, qx, qy, qz)
         */
        G2O_TYPES_SLAM3D_API Eigen::VectorXd toVectorMQT(const Isometry3D &t);

        /**
         * Converts an Isometry3D to (x, y, z, qx, qy, qz, qw)
         */
        G2O_TYPES_SLAM3D_API Eigen::VectorXd toVectorQT(const Isometry3D &t);

        /**
         * Converts a (x, y, z, qx, qy, qz, qw) to an Isometry3D
         */
        G2O_TYPES_SLAM3D_API Isometry3D fromVectorQT(const Eigen::VectorXd &v);

    } // end namespace internal
} // end namespace g2o

#endif
