#pragma once

// SDFormat
#include <sdf/sdf.hh>


class SDFinterface {
    // SDFormat
    sdf::SDFPtr sdf_;
    sdf::ElementPtr sdf_root_, sdf_world_;

    int model_count;

    public:
        SDFinterface();
        ~SDFinterface();

        // Functions for adding models to SDF tree
        void AddPlane(double x, double y, double z,
                      double R, double P, double Y,
                      double L, double W,
                      const std::string &frame_id);
        void AddBox(double x, double y, double z,
                    double R, double P, double Y,
                    double L, double W, double H,
                    const std::string &frame_id);
        void AddSphere(double x, double y, double z, double radius,
                       const std::string &frame_id);
        void AddCylinder(double x, double y, double z,
                         double R, double P, double Y,
                         double radius, double length,
                         const std::string &frame_id);

        void WriteSDF(const std::string &filename);
};