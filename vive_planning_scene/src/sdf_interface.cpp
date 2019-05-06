#include "vive_planning_scene/sdf_interface.h"


SDFinterface::SDFinterface() :
    sdf_(new sdf::SDF() )
{
    // SDFormat
    sdf::init(sdf_);

    // Add root element
    sdf_root_ = sdf_->Root();
    // Add world element
    sdf_world_ = sdf_root_->AddElement("world");
    sdf_world_->GetAttribute("name")->Set("robot cell");
}
SDFinterface::~SDFinterface() {
}

void SDFinterface::WriteSDF(const std::string &filename) {
      /**
     * Write the current SDF tree to an SDF file named filename
     * (".sdf" filetype is automatically added to filename)
     */

    sdf_->Write(filename + ".sdf");
}


void SDFinterface::AddPlane(double x, double y, double z,
                            double R, double P, double Y,
                            double L, double W,
                            const std::string &frame_id)
{
      /**
     * Adds a plane model with center at (x, y, z),
     * roll R, pitch Y, yaw Y with respect to frame_id,
     * defined by its length L and width W to the SDF tree.
     */

    // Increment model counter
    model_count++;
    // Add model to SDF tree
    sdf::ElementPtr sdf_model_ = sdf_world_->AddElement("model");
    sdf_model_->GetAttribute("name")->Set("plane_" + std::to_string(model_count) );

    // Make model immovable, i.e. disable body dynamics
    sdf::ElementPtr sdf_static_ = sdf_model_->AddElement("static");
    sdf_static_->Set("1");

    // Set plane pose (center)
    sdf::ElementPtr sdf_pose_ = sdf_model_->AddElement("pose");
    sdf_pose_->Set(std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + " " +
                   std::to_string(R) + " " + std::to_string(P) + " " + std::to_string(Y) );
    sdf_pose_->GetAttribute("frame")->Set(frame_id);

    // Add a single link to the model
    sdf::ElementPtr sdf_link_ = sdf_model_->AddElement("link");
    sdf_link_->GetAttribute("name")->Set("link");

    // Collision geometry
    sdf::ElementPtr sdf_collision_ = sdf_link_->AddElement("collision");
    sdf_collision_->GetAttribute("name")->Set("collision");

    sdf::ElementPtr sdf_collision_geometry_ = sdf_collision_->GetElement("geometry");
    sdf::ElementPtr sdf_collision_plane_ = sdf_collision_geometry_->AddElement("plane");
    sdf::ElementPtr sdf_collision_plane_size = sdf_collision_plane_->GetElement("size");
    sdf_collision_plane_size->Set(std::to_string(L) + " " + std::to_string(W) );
    // Visual geometry
    sdf::ElementPtr sdf_visual_ = sdf_link_->AddElement("visual");
    sdf_visual_->GetAttribute("name")->Set("visual");

    sdf::ElementPtr sdf_visual_geometry_ = sdf_visual_->GetElement("geometry");
    sdf::ElementPtr sdf_visual_plane_ = sdf_visual_geometry_->AddElement("plane");
    sdf::ElementPtr sdf_visual_plane_size = sdf_visual_plane_->GetElement("size");
    sdf_collision_plane_size->Set(std::to_string(L) + " " + std::to_string(W) );
}

void SDFinterface::AddBox(double x, double y, double z,
                          double R, double P, double Y,
                          double L, double W, double H,
                          const std::string &frame_id)
{
      /**
     * Adds a box model with center at (x, y, z),
     * roll R, pitch Y, yaw Y with respect to frame_id,
     * defined by its length L, width W and height H to the SDF tree.
     */

    // Increment model counter
    model_count++;
    // Add model to SDF tree
    sdf::ElementPtr sdf_model_ = sdf_world_->AddElement("model");
    sdf_model_->GetAttribute("name")->Set("box_" + std::to_string(model_count) );

    // Make model immovable, i.e. disable body dynamics
    sdf::ElementPtr sdf_static_ = sdf_model_->AddElement("static");
    sdf_static_->Set("1");

    // Set box pose (center)
    sdf::ElementPtr sdf_pose_ = sdf_model_->AddElement("pose");
    sdf_pose_->Set(std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + " " +
                   std::to_string(R) + " " + std::to_string(P) + " " + std::to_string(Y) );
    sdf_pose_->GetAttribute("frame")->Set(frame_id);

    // Add a single link to the model
    sdf::ElementPtr sdf_link_ = sdf_model_->AddElement("link");
    sdf_link_->GetAttribute("name")->Set("link");

    // Collision geometry
    sdf::ElementPtr sdf_collision_ = sdf_link_->AddElement("collision");
    sdf_collision_->GetAttribute("name")->Set("collision");

    sdf::ElementPtr sdf_collision_geometry_ = sdf_collision_->GetElement("geometry");
    sdf::ElementPtr sdf_collision_box_ = sdf_collision_geometry_->AddElement("box");
    sdf::ElementPtr sdf_collision_box_size = sdf_collision_box_->GetElement("size");
    sdf_collision_box_size->Set(std::to_string(L) + " " + std::to_string(W) + " " + std::to_string(H) );
    // Visual geometry
    sdf::ElementPtr sdf_visual_ = sdf_link_->AddElement("visual");
    sdf_visual_->GetAttribute("name")->Set("visual");

    sdf::ElementPtr sdf_visual_geometry_ = sdf_visual_->GetElement("geometry");
    sdf::ElementPtr sdf_visual_box_ = sdf_visual_geometry_->AddElement("box");
    sdf::ElementPtr sdf_visual_box_size = sdf_visual_box_->GetElement("size");
    sdf_visual_box_size->Set(std::to_string(L) + " " + std::to_string(W) + " " + std::to_string(H) );
}

void SDFinterface::AddSphere(double x, double y, double z, double radius,
                             const std::string &frame_id)
{
      /**
     * Adds a sphere model with center at (x, y, z) with respect to frame_id,
     * defined by its radius to the SDF tree.
     */

    // Increment model counter
    model_count++;
    // Add model to SDF tree
    sdf::ElementPtr sdf_model_ = sdf_world_->AddElement("model");
    sdf_model_->GetAttribute("name")->Set("sphere_" + std::to_string(model_count) );

    // Make model immovable, i.e. disable body dynamics
    sdf::ElementPtr sdf_static_ = sdf_model_->AddElement("static");
    sdf_static_->Set("1");

    // Set sphere pose (center), orientation does not matter in this case
    sdf::ElementPtr sdf_pose_ = sdf_model_->AddElement("pose");
    sdf_pose_->Set(std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + " 0 0 0");
    sdf_pose_->GetAttribute("frame")->Set(frame_id);

    // Add a single link to the model
    sdf::ElementPtr sdf_link_ = sdf_model_->AddElement("link");
    sdf_link_->GetAttribute("name")->Set("link");

    // Collision geometry
    sdf::ElementPtr sdf_collision_ = sdf_link_->AddElement("collision");
    sdf_collision_->GetAttribute("name")->Set("collision");

    sdf::ElementPtr sdf_collision_geometry_ = sdf_collision_->GetElement("geometry");
    sdf::ElementPtr sdf_collision_sphere_ = sdf_collision_geometry_->AddElement("sphere");
    sdf_collision_sphere_->GetElement("radius")->Set(std::to_string(radius) );
    // Visual geometry
    sdf::ElementPtr sdf_visual_ = sdf_link_->AddElement("visual");
    sdf_visual_->GetAttribute("name")->Set("visual");

    sdf::ElementPtr sdf_visual_geometry_ = sdf_visual_->GetElement("geometry");
    sdf::ElementPtr sdf_visual_sphere_ = sdf_visual_geometry_->AddElement("sphere");
    sdf_visual_sphere_->GetElement("radius")->Set(std::to_string(radius) );
}

void SDFinterface::AddCylinder(double x, double y, double z,
                               double R, double P, double Y,
                               double radius, double length,
                               const std::string &frame_id)
{
      /**
     * Adds a cylinder model with center at (x, y, z),
     * roll R, pitch Y, yaw Y with respect to frame_id,
     * defined by its radius and length to the SDF tree.
     */

    // Increment model counter
    model_count++;
    // Add model to SDF tree
    sdf::ElementPtr sdf_model_ = sdf_world_->AddElement("model");
    sdf_model_->GetAttribute("name")->Set("cylinder_" + std::to_string(model_count) );

    // Make model immovable, i.e. disable body dynamics
    sdf::ElementPtr sdf_static_ = sdf_model_->AddElement("static");
    sdf_static_->Set("1");

    // Set cylinder pose (center)
    sdf::ElementPtr sdf_pose_ = sdf_model_->AddElement("pose");
    sdf_pose_->Set(std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + " " +
                   std::to_string(R) + " " + std::to_string(P) + " " + std::to_string(Y) );
    sdf_pose_->GetAttribute("frame")->Set(frame_id);

    // Add a single link to the model
    sdf::ElementPtr sdf_link_ = sdf_model_->AddElement("link");
    sdf_link_->GetAttribute("name")->Set("link");

    // Collision geometry
    sdf::ElementPtr sdf_collision_ = sdf_link_->AddElement("collision");
    sdf_collision_->GetAttribute("name")->Set("collision");

    sdf::ElementPtr sdf_collision_geometry_ = sdf_collision_->GetElement("geometry");
    sdf::ElementPtr sdf_collision_cylinder_ = sdf_collision_geometry_->AddElement("cylinder");
    sdf_collision_cylinder_->GetElement("radius")->Set(std::to_string(radius) );
    sdf_collision_cylinder_->GetElement("length")->Set(std::to_string(length) );
    // Visual geometry
    sdf::ElementPtr sdf_visual_ = sdf_link_->AddElement("visual");
    sdf_visual_->GetAttribute("name")->Set("visual");

    sdf::ElementPtr sdf_visual_geometry_ = sdf_visual_->GetElement("geometry");
    sdf::ElementPtr sdf_visual_cylinder_ = sdf_visual_geometry_->AddElement("cylinder");
    sdf_visual_cylinder_->GetElement("radius")->Set(std::to_string(radius) );
    sdf_visual_cylinder_->GetElement("length")->Set(std::to_string(length) );
}