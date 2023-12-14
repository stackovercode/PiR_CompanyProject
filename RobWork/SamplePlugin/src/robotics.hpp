#ifndef ROBOTICS_HPP
#define ROBOTICS_HPP

#include <rw/rw.hpp>
#include <iostream>
#include <vector>
#include <map>
#include <fstream>
#include <string>
#include <utility> // std::pair
#include <cstdio>
#include <cstdlib>
#include <rw/core/Ptr.hpp>
#include <Eigen/Dense>
#include <rw/math/Transform3D.hpp>
#include <Eigen/Geometry>

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rw/invkin.hpp>
#include <rw/math/EAA.hpp>

#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>

// Additional:
#include <functional>
#include <vector>
#include <rw/math.hpp>


class Robotics{
    
public:
    Robotics();

    double constantVelocity(double t, double t0, double t1);

    //void perform_test(std::vector<rw::math::Q> _q, std::vector<double> _t, std::string f_name, bool blend);

    std::pair<std::pair<Eigen::Vector3d, Eigen::Matrix3d>,Eigen::Matrix4d> trans2Eigen(rw::math::Transform3D<> P);

    rw::trajectory::TimedStatePath linInterp (rw::models::SerialDevice::Ptr robot_ur5,
                                            rw::kinematics::State state, std::vector<rw::math::Q> path, double duration);


    std::vector<rw::math::Q> findConfigurations(const std::string nameGoal, 
                                            const std::string nameTcp, 
                                            rw::models::SerialDevice::Ptr robot_ur5, 
                                            rw::models::WorkCell::Ptr wc, 
                                            rw::kinematics::State state);

    rw::math::Q findCollisionFreeSolution(rw::models::SerialDevice::Ptr robot_ur5, 
                                        rw::kinematics::State state, 
                                        rw::proximity::CollisionDetector& detector, 
                                        std::vector<rw::math::Q> solutions);

    rw::math::Q findCollisionFreeSolution(rw::models::SerialDevice::Ptr robot_ur5, 
                                        rw::kinematics::State state, 
                                        rw::proximity::CollisionDetector& detector, 
                                        std::vector<rw::math::Q> solutions, 
                                        rw::math::Q prevSolution);

    std::vector<rw::math::Transform3D<>> pointOrder(rw::math::Transform3D<> pickFrame, 
                                                            rw::math::Transform3D<> homeFrame, 
                                                            rw::math::Transform3D<> placeFrame, 
                                                            rw::math::Transform3D<> nearPickFrame, 
                                                            rw::math::Transform3D<> nearPlaceFrame); 

    std::pair<std::vector<rw::math::Q>,std::vector<rw::math::Transform3D<>>> P2P(std::vector<rw::math::Transform3D<>> P, 
                                                                                            std::vector<float> T, 
                                                                                            rw::kinematics::MovableFrame *targetFrame, 
                                                                                            rw::models::SerialDevice::Ptr robot_ur5, 
                                                                                            rw::models::WorkCell::Ptr wc, 
                                                                                            rw::kinematics::State state, 
                                                                                            rw::proximity::CollisionDetector& detector);
                                                                                            

    rw::math::Transform3D<double> convertToTransform3D(const Eigen::Vector3d translation, const Eigen::Quaterniond rotation);


    void create_rwplay(std::vector<rw::math::Q> jointPath, 
                        rw::models::SerialDevice::Ptr robot_ur5,
                        rw::models::WorkCell::Ptr wc, 
                        rw::kinematics::State state, 
                        const std::string rwplayPath);

    void create_data(std::vector<rw::math::Q> joints, 
                                const std::string jointPath, 
                                std::vector<rw::math::Transform3D<>> cartesian, 
                                const std::string cartPath);



    std::string WC_FILE = "../../../WorkCell/Scene.wc.xml";

private:

 
};

#endif // ROBOTICS_HPP