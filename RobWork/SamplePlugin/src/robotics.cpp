#include "robotics.hpp"

Robotics::Robotics(){}

double Robotics::constantVelocity(double t, double t0, double t1) {
    return (t-t0)/(t1-t0);
}


// Kaspers solution from Robotics Exercise 4
rw::trajectory::TimedStatePath Robotics::linInterp (rw::models::SerialDevice::Ptr robot_ur5,
                                            rw::kinematics::State state, std::vector<rw::math::Q> path, double duration){

    rw::trajectory::TimedStatePath res;

    for (size_t i = 0; i < path.size(); i++){
    
       // if ( path.size() > 0 )
        rw::trajectory::LinearInterpolator<rw::math::Q> interp (path[i], path[i], duration);
       // else
         //   interp (path[i-1], path[i], duration);

        for (double i = 0; i < duration; i += 0.05) {
            robot_ur5->setQ (interp.x (i), state);
            res.push_back (rw::trajectory::TimedState (i, state));
        }
    }
    return res;

}

std::vector<rw::math::Q> Robotics::findConfigurations(const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot_ur5, rw::models::WorkCell::Ptr wc, rw::kinematics::State state){
    // Get, make and print name of frames
    const std::string robotName = robot_ur5->getName();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp = robotName + "." + "TCP";

    // Find frames and check for existence
    rw::kinematics::Frame* frameGoal = wc->findFrame(nameGoal);
    rw::kinematics::Frame* frameTcp = wc->findFrame(nameTcp);
    rw::kinematics::Frame* frameRobotBase = wc->findFrame(nameRobotBase);
    rw::kinematics::Frame* frameRobotTcp = wc->findFrame(nameRobotTcp);
    if(frameGoal==nullptr || frameTcp==nullptr || frameRobotBase==nullptr || frameRobotTcp==nullptr)
    {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (frameGoal==nullptr ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (frameTcp==nullptr ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotBase << "\": " << (frameRobotBase==nullptr ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp==nullptr ? "NO!" : "YES!") << std::endl;
    }

    // Make "helper" transformations
    rw::math::Transform3D<> frameBaseTGoal = rw::kinematics::Kinematics::frameTframe(frameRobotBase, frameGoal, state);
    rw::math::Transform3D<> frameTcpTRobotTcp = rw::kinematics::Kinematics::frameTframe(frameTcp, frameRobotTcp, state);

    // get grasp frame in robot tool frame
    rw::math::Transform3D<> targetAt = frameBaseTGoal * frameTcpTRobotTcp;

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(robot_ur5, state) );
    return closedFormSovler->solve(targetAt, state);
}

rw::math::Q Robotics::findCollisionFreeSolution(rw::models::SerialDevice::Ptr robot_ur5, rw::kinematics::State state, rw::proximity::CollisionDetector& detector, std::vector<rw::math::Q> solutions){
    rw::math::Q configuration;
    for ( unsigned int i = 0; i < solutions.size(); i++ )
    {
        robot_ur5->setQ(solutions[i], state);
        if ( !detector.inCollision(state, NULL, true) )
        {
            configuration = solutions[i];
            break;
        }
    }
    return configuration;
}

rw::math::Q Robotics::findCollisionFreeSolution(rw::models::SerialDevice::Ptr robot_ur5, rw::kinematics::State state, rw::proximity::CollisionDetector& detector, std::vector<rw::math::Q> solutions, rw::math::Q prevSolution){
    rw::math::Q configuration;
    double dist = std::numeric_limits<double>::max();

    for ( unsigned int i = 0; i < solutions.size(); i++ )
    {
        robot_ur5->setQ(solutions[i], state);
        if ( !detector.inCollision(state, NULL, true) )
        {
            double cdist = rw::math::Q(prevSolution - solutions[i]).norm2();
            if ( cdist < dist )
            {
                configuration = solutions[i];
                dist = cdist;
            }
        }
    }
    return configuration;
}

std::vector<rw::math::Transform3D<>> Robotics::pointOrder(rw::math::Transform3D<> pickFrame, rw::math::Transform3D<> homeFrame, rw::math::Transform3D<> placeFrame, rw::math::Transform3D<> nearPickFrame, rw::math::Transform3D<> nearPlaceFrame){   
    // Test Q poins.
    // rw::math::Q Q1_homePos (1.571 -1 -1.7 -2 1.571 0);
    // rw::math::Q Q2_nearPick (2.848, -1.203, -1.558, -1.563, 1.571, 0); 
    // rw::math::Q Q3_nearPlace (-1.67447, -1.59122, -1.42628, -1.56299, 1.57101, 1.55038); 

    //0. point: Home
    rw::math::Transform3D<> Point0 = homeFrame; 
    // 1. point: Near pick
    rw::math::Transform3D<> Point1 = nearPickFrame;
    // 2. point: Above pick
    rw::math::Transform3D<> Point2 = pickFrame*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.25));
    // 3. point: pick
    rw::math::Transform3D<> Point3 = pickFrame;
    // 4. point: above pick
    rw::math::Transform3D<> Point4 = pickFrame*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.25));
    // 5. point: Near pick
    rw::math::Transform3D<> Point5 = nearPickFrame;
    // 6. point: Near place
    rw::math::Transform3D<> Point6 = nearPlaceFrame;
    // 7. point: Above place
    rw::math::Transform3D<> Point7 = placeFrame*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.25));
    // 8. point: place
    rw::math::Transform3D<> Point8 = placeFrame;
    // 9. point: Above place
    rw::math::Transform3D<> Point9 = placeFrame*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.25));
    // 10. point: home
    rw::math::Transform3D<> Point10 = homeFrame;

    std::vector<rw::math::Transform3D<>> path = {Point0, Point1, Point2, Point3, Point4, Point5, Point6, Point7, Point8, Point9, Point10};
    return path;
}


rw::math::Transform3D<double> Robotics::convertToTransform3D(const Eigen::Vector3d translation, const Eigen::Quaterniond rotation){
  // Create an Eigen::Isometry3d with the translation and rotation
  Eigen::Isometry3d iso = Eigen::Translation3d(translation) * rotation;

  // Convert the Eigen::Isometry3d to a rw::math::Transform3D
  return rw::math::Transform3D<double>(iso.matrix());
}

// rw::math::Transform3D<double> Robotics::convertTo(const Eigen::Vector3d translation, const Eigen::Quaterniond rotation){
//   // Create an Eigen::Isometry3d with the translation and rotation
//   Eigen::Isometry3d iso = Eigen::Translation3d(translation) * rotation;

//   // Convert the Eigen::Isometry3d to a rw::math::Transform3D
//   return rw::math::Transform3D<double>(iso.matrix());
// }


/*Linear interpolation derived from Robotics lecture 4 slides 12-15*/
/*Method from Robotics lecture notes and reconstructed for using quaternions*/
std::pair<std::vector<rw::math::Q>,std::vector<rw::math::Transform3D<>>> Robotics::P2P(std::vector<rw::math::Transform3D<>> P, std::vector<float> T, rw::kinematics::MovableFrame* targetFrame, rw::models::SerialDevice::Ptr robot_ur5, rw::models::WorkCell::Ptr wc, rw::kinematics::State state, rw::proximity::CollisionDetector& detector){ 
    std::vector<rw::math::Q> path;
    std::vector<rw::math::Transform3D<>> point;
    for ( unsigned int i = 1; i < P.size(); i++ ){
        for ( float t = T[i-1]; t < T[i]; t += 0.01f ){

            /* This is the code for calucalate using EEA.
            // rw::math::Vector3D<> Pi = P[i-1].P() + (P[i].P() - P[i-1].P())*(t - T[i-1])/(T[i] - T[i-1]);
            // Eigen::AngleAxisd axis_angle(P[i].R().e()*(P[i].R().e()).inverse());
            // rw::math::EAA<> eea2Mat(Eigen::Vector3d(axis_angle.axis()*axis_angle.angle() * (t - T[i-1])/(T[i]-T[i-1])));
            // point.push_back(rw::math::Transform3D<>( Pi, eea2Mat.toRotation3D()*P[i-1].R()));
            */

            // This is the code for calucalate using quaternions.
            //Eigen::Vector3d Pi = P[i-1].P() + (P[i].P() - P[i-1].P())*(t - T[i-1])/(T[i] - T[i-1]);
            Eigen::Vector3d Pi = P[i-1].P() + (P[i].P() - P[i-1].P())*constantVelocity(t,T[i-1],T[i]);
            Eigen::Quaterniond q1(P[i-1].R().e()), q2(P[i].R().e());            
            //Eigen::Quaterniond q = q1.slerp(t - T[i-1], q2);
            Eigen::Quaterniond q = q1.slerp(constantVelocity(t,T[i-1],T[i]), q2);
            point.push_back(convertToTransform3D(Pi,q));

            targetFrame->moveTo(point.back(), state);

            std::vector<rw::math::Q> solutions = Robotics::findConfigurations("GraspTarget", "GraspTCP", robot_ur5, wc, state);

            rw::math::Q configuration;
            if ( path.size() > 0 )
                configuration = Robotics::findCollisionFreeSolution(robot_ur5, state, detector, solutions, path.back());
            else
                configuration = Robotics::findCollisionFreeSolution(robot_ur5, state, detector, solutions);

            path.push_back(configuration);
        }
    }
    return std::make_pair(path, point);
}

std::pair<std::pair<Eigen::Vector3d, Eigen::Matrix3d>,Eigen::Matrix4d> Robotics::trans2Eigen(rw::math::Transform3D<> P){
    Eigen::Matrix3d R = P.R().e();
    Eigen::Vector3d T = P.P();
    Eigen::Matrix4d Trans; 
    Trans.setIdentity(); 
    Trans.block<3,3>(0,0) = R;
    Trans.block<3,1>(0,3) = T;
    return std::make_pair(std::make_pair(T,R),Trans);
}

void Robotics::create_rwplay(std::vector<rw::math::Q> jointPath, rw::models::SerialDevice::Ptr robot_ur5, rw::models::WorkCell::Ptr wc, rw::kinematics::State state, const std::string rwplayPath){
    rw::trajectory::TimedStatePath statePath;
    double time = 0;
    double dur = 0.1;
    for ( rw::math::Q configuration : jointPath ) {//jointPath
        robot_ur5->setQ(configuration, state);
        statePath.push_back(rw::trajectory::TimedState(time, state));
        time += dur/double(1);
    }
    rw::loaders::PathLoader::storeTimedStatePath(*wc, statePath, rwplayPath);
}

void Robotics::create_data(std::vector<rw::math::Q> joints, const std::string jointPath, std::vector<rw::math::Transform3D<>> cartesian, const std::string cartPath){
    std::ofstream tfFile, qFile;
    qFile.open(jointPath);

    for ( rw::math::Q jointQ : joints ){
        qFile << jointQ[0] << " " << jointQ[1] << " " << jointQ[2] << " " << jointQ[3] << " " << jointQ[4] << " " << jointQ[5] << std::endl;
    }
    qFile.close();

    tfFile.open(cartPath);
    for ( rw::math::Transform3D<> tf : cartesian ){
        tfFile << tf.R().getRow(0)[0] << " " << tf.R().getRow(0)[1] << " " << tf.R().getRow(0)[2] << " " << tf.P()[0] << std::endl;
        tfFile << tf.R().getRow(1)[0] << " " << tf.R().getRow(1)[1] << " " << tf.R().getRow(1)[2] << " " << tf.P()[1] << std::endl;
        tfFile << tf.R().getRow(2)[0] << " " << tf.R().getRow(2)[1] << " " << tf.R().getRow(2)[2] << " " << tf.P()[2] << std::endl;
        tfFile <<          0          << " " <<          0          << " " <<          0          << " " <<      1    << std::endl;
    }
    tfFile.close();
}

// void Robotics::perform_test(std::vector<rw::math::Q> _q, std::vector<double> _t, std::string f_name, bool blend) {
//     std::ofstream outfile;
//     outfile.open(f_name);
//     std::map<int, rw::math::Q> path;

//     for ( int k = 0; k < 50; k++) {

//         rw::common::Timer t;
//         t.resetAndResume();
//         path = interpolator.q_interpolation_nb(_q, _t);
//         t.pause();
//         outfile << t.getTime() << ",";
//         std::cout << "Planning time: " << t.getTime() << std::endl;


//         rw::math::EuclideanMetric<rw::math::Q> metric2;
//         double distance = 0;

//         for (size_t i = 0; i < path.size()-1; i++) {
//             distance += metric2.distance(path.find(int(i+1))->second, path.find(int(i))->second);
//         }
//         outfile << distance << std::endl;
//         std::cout << "Distance: " << distance << std::endl;
//     }

//     outfile.close();
// }