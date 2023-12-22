#include "reachability.hpp"
#include <bits/stdc++.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

USE_ROBWORK_NAMESPACE
using namespace robwork;

int main(int argc, char *argv[]) {


int temp = 1;
while (true){
    Reachability reach;
    rw::math::Vector3D<> cylinder_pos;
    std::vector<rw::math::Vector3D<>> cylinder_positions;
    std::string folder;
    std::string file_path;
    double stepsize = 0.15; 
    // 0.06 = 210
    int experiment_nr;

    // Experiment:  
        // 1. place Left
        // 2. Place Right


    if (temp == 1)
    {
        experiment_nr = 1;
        file_path = "../../Experiments/stepsize_test/side/data/left.txt";
        folder = "../../Experiments/stepsize_test/side/rwplays/left/";
        std::cout << "during 1. position" << std::endl;
    }
    else if (temp == 2)
    {
        experiment_nr = 2;
        file_path = "../../Experiments/stepsize_test/side/data/right.txt";
        folder = "../../Experiments/stepsize_test/side/rwplays/right/";
        std::cout << "during 2. position" << std::endl;
    } else {
        return 0;
    }
    

    if (temp <= 1){

    // load workcell;
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(reach.WC_FILE);
    if (NULL == wc) {
        RW_THROW("COULD NOT LOAD WORKCELL ... check path!");
        return -1;
    }

    // find device
    const std::string device_name = "UR10e";
    rw::models::SerialDevice::Ptr robot_ur5 = wc->findDevice<rw::models::SerialDevice>(device_name);
    if (NULL == robot_ur5) {
        RW_THROW("Could not load " + device_name + " ... check model");
        return -1;
    }

    // find ur5 robot base frame
    const std::string base_name = "URReference";
    rw::kinematics::MovableFrame::Ptr base_frame = wc->findFrame<rw::kinematics::MovableFrame>(base_name);
    if (NULL == base_frame) {
        RW_THROW("Could not load " + base_name + " ... check model");
        return -1;
    }

    // find cylinder
    const std::string cylinder_name = "Cylinder";
    rw::kinematics::MovableFrame::Ptr cylinder_frame = wc->findFrame<rw::kinematics::MovableFrame>(cylinder_name);
    if (NULL == cylinder_frame) {
        RW_THROW("Could not find movable frame " + cylinder_name + " ... check model");
        return -1;
    }

    Frame* table_frame = wc->findFrame ("Table");


    // get start state and default rotation
    rw::kinematics::State state = wc->getDefaultState();
    rw::math::Rotation3D<> base_rot = base_frame->getTransform(state).R();

    rw::math::Rotation3D<> cylinder_rot = cylinder_frame->getTransform(state).R(); 


    // Generate position for reachability analysis y=0.3,-0.6 and x=-0.4,0.4
    std::vector<rw::math::Vector3D<>> base_positions;
    for (double y = 0.3; y >= -0.6; y -= stepsize) { 
        for (double x = -0.4; x <= 0.4; x += stepsize) {
            // // check for goal position
            // if (y < -0.3 && x > 0.1){
            //      continue; 
            //     }
            rw::math::Vector3D<> pos(x, y, 0.0);
            base_positions.push_back(pos);
        }
    }

    std::cout << "Number of possible base positions ---> " << base_positions.size() << std::endl;


    switch (experiment_nr){
    case 1:
        cylinder_pos = rw::math::Vector3D<>( 0.7, 0, 0.225);
        cylinder_positions = {rw::math::Vector3D<>( 0.7, 0, 0.225)};
        break;
    case 2:
        cylinder_pos = rw::math::Vector3D<>(-0.30, -1.1, 0.2 );
        cylinder_positions = {rw::math::Vector3D<>(-0.30, -1.1, 0.2)};
        break;
    default:
        std::cout << "Error here!" << std::endl;
        break;
    }


    // check for every base position the collision free solution to each cylinder position
    std::vector<unsigned int> number_of_solutions;
    std::vector<float> x_positions, y_positions;
    std::string rwplay = "";
    std::vector<rw::math::Q> collision_free_solutions;

    
    rw::math::Transform3D<> cylinder_trans(cylinder_positions[0], cylinder_rot);
    cylinder_frame->moveTo(cylinder_trans, table_frame, state);


    for (unsigned int i = 0; i < base_positions.size(); i++) {
        // move base frame
        rw::math::Transform3D<> base_trans(base_positions[i], base_rot);
        base_frame->moveTo(base_trans, state);


        // get collision free solutions
        unsigned int solutions = 0;
       for (unsigned int j = 0; j < cylinder_positions.size(); j++) {
           // generate rwplay file
           //const std::string folder = "../../rwplays/";//"/home/reventlov/RobCand/RoVi_Project/reachabilityAnalysis/rwplays/";//"../../rwplays/";
           rwplay = folder + "_position" + std::to_string(i) + "_cylinder" + std::to_string(j+1) + ".rwplay";
                    
           // move cylinder
           //rw::math::Transform3D<> cylinder_trans(cylinder_positions[j], cylinder_rot);
           //cylinder_frame->moveTo(cylinder_trans, state);

            std::vector<rw::math::Q> collision_free_solutions = reach.getCollisionFreeSolutions(wc, robot_ur5, cylinder_frame, "GraspTarget", rwplay, state);

           // store total number of solutions
           solutions += collision_free_solutions.size();
       }

        // generate rwplay file
        // const std::string folder = "../../rwplays/";//"/home/reventlov/RobCand/RoVi_Project/reachabilityAnalysis/rwplays/";//"../../rwplays";
        // rwplay = folder + "_position" + std::to_string(i) + "_cylinder_Experiment_" + ".rwplay";
        // std::cout << "Cylinder position before moving: " << cylinder_pos << std::endl;

        // // move cylinder
        // rw::math::Transform3D<> cylinder_trans(cylinder_pos, cylinder_rot);
        // cylinder_frame->moveTo(cylinder_trans, state);
        // // Move cylinder
        // std::cout << "Cylinder position after moving: " << cylinder_frame->getTransform(state).P() << std::endl;
        // get collision free solutions
        //collision_free_solutions = reach.getCollisionFreeSolutions(wc, robot_ur5, cylinder_frame, "GraspTarget", rwplay, state);

        // store total number of solutions
        //solutions += collision_free_solutions.size();

        // save data
        x_positions.push_back(base_positions[i](0));
        y_positions.push_back(base_positions[i](1));
        number_of_solutions.push_back(solutions);

        // show process
        if (i % 2 == 0) { 
            double full = (double)i/base_positions.size()*15;
            int prog = (double)i/base_positions.size()*100;
		    reach.showProgres(full, prog);
        }

    }
    //std::cout << base_positions.size() << " / " << base_positions.size() << std::endl;
    double full = (double)base_positions.size()/base_positions.size()*15;
    int prog = 100;
    reach.showProgres(full, prog);
    std::cout << base_positions.size() << " / " << base_positions.size() << std::endl;

    // save all data to file_path
    reach.saveConfirgurations(x_positions, y_positions, number_of_solutions, file_path);
    temp++;
    } 

    if (temp > 1){
        break;
    }
}
return 0;

}