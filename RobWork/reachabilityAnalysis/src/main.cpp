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
    double stepsize = 0.3; 
    // 0.06 = 190
    // 0.05 = 258
    // 0.08 = 68
    int experiment_nr;

    // Experiment:  
        // 1. Pick right
        // 2. Pick mid
        // 3. Pick left
        // 4. Place
        // 5. All

    if (temp == 1)
    {
        experiment_nr = 1;
        // file_path = "../../Experiments/stepsize_006/side/data/right.txt";
        // folder = "../../Experiments/stepsize_006/side/rwplays/right/";
        file_path = "../../Experiments/stepsize_test/side/data/1.txt";
        folder = "../../Experiments/stepsize_test/side/rwplays/1/";
        std::cout << "during 1. position" << std::endl;
    }
    else if (temp == 2)
    {
        experiment_nr = 2;
        // file_path = "../../Experiments/stepsize_006/side/data/mid.txt";
        // folder = "../../Experiments/stepsize_006/side/rwplays/mid/";
        file_path = "../../Experiments/stepsize_test/side/data/2.txt";
        folder = "../../Experiments/stepsize_test/side/rwplays/2/";
        std::cout << "during 2. position" << std::endl;
    }
    else if (temp == 3)
    {
        experiment_nr = 3;
        // file_path = "../../Experiments/stepsize_006/side/data/left.txt";
        // folder = "../../Experiments/stepsize_006/side/rwplays/left/";
        file_path = "../../Experiments/stepsize_test/side/data/3.txt";
        folder = "../../Experiments/stepsize_test/side/rwplays/3/";
        std::cout << "during 3. position" << std::endl;
    } 
    else if (temp == 4)
    {
        experiment_nr = 4;
        // file_path = "../../Experiments/stepsize_006/side/data/left.txt";
        // folder = "../../Experiments/stepsize_006/side/rwplays/left/";
        file_path = "../../Experiments/stepsize_test/side/data/4.txt";
        folder = "../../Experiments/stepsize_test/side/rwplays/4/";
        std::cout << "during 3. position" << std::endl;
    } 
    else if (temp == 5)
    {
        experiment_nr = 5;
        // file_path = "../../Experiments/stepsize_006/side/data/left.txt";
        // folder = "../../Experiments/stepsize_006/side/rwplays/left/";
        file_path = "../../Experiments/stepsize_test/side/data/5.txt";
        folder = "../../Experiments/stepsize_test/side/rwplays/5/";
        std::cout << "during 3. position" << std::endl;
    } 
    else if (temp == 6)
    {
        experiment_nr = 6;
        // file_path = "../../Experiments/stepsize_006/side/data/left.txt";
        // folder = "../../Experiments/stepsize_006/side/rwplays/left/";
        file_path = "../../Experiments/stepsize_test/side/data/6.txt";
        folder = "../../Experiments/stepsize_test/side/rwplays/6/";
        std::cout << "during 3. position" << std::endl;
    } 
    else if (temp == 7)
    {
        experiment_nr = 7;
        // file_path = "../../Experiments/stepsize_006/side/data/place.txt";
        // folder = "../../Experiments/stepsize_006/side/rwplays/place/";
        file_path = "../../Experiments/stepsize_test/side/data/7.txt";
        folder = "../../Experiments/stepsize_test/side/rwplays/7/";
        std::cout << "during 4. position" << std::endl;
    } else {
        return 0;
    }
    

    if (temp <= 7){

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
    //const std::string cylinder_name = "Cyl";
    rw::kinematics::MovableFrame::Ptr cylinder_frame = wc->findFrame<rw::kinematics::MovableFrame>(cylinder_name);
    if (NULL == cylinder_frame) {
        RW_THROW("Could not find movable frame " + cylinder_name + " ... check model");
        return -1;
    }

    // // find table referece frame
    // const std::string table_name = "Table";
    // rw::core::Ptr<Frame> table_refframe = wc->findFrame<rw::core::Ptr<Frame>>(table_name);
    // if (NULL == table_refframe) {
    //     RW_THROW("Could not find movable frame " + table_name + " ... check model");
    //     return -1;
    // }
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
    //return 0;
    // cylinder_pos = rw::math::Vector3D<>(std::stod("-0.36 0.56 0.15"),
                                            // std::stod("0.0 0.56 0.15"),
                                            // std::stod("0.36 0.56 0.15"));
        // cylinder_pos = rw::math::Vector3D<>(
        //                     std::stod("-0.36"),
        //                     std::stod("0.56"),
        //                     std::stod("0.15"));
        // cylinder_pos = rw::math::Vector3D<>(
        //                     std::stod("0.0"),
        //                     std::stod("0.56 "),
        //                     std::stod("0.15"));
        // cylinder_pos = rw::math::Vector3D<>(
        //                     std::stod("0.36"),
        //                     std::stod("0.56"),
        //                     std::stod("0.15"));
        // cylinder_pos = rw::math::Vector3D<>(
        //                     std::stod("0.3"),
        //                     std::stod("-0.5"),
        //                     std::stod("0.15"));

    switch (experiment_nr){
    case 1:
        //-1,272. 2,405. 0,550
        // cylinder_pos = rw::math::Vector3D<>(
        //            std::stod("-0.34"),
        //            std::stod("0.54"),
        //            std::stod("0.15"));
        // cylinder_pos = rw::math::Vector3D<>(-0.34, 0.54, 0.25);
        // cylinder_positions = {rw::math::Vector3D<>(-0.34, 0.54, 0.25)};
        cylinder_pos = rw::math::Vector3D<>(-0.46, -1.1, 0.2 );
        cylinder_positions = {rw::math::Vector3D<>(-0.46, -1.1, 0.2)};
        break;
    case 2:
        // cylinder_pos = rw::math::Vector3D<>(
        //            std::stod("0.0"),
        //            std::stod("0.54 "),
        //            std::stod("0.15"));
        // cylinder_pos = rw::math::Vector3D<>(0.0, 0.54, 0.15);
        // cylinder_positions = {rw::math::Vector3D<>(0.0, 0.54, 0.15)};
        cylinder_pos = rw::math::Vector3D<>(-0.30, -1.1, 0.2 );
        cylinder_positions = {rw::math::Vector3D<>(-0.30, -1.1, 0.2)};
        break;
    case 3:
        // cylinder_pos = rw::math::Vector3D<>(
        //            std::stod("0.34"),
        //            std::stod("0.54"),
        //            std::stod("0.15"));
        // cylinder_pos = rw::math::Vector3D<>(0.34, 0.54, 0.15);
        // cylinder_positions = {rw::math::Vector3D<>(0.34, 0.54, 0.15)};
        cylinder_pos = rw::math::Vector3D<>(-0.150, -1.1, 0.2);
        cylinder_positions = {rw::math::Vector3D<>(-0.150, -1.1, 0.2)};
        break;
    case 4:
        // cylinder_pos = rw::math::Vector3D<>(
        //                    std::stod("0.3"),
        //                    std::stod("-0.5"),
        //                    std::stod("0.15"));
        // cylinder_pos = rw::math::Vector3D<>(0.3, -0.5, 0.15);
        // cylinder_positions = {rw::math::Vector3D<>(0.3, -0.5, 0.15)};
        cylinder_pos = rw::math::Vector3D<>(0.025, -1.1, 0.2);
        cylinder_positions = {rw::math::Vector3D<>(0.025, -1.1, 0.2)};
        break;
    case 5:
        // cylinder_pos = rw::math::Vector3D<>(
        //                    std::stod("0.3"),
        //                    std::stod("-0.5"),
        //                    std::stod("0.15"));
        // cylinder_pos = rw::math::Vector3D<>(0.3, -0.5, 0.15);
        // cylinder_positions = {rw::math::Vector3D<>(0.3, -0.5, 0.15)};
        cylinder_pos = rw::math::Vector3D<>(0.175, -1.1, 0.2);
        cylinder_positions = {rw::math::Vector3D<>(0.175, -1.1, 0.2)};
        break;
    case 6:
        // cylinder_pos = rw::math::Vector3D<>(
        //                    std::stod("0.3"),
        //                    std::stod("-0.5"),
        //                    std::stod("0.15"));
        // cylinder_pos = rw::math::Vector3D<>(0.3, -0.5, 0.15);
        // cylinder_positions = {rw::math::Vector3D<>(0.3, -0.5, 0.15)};
        cylinder_pos = rw::math::Vector3D<>(0.35, -1.1, 0.2);
        cylinder_positions = {rw::math::Vector3D<>(0.35, -1.1, 0.2)};
        break;
    case 7:
        // cylinder_pos = rw::math::Vector3D<>(
        //                    std::stod("0.3"),
        //                    std::stod("-0.5"),
        //                    std::stod("0.15"));
        // cylinder_pos = rw::math::Vector3D<>(0.3, -0.5, 0.15);
        // cylinder_positions = {rw::math::Vector3D<>(0.3, -0.5, 0.15)};
        cylinder_pos = rw::math::Vector3D<>(0.50, -1.1, 0.2);
        cylinder_positions = {rw::math::Vector3D<>(0.50, -1.1, 0.2)};
        break;
    default:
        std::cout << "Error here!" << std::endl;
        break;
    }


//    // create vector with cylinder positions
//    std::vector<rw::math::Vector3D<>> cylinder_positions = {
    // Original
    //    rw::math::Vector3D<>(-0.25, 0.474, 0.15),
    //    rw::math::Vector3D<>(0.25, 0.474, 0.15),
    //    rw::math::Vector3D<>(0.3, -0.5, 0.15)
    //Experiment pos
    //    rw::math::Vector3D<>(-0.36, 0.56, 0.15),
    //    rw::math::Vector3D<>(0.0, 0.56, 0.15),
    //    rw::math::Vector3D<>(0.36, 0.56, 0.15),
    //    rw::math::Vector3D<>(0.3, -0.5, 0.15)
    // Moved
    //    rw::math::Vector3D<>(-0.36, 0.56, 0.15),
    //    rw::math::Vector3D<>(0.0, 0.56, 0.15),
    //    rw::math::Vector3D<>(0.36, 0.56, 0.15),
    //    rw::math::Vector3D<>(0.3, -0.5, 0.15)
 //  };

    // check for every base position the collision free solution to each cylinder position
    std::vector<unsigned int> number_of_solutions;
    std::vector<float> x_positions, y_positions;
    std::string rwplay = "";
    std::vector<rw::math::Q> collision_free_solutions;

    //rw::math::RPY<> rot(0, 0, 90*rw::math::Deg2Rad); // INSERTED NOW EMIL; DELETE IF NOT WORKING
    // Assuming cylinder_frame and state are correctly set
    // rw::math::RPY<> initialRot(0, 0, 180*rw::math::Deg2Rad); // Laying down the cylinder
    // rw::math::Transform3D<> cylinder_trans(cylinder_positions[0], initialRot);
    // //rw::math::Transform3D<> initialTrans(cylinder_frame->getTransform(state).P(), initialRot);
    // cylinder_frame->moveTo(cylinder_trans, table_frame, state);

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


           // get collision free solutions
           std::vector<rw::math::Q> collision_free_solutions = reach.getCollisionFreeSolutions(wc, robot_ur5, cylinder_frame, "GraspTarget", rwplay, state);

           // store total number of solutions
           solutions += collision_free_solutions.size();
       }

        // generate rwplay file
        //const std::string folder = "../../rwplays/";//"/home/reventlov/RobCand/RoVi_Project/reachabilityAnalysis/rwplays/";//"../../rwplays";
        //rwplay = folder + "_position" + std::to_string(i) + "_cylinder_Experiment_" + ".rwplay";

        // move cylinder
        // rw::math::Transform3D<> cylinder_trans(cylinder_pos, cylinder_rot);
        // cylinder_frame->moveTo(cylinder_trans, state);

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

    if (temp > 7){ // FULL experiment is done with temp > 4
        break;
    }
}
return 0;

}