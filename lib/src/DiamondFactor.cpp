#include "../include/DiamondFactor.h"
#include "../include/DiamondConstraintsFactor.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>


int main(int argc, char *argv[])
{

    // Open the CSV file
    std::ifstream file("../../data/GT_data_and_joint_values_for_factor_graph_base_on_symforce.csv");
    std::vector<std::vector<double>> data;
    if (file) {
        std::string line;
        while (getline(file, line)) {
            std::stringstream ss(line);
            std::vector<double> row;
            std::string val;
            while (getline(ss, val, ',')) {
                row.push_back(stod(val));
            }
            data.push_back(row);
        }
        std::cout << "Number of data: " << data.size() << std::endl;
    } else {
        std::cout << "Unable to open file." << std::endl;
    }    
    // test data 
    // int i  = 1200;
    // int j = 9; // j=9 or j=0
    // std::cout << data[i][0+j] << std::endl; 
    // std::cout << data[i][1+j] << std::endl;    
    // std::cout << data[i][2+j] << std::endl;   
    // std::cout << data[i][3+j] << std::endl; 
    // std::cout << data[i][4+j] << std::endl;    
    // std::cout << data[i][5+j] << std::endl;
    // std::cout << data[i][6+j] << std::endl; 
    // std::cout << data[i][7+j] << std::endl;    
    // std::cout << data[i][8+j] << std::endl; 
    // std::cout << data[i][18] << std::endl;    
    // std::cout << data[i][19] << std::endl;    
    // std::cout << data[i][20] << std::endl; 
    // std::cout << data[i][21] << std::endl; 

    NonlinearFactorGraph graph;
    Values initial_estimate;

    auto prior_noiseModel_hand_eye = noiseModel::Diagonal::Sigmas((gtsam::Vector(3)<< 5.0 * M_PI/180.0 / 3.0, 5.0 * M_PI/180.0 / 3.0, 5.0 * M_PI/180.0 / 3.0).finished()); //
    auto PrirorFactorNoiseModel = noiseModel::Isotropic::Sigma(1, 5.0 * M_PI/180 / 3.0);  

    auto DiamondCalibrationNoise = noiseModel::Isotropic::Sigma(3, 0.5);  
    auto InequalityNoiseModel = noiseModel::Isotropic::Sigma(1, 1.0/3.0);  
    auto EqualityNoiseModel = noiseModel::Isotropic::Sigma(1, 0.001);  
    auto BoundFactorNoiseModel = noiseModel::Isotropic::Sigma(1, 1.0);  

    for (int i = 10; i <= 1000 ; i++) // data.size()-1
    {
        gtsam::Rot3 RotGT1 = { {data[i][0],  data[i][1], data[i][2]}, {data[i][3],  data[i][4], data[i][5]}, {data[i][6], data[i][7], data[i][8]} };
        gtsam::Rot3 RotGT2 = { {data[i][9],  data[i][10], data[i][11]}, {data[i][12],  data[i][13], data[i][14]}, {data[i][15], data[i][16], data[i][17]} };

        double theta11 = data[i][18];
        double theta12 = data[i][19];
        double theta21 = data[i][20];
        double theta22 = data[i][21];

        graph.add(std::make_shared<DiamondCalibrationFactor>(Symbol('o', 0), Symbol('o', 1), Symbol('x', 0), Symbol('x', 1), Symbol('r', 0), theta11, theta12, theta21, theta22, RotGT1,  RotGT2, DiamondCalibrationNoise));
    }

    graph.add(std::make_shared<InequalityPLUS>(Symbol('x', 0), Symbol('x', 1), InequalityNoiseModel)); // key second > key first
    graph.add(std::make_shared<Equality>(Symbol('x', 0), Symbol('x', 1), 90.0 * M_PI/180.0, EqualityNoiseModel));
    graph.add(std::make_shared<BoundFactor>(Symbol('x', 0), 40.0 * M_PI/180.0, 50.0 * M_PI/180.0, BoundFactorNoiseModel)); 
    graph.add(std::make_shared<BoundFactor>(Symbol('x', 1), 40.0 * M_PI/180.0, 50.0 * M_PI/180.0, BoundFactorNoiseModel)); 
    graph.add(gtsam::PriorFactor<double>(Symbol('o', 0), 180.0 * M_PI/180, PrirorFactorNoiseModel)); 
    graph.add(gtsam::PriorFactor<double>(Symbol('o', 1), 78.0 * M_PI/180, PrirorFactorNoiseModel)); 
    graph.add(gtsam::PriorFactor<gtsam::Rot3>(Symbol('r', 0), gtsam::Rot3(), prior_noiseModel_hand_eye)); 

    initial_estimate.insert(Symbol('x', 0), 45.0 * M_PI/180.0); // x(0)
    initial_estimate.insert(Symbol('x', 1), 45.0 * M_PI/180.0); // x(1)
    initial_estimate.insert(Symbol('o', 0), 180.0 * M_PI/180); // offset(0)
    initial_estimate.insert(Symbol('o', 1), 78.0 * M_PI/180); // offset(1)
    initial_estimate.insert(Symbol('r', 0), gtsam::Rot3()); // r(0)

    // graph.print();

    // Optimize using Levenberg-Marquardt optimization
    gtsam::LevenbergMarquardtParams params; 
    std::cout << std::endl;
    // params.setVerbosityLM("SUMMARY");
    LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, params);
    Values result_LM = optimizer.optimize();
    // result_LM.print();
    std::cout << "Optimization error is: " << optimizer.error() << std::endl;

    std::cout << "Alpha is: " << (result_LM.at<double>(Symbol('x', 0))) * 180/M_PI << std::endl;
    std::cout << "Beta is: " << (result_LM.at<double>(Symbol('x', 1))) * 180/M_PI << std::endl;
    std::cout << "Alpha + Beta is: " << (result_LM.at<double>(Symbol('x', 1)) + result_LM.at<double>(Symbol('x', 0))) * 180/M_PI << std::endl;
    std::cout << "Offset0 is: " << (result_LM.at<double>(Symbol('o', 0))) * 180/M_PI << std::endl;
    std::cout << "Offset1 is: " << (result_LM.at<double>(Symbol('o', 1))) * 180/M_PI << std::endl;
    gtsam::Rot3 Hand_Eye_Matrix = result_LM.at<gtsam::Rot3>(Symbol('r', 0));
    std::pair<gtsam::Unit3, double> AngleAxis = Hand_Eye_Matrix.axisAngle();
    std::cout << "Roll: " << Hand_Eye_Matrix.roll() * 180.0 / M_PI << std::endl;
    std::cout << "Pitch: " << Hand_Eye_Matrix.pitch() * 180.0 / M_PI << std::endl;
    std::cout << "Yaw: " << Hand_Eye_Matrix.yaw() * 180.0 / M_PI << std::endl;
    // std::cout << "\nHand_Eye_Matrix matrix is: " << std::endl << Hand_Eye_Matrix << std::endl;
    // std::cout << "\nAxis of rotation matrix is: " << std::endl << AngleAxis.first << std::endl;
    // std::cout << "Angle of rotation matrix is: " << AngleAxis.second * 180/M_PI << std::endl;

    return 0;
}
