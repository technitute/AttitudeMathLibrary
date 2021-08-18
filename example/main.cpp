// ######################################################################### //
// Attitude Math Library
// Copyright (c) 2021 Dr. Steven Dumble Phd
// Distributed under the MIT Software License (See accompanying file LICENSE)
// ######################################################################### //

#include <AML.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

using namespace AML;

int main(int argc, char** argv)
{

    // Open Output File
    std::ofstream outputFile;
    outputFile.open("AttitudeResults.csv");
    if (!outputFile.is_open())
    {
        std::cout << "Error: Unable to Open Output File." << std::endl;
    }
    else
    {
        // Write Header
        outputFile << "Time (sec),"
                   << "Roll_DCM (deg),"
                   << "Pitch_DCM (deg),"
                   << "Yaw_DCM (deg),"
                   << "Roll_Euler (deg),"
                   << "Pitch_Euler (deg),"
                   << "Yaw_Euler (deg),"
                   << "Roll_Quat (deg),"
                   << "Pitch_Quat (deg),"
                   << "Yaw_Quat (deg)" << std::endl;
    }

    double dt = 0.001;                             // Forward Euler First Order Step Size (sec)
    double endTime = 20.0;                         // Integration End Time (sec)
    EulerAngles attitude0 = EulerAngles(0, 0, 0);  // Initial Conditions at Time 0
    Vector3 bodyAngularRates = { 0.2, -0.5, 0.4 }; // Instantaneous Body Angular Rates (rad/s)

    Matrix33 attitudeDCM = eulerAngles2DCM(attitude0);
    EulerAngles attitudeEuler = attitude0;
    Quaternion attitudeQuat = eulerAngles2Quat(attitude0);

    double time = 0.0;
    while (time < endTime)
    {
        // Step Time
        time += dt;

        // Integrate Attitude using First Order Euler Intergration
        attitudeDCM = integrateDCM(attitudeDCM, dcmKinematicRates_BodyRates(attitudeDCM, bodyAngularRates), dt);
        attitudeEuler =
            integrateEulerAngles(attitudeEuler, eulerAngleKinematicRates(attitudeEuler, bodyAngularRates), dt);
        attitudeQuat = integrateQuat(attitudeQuat, quatKinematicRates_BodyRates(attitudeQuat, bodyAngularRates), dt);

        // Convert Attitudes into Euler Angles (XYZ Order) for Output
        EulerAngles attitudeDCM_Euler = dcm2EulerAngles(attitudeDCM);
        EulerAngles attitudeEuler_Euler = attitudeEuler;
        EulerAngles attitudeQuat_Euler = quat2EulerAngles(attitudeQuat);

        // Save Timestep Data
        if (outputFile.is_open())
        {
            outputFile << time << "," << attitudeDCM_Euler.phi * (180.0 / M_PI) << ","
                       << attitudeDCM_Euler.theta * (180.0 / M_PI) << "," << attitudeDCM_Euler.psi * (180.0 / M_PI)
                       << "," << attitudeEuler_Euler.phi * (180.0 / M_PI) << ","
                       << attitudeEuler_Euler.theta * (180.0 / M_PI) << "," << attitudeEuler_Euler.psi * (180.0 / M_PI)
                       << "," << attitudeQuat_Euler.phi * (180.0 / M_PI) << ","
                       << attitudeQuat_Euler.theta * (180.0 / M_PI) << "," << attitudeQuat_Euler.psi * (180.0 / M_PI)
                       << std::endl;
        }
    }

    // Close File
    outputFile.close();

    return 0;
}
