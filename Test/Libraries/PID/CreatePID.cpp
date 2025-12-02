#include "../../../Libraries/Control/PID.h"
#include "../../../Libraries/Control/Plant.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <filesystem>

int main()
{
    // 1. Create a PID Controller object
    Libraries::Control::PID<double> pidController;
    pidController.kp = 0.85; // Proportional gain
    pidController.ki = 0.15; // Integral gain
    pidController.kd = 0.3; // Derivative gain
    pidController.setOutputLimits(-5.0, 5.0);
    // 2. Create a Plant/Process transfer function object
    // Define the transfer function coefficients.  For example:
    // H(z) = (1 + 0.5z^-1) / (1 - 0.8z^-1 + 0.3z^-2)
    std::vector<double> numerator_coeffs = {1.0, 0.5};         // b0, b1
    std::vector<double> denominator_coeffs = {1.0, -0.8, 0.3}; // a0, a1, a2
    Libraries::Control::Plant<double> plant(numerator_coeffs, denominator_coeffs);

    // 3. Define the control loop parameters
    double startTime = 0.0;
    double endTime = 100.0;
    double sampleTime = 1;
    int numSteps = static_cast<int>((endTime - startTime) / sampleTime) + 1;

    // 4. Define the reference signal
    std::vector<double> referenceSignal(numSteps);

    for (int i = 0; i < numSteps; ++i)
    {
        double currentTime = startTime + i * sampleTime;
        if (i < 10 / sampleTime)
        {
            referenceSignal[i] = 0;
        }
        else
        {
            if (i < 40)
            {
                referenceSignal[i] = 3.5;
            }
            else
            {
                if (i < 50)
                {
                    referenceSignal[i] = 0.0;
                }
                else
                {
                    referenceSignal[i] = -3.5;
                }
            }
        }
    }

    // 5. Simulate the feedback loop and store the results
    std::vector<double> time(numSteps);
    std::vector<double> plantOutput(numSteps);
    std::vector<double> controlSignal(numSteps);

    double currentPlantOutput = 0.0;

    for (int i = 0; i < numSteps; ++i)
    {
        time[i] = startTime + i * sampleTime;

        // Calculate the control signal based on the error.  Pass currentPlantOutput.
        double u = pidController.calculate(referenceSignal[i], currentPlantOutput, sampleTime);
        controlSignal[i] = u;

        // Apply the control signal to the plant and get the new output
        currentPlantOutput = plant.process(u);
        plantOutput[i] = currentPlantOutput;
    }

    // 6. Output the results to a file for plotting
    // Ensure the directory exists
    std::filesystem::create_directories("Export");
    std::ofstream outputFile("Export/pid_control_response.dat");
    if (outputFile.is_open())
    {
        outputFile << "Time\tReference\tControlSignal\tPlantOutput\n";
        for (int i = 0; i < numSteps; ++i)
        {
            outputFile << time[i] << "\t" << referenceSignal[i] << "\t" << controlSignal[i] << "\t" << plantOutput[i] << "\n";
        }
        outputFile.close();
        std::cout << "Simulation to /Export/pid_control_response.dat" << std::endl;
    }
    else
    {
        std::cerr << "Error opening file for writing simulation results!" << std::endl;
    }

    return 0;
}
