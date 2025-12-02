/******************************************************************************
**
** @file        PID
** @author      Sebastian Gomez
** @date        2025-04-24
** @brief       PID controller implementation
** @project     Phoenix
** @version     1
**
******************************************************************************/
#ifndef LIBRARIES_CONTROL_PID_H
#define LIBRARIES_CONTROL_PID_H
#include <algorithm> // For std::clamp
#include <limits>    // For std::numeric_limits
namespace Libraries
{
    namespace Control
    {
        /*
        ** Proportional-Integral-Derivative (PID) Controller Class
        */
        template <typename T>
        class PID
        {
        public:
            PID() : e(0),
                    kd(0),
                    ki(0),
                    kp(0),
                    i(0),
                    minOutput(std::numeric_limits<T>::lowest()),
                    maxOutput(std::numeric_limits<T>::max())
            {
            }
            virtual ~PID()
            {
            }
            /**
             * Calculation of control signal next step
             * u(t) = K_p e(t) + K_i \int_0^t e(tau) dtau + K_d \frac{de(t)}{dt}
             */
            T calculate(const T &setpoint, const T &currentValue, float dt)
            {
                T error = setpoint - currentValue;
                T proportional = this->kp * error;
                this->i += this->ki * error * dt;
                T derivative = this->kd * (error - this->e) / dt;
                this->e = error;

                // Calculate the raw output
                T output = proportional + this->i + derivative;

                // Clamp the output to the specified limits
                return std::clamp(output, minOutput, maxOutput);
            }

            void reset()
            {
                this->e = 0;
                this->i = 0;
            }
            /**
             * Set output limits for the control signal.
             */
            void setOutputLimits(T min, T max)
            {
                minOutput = min;
                maxOutput = max;
            }

            T kd;
            T ki;
            T kp;

        protected:
        private:
            T e;         // Error
            T i;         // Integral term
            T minOutput; // Minimum output limit
            T maxOutput; // Maximum output limit
        };
    } // namespace Control
} // namespace Libraries
#endif
