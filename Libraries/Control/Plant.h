#ifndef PLANT_H
#define PLANT_H

#include <vector>
#include <stdexcept> // For std::invalid_argument
#include <algorithm> // For std::fill

namespace Libraries {
    namespace Control {

        template <typename T>
        class Plant {
        private:
            std::vector<T> a; // Denominator coefficients (feedback)
            std::vector<T> b; // Numerator coefficients (feedforward)
            std::vector<T> input_history;
            std::vector<T> output_history;
            int order;

        public:
            // Constructor: Initialize the Plant object with the transfer function coefficients.
            Plant(std::vector<T> b_coeffs, std::vector<T> a_coeffs)
                : b(b_coeffs), a(a_coeffs), order(a_coeffs.size() - 1),
                  input_history(order, 0.0), output_history(order, 0.0) {
                if (a_coeffs.empty() || a_coeffs[0] == 0.0) {
                    throw std::invalid_argument("Denominator coefficients must be provided and the first coefficient cannot be zero.");
                }
                if (b_coeffs.empty()) {
                    throw std::invalid_argument("Numerator coefficients must be provided.");
                }

                // Normalize the denominator coefficients so that a[0] is 1.
                T a0 = a[0];
                for (auto &coeff : a) {
                    coeff /= a0;
                }
                for (auto &coeff : b) {
                    coeff /= a0;
                }
            }

            // Destructor
            ~Plant() {}

            // Function to calculate the output of the system given an input.
            T process(T input) {
                // Store the current input
                input_history.insert(input_history.begin(), input);
                input_history.pop_back();

                // Calculate the output using the difference equation:
                T output = 0.0;

                // Calculate the feedforward part (numerator)
                for (size_t i = 0; i < b.size(); ++i) {
                    if (i < input_history.size())
                        output += b[i] * input_history[i];
                }

                // Calculate the feedback part (denominator)
                for (size_t i = 1; i < a.size(); ++i) {
                    output -= a[i] * output_history[i - 1];
                }

                // Store the current output
                output_history.insert(output_history.begin(), output);
                output_history.pop_back();

                return output;
            }

            // Function to reset the Plant's state (input and output history).
            void reset() {
                std::fill(input_history.begin(), input_history.end(), 0.0);
                std::fill(output_history.begin(), output_history.end(), 0.0);
            }

            // Method to get the order of the system.
            int getOrder() const { return order; }

            // Method to get numerator coefficients.
            const std::vector<T>& getNumeratorCoefficients() const {
                return b;
            }

            // Method to get denominator coefficients.
            const std::vector<T>& getDenominatorCoefficients() const {
                return a;
            }
        };

    } // namespace Control
} // namespace Libraries

#endif // PLANT_H