#include <iostream>

#include "manipulator_ik/inverse_kinematics.hpp"

int main()
{
    const double tolerance = 1e-9;
    bool all_passed = true;

    std::cout << "=== PSEUDOINVERSE TESTS ===\n\n";

    // TEST 1
    {
        Eigen::MatrixXd J(2, 2);
        J << 1.0, 0.0,
             0.0, 2.0;

        Eigen::MatrixXd result = Pseudo_Inverse(J);
        Eigen::MatrixXd expected(2, 2);
        expected << 1.0, 0.0,
                    0.0, 0.5;

        double error = (result - expected).norm();
        bool passed = error <= tolerance;
        all_passed = all_passed && passed;

        std::cout << "TEST 1 - Square invertible matrix\n";
        std::cout << "Result:\n" << result << "\n\n";
        std::cout << "Expected:\n" << expected << "\n\n";
        std::cout << "Error: " << error << "\n";
        std::cout << (passed ? "PASSED" : "NOT PASSED") << "\n\n";
    }

    // TEST 2
    {
        Eigen::MatrixXd J(3, 2);
        J << 1.0, 0.0,
             0.0, 2.0,
             0.0, 0.0;

        Eigen::MatrixXd result = Pseudo_Inverse(J);
        Eigen::MatrixXd expected(2, 3);
        expected << 1.0, 0.0, 0.0,
                    0.0, 0.5, 0.0;

        double error = (result - expected).norm();
        bool passed = error <= tolerance;
        all_passed = all_passed && passed;

        std::cout << "TEST 2 - Rectangular 3x2 Jacobian\n";
        std::cout << "Result:\n" << result << "\n\n";
        std::cout << "Expected:\n" << expected << "\n\n";
        std::cout << "Error: " << error << "\n";
        std::cout << (passed ? "PASSED" : "NOT PASSED") << "\n\n";
    }

    // TEST 3
    {
        Eigen::MatrixXd J(2, 2);
        J << 1.0, 0.0,
             0.0, 0.0;

        Eigen::MatrixXd result = Pseudo_Inverse(J);
        Eigen::MatrixXd expected(2, 2);
        expected << 1.0, 0.0,
                    0.0, 0.0;

        double error = (result - expected).norm();
        bool passed = error <= tolerance;
        all_passed = all_passed && passed;

        std::cout << "TEST 3 - Singular matrix\n";
        std::cout << "Result:\n" << result << "\n\n";
        std::cout << "Expected:\n" << expected << "\n\n";
        std::cout << "Error: " << error << "\n";
        std::cout << (passed ? "PASSED" : "NOT PASSED") << "\n\n";
    }

    return all_passed ? 0 : 1;
}
