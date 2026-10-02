#include <iostream>

#include "manipulator_ik/inverse_kinematics.hpp"

int main()
{
    std::cout << "=== PSEUDOINVERSE TESTS ===\n\n";

    // TEST 1
    {
        Eigen::MatrixXd J(2, 2);

        J << 1.0, 0.0,
             0.0, 2.0;

        Eigen::MatrixXd J_pinv = Pseudo_Inverse(J);

        std::cout << "TEST 1 - Square invertible matrix\n";
        std::cout << "J:\n" << J << "\n\n";
        std::cout << "J pseudoinverse:\n" << J_pinv << "\n\n";

        std::cout << "Expected:\n";
        std::cout << "1   0\n";
        std::cout << "0 0.5\n\n";
    }

    // TEST 2
    {
        Eigen::MatrixXd J(3, 2);

        J << 1.0, 0.0,
             0.0, 2.0,
             0.0, 0.0;

        Eigen::MatrixXd J_pinv = Pseudo_Inverse(J);

        std::cout << "TEST 2 - Rectangular 3x2 Jacobian\n";
        std::cout << "J:\n" << J << "\n\n";
        std::cout << "J pseudoinverse:\n" << J_pinv << "\n\n";

        std::cout << "Expected size: 2x3\n";
        std::cout << "Expected:\n";
        std::cout << "1   0 0\n";
        std::cout << "0 0.5 0\n\n";
    }

    // TEST 3
    {
        Eigen::MatrixXd J(2, 2);

        J << 1.0, 0.0,
             0.0, 0.0;

        Eigen::MatrixXd J_pinv = Pseudo_Inverse(J);

        std::cout << "TEST 3 - Singular matrix\n";
        std::cout << "J:\n" << J << "\n\n";
        std::cout << "J pseudoinverse:\n" << J_pinv << "\n\n";

        std::cout << "Expected:\n";
        std::cout << "1 0\n";
        std::cout << "0 0\n\n";
    }

    return 0;
}