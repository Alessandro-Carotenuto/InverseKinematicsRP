    #include "manipulator_ik/inverse_kinematics.hpp"


    //PSEUDO INVERSE VIA SVD 
    Eigen::MatrixXd Pseudo_Inverse(Eigen::MatrixXd& Jacobian, double tolerance)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd = Jacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXd inverseSingularValues = svd.singularValues(); //initializing them as the singular val

        //INVERT SINGUALR VALUES
        
        for (int i =0; i<inverseSingularValues.size(); i++)
        {
            if (inverseSingularValues[i]<=tolerance)
            {
                inverseSingularValues[i]=0;
            }
            else
            {
                inverseSingularValues[i]=1/inverseSingularValues[i];
            }
        }

        Eigen::MatrixXd V = svd.matrixV();
        Eigen::MatrixXd U = svd.matrixU();
        Eigen::MatrixXd pinvSV = inverseSingularValues.asDiagonal();

        return V*pinvSV*U.transpose();

    }
