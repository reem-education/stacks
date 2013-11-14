/////////////////////////////////////////////////////////////////////////
//
// Secondary Task Functions for Visual Servoing
//
/////////////////////////////////////////////////////////////////////////

#include "secondary_task.h"

namespace secondary_task
{
    // Sigmoid Function --> returns the sigmoid value f(x) at the specified x
    // used for smoothing abrupt transitions between the low and high limits
    double sigmoid(double &x,
                   double &low_limit,
                   double &high_limit,
                   double multiplier,
                   double add)
    {
        return 1 / (1 + (exp((-multiplier*((x - low_limit)/(high_limit - low_limit)))+add)));
    }

    // The joint limit activation vector is based on a sign function to move away from the joint limit
    void makeJointLimitActivationVector(vpColVector &jnt_limit_avoidance_activation,
                                        const KDL::JntArray &kdlJointPosition,
                                        const std::vector<double> &q_max,
                                        const std::vector<double> &q_min,
                                        double JLA_rho0)
    {
        for(unsigned int i=0; i < kdlJointPosition.rows(); i++)
        {
            double jointRange =  q_max.at(i) - q_min.at(i);
            if(    kdlJointPosition.data[i]   >   (q_max.at(i) - (JLA_rho0 * jointRange) )   )
            {
                jnt_limit_avoidance_activation[i] = 1.0;
            }
            else if(    kdlJointPosition.data[i]   <   (q_min.at(i) + (JLA_rho0 * jointRange) )    )
            {
                jnt_limit_avoidance_activation[i] = -1.0;
            }
            else
            {
                jnt_limit_avoidance_activation[i] = 0.0;
            }

            //std::cout << "joint " << i << " with q_min: " << q_min.at(i) << " and q_max: " << q_max.at(i) << " has limit avoidance: " << jnt_limit_avoidance_activation[i] << " because joint at " << kdlJointPosition.data[i] << " and safe range is [" << q_min.at(i) + (JLA_rho0 * jointRange) << ", " << q_max.at(i) - (JLA_rho0 * jointRange) << "]" << std::endl;
        }
    }

    // Creates a Smooth transition function from 0 to 1
    double makeSwitchingGain(double x,
                             double lower_limit,
                             double upper_limit)
    {
        double Gain;
        if(x > upper_limit)
        {
            Gain = 1.0;
        }
        else if(x < lower_limit)
        {
            Gain = 0.0;
        }
        else
        {
            Gain = sigmoid(x, lower_limit, upper_limit);
        }
        return Gain;
    }

    // Create a 2 sided Smooth transition function for the joint limit avoidance injection
    void makeSmoothInjectionGain(vpColVector &smooth_injection_Lambda,
                                 const KDL::JntArray &kdlJointPosition,
                                 const std::vector<double> &q_max,
                                 const std::vector<double> &q_min,
                                 double JLA_rho0,
                                 double JLA_rho1)
    {
        double JLA_min_limit0, JLA_min_limit1, JLA_max_limit0, JLA_max_limit1;
        for(unsigned int i=0; i < kdlJointPosition.rows(); i++)
        {
            double jointRange =  q_max.at(i) - q_min.at(i);
            JLA_min_limit0 = q_min.at(i) + (JLA_rho0 * jointRange);
            JLA_max_limit0 = q_max.at(i) - (JLA_rho0 * jointRange);
            JLA_min_limit1 = JLA_min_limit0 - (JLA_rho1 * JLA_rho0 * jointRange);
            JLA_max_limit1 = JLA_max_limit0 + (JLA_rho1 * JLA_rho0 * jointRange);
            double currentJointPosition = kdlJointPosition.data[i];

            // joint is in "Danger" position of joint max and min
            if(   (currentJointPosition < JLA_min_limit1) || (currentJointPosition > JLA_max_limit1)    )
            {
                smooth_injection_Lambda[i] = 1.0;
            }
            // joint is in "Warning" position to joint minimum
            else if(  (currentJointPosition >= JLA_min_limit1) && (currentJointPosition <= JLA_min_limit0 )  )
            {
                smooth_injection_Lambda[i] = secondary_task::sigmoid(currentJointPosition, JLA_min_limit0, JLA_min_limit1);
            }
            // joint is in "Warning" position to joint maximum
            else if(  (currentJointPosition >= JLA_max_limit0) && (currentJointPosition <= JLA_max_limit1 )  )
            {
                smooth_injection_Lambda[i] = secondary_task::sigmoid(currentJointPosition, JLA_max_limit0, JLA_max_limit1);
            }
            else // joint is in a safe configuration
            {
                smooth_injection_Lambda[i] = 0.0;
            }
        }
    }

    // Compute contorl law using Error Norm
    void computeControlLaw_E_Norm(vpColVector &qdot_E_Norm,
                                  vpMatrix &Projector_E_Norm,
                                  const vpMatrix &taskJacobian,
                                  const vpColVector &error,
                                  double taskLambda,
                                  vpMatrix &identity)
    {
        double denominator;
        denominator = (error.transpose()*taskJacobian*taskJacobian.transpose()*error)[0];
        if (denominator != 0.0 ) // prevent division by zero
        {
        double error_Norm_Square = (error.transpose() * error)[0];


        qdot_E_Norm = -taskLambda
                      * (  error_Norm_Square  /  denominator   )
                      * taskJacobian.transpose()
                      * error;


        vpMatrix Projector_E_Norm_part;
        Projector_E_Norm_part = (taskJacobian.transpose()*error);
        Projector_E_Norm = identity - ((1/denominator)*Projector_E_Norm_part*error.transpose()*taskJacobian);
        }
        else // Set outputs to zero
        {
            unsigned int dimension = taskJacobian.getCols();
            qdot_E_Norm.resize(dimension);
            Projector_E_Norm.resize(dimension,dimension);
            for(unsigned int i=0; i < dimension; i++)
            {
                qdot_E_Norm[i] = 0.0;
                for(unsigned int j=0; j < dimension; j++)
                {
                    Projector_E_Norm[i][j] = 0.0;
                }
            }
        }
    }

    // Compute adaptive gain for ensuring the execution of the secondary task
    void make2ndTaskGain(vpColVector &ensure_2nd_task_Lambda,
                         vpColVector q1dot,
                         vpColVector Pg,
                         double relative_mag_2nd_task)
    {
        for(unsigned int i=0; i < q1dot.getRows(); i++)
        {
            if (Pg[i] != 0.0 ) // prevent division by zero
            {
                ensure_2nd_task_Lambda[i] = (  1 + relative_mag_2nd_task  )
                                            * (  fabs(q1dot[i]) / fabs(Pg[i])  );
            }
            else // set output to zero
            {
                ensure_2nd_task_Lambda[i] = 0.0;
            }
        }
    }


    void computeSecondaryTask(const vpColVector& qdot_primary,
                              const vpMatrix& taskJacobian,
                              double taskLambda,
                              const vpColVector& taskError,
                              const KDL::JntArray& kdlJointPosition,
                              const std::vector<double>& q_min,
                              const std::vector<double>& q_max,
                              bool useLargeProjector,
                              vpColVector& qdot,  //final qdot
                              vpColVector& q1dot, //qdot from the primary task
                              vpColVector& q2dot  //qdot from the secondary task
                              )
    {
      for (int i = 0; i< q1dot.getRows(); ++i)
      {
        q1dot[i] = 0;
        q2dot[i] = 0;
      }
      int number_of_joints = taskJacobian.getCols();

      // ----------------- COMPUTE qdot AND PROJECTOR OPERATOR OF THE CLASSICAL METHOD      
      double switching_Lambda = 0.0;
      // Change VISP default limit on the pseudoInverse SVD to lessen effects of singularities
      vpMatrix taskJacPinv;
      unsigned int taskJacRank;
      taskJacRank = taskJacobian.pseudoInverse(taskJacPinv, 1e-3);
      //compute qdot only considering primary task
      vpColVector qdot_classical;
//      qdot_classical = -taskLambda * taskJacPinv * taskError;
      qdot_classical = qdot_primary;
      // Compute the classical projection operator
      vpMatrix Projector_classical;
      vpMatrix identity;
      identity.eye(number_of_joints);
      Projector_classical = identity - (taskJacPinv*taskJacobian);

      // ----------------- COMPUTE qdot AND PROJECTOR OPERATOR OF THE LARGE PROJECTION OPERATOR METHOD
      vpColVector qdot_E_Norm;
      vpMatrix Projector_E_Norm;
      secondary_task::computeControlLaw_E_Norm(qdot_E_Norm, Projector_E_Norm, taskJacobian, taskError, taskLambda, identity);

      // ----------------- DEFINE TRANSITION BETWEEN THE 2 METHODS
      // Error limits to switch between classical approach and error norm approach
      double E_norm_lower_limit = 0.05; // tune such that no singularity problem from E_norm
      double E_norm_upper_limit = 0.1; // tune such that secondary tasks are still done

      double error_Norm = sqrt((taskError.transpose() * taskError)[0]);

      if ( !useLargeProjector )
      {
        switching_Lambda = 0.0;
      }
      else
      {
        // Switching Gain for switching between using error and error_norm as basis
        switching_Lambda = secondary_task::makeSwitchingGain(error_Norm, E_norm_lower_limit, E_norm_upper_limit);
      }

      // Keep using the classical q_dot to keep an exponential decrease of each feature
      q1dot = qdot_classical;

      // Final projector operator with switching
      vpMatrix Projector_Switching;
      Projector_Switching = (switching_Lambda * Projector_E_Norm)
          + ((1 - switching_Lambda) * Projector_classical);
      //        Projector_Switching = (switching_Lambda * Projector_E_Norm); // not using classical projector


      // ----------------- DEFINE JOINT LIMIT AVOIDANCE AS SECONDARY TASK
      vpColVector jnt_limit_avoidance_activation;
      jnt_limit_avoidance_activation.resize(number_of_joints);
      // warning limit for joint limit avoidance (percent of joint range)
      double JLA_rho0 = 0.05; //0.1
      // Fill the joint limit avoidance activation vector using a sign function
      secondary_task::makeJointLimitActivationVector(jnt_limit_avoidance_activation,
                                                     kdlJointPosition,
                                                     q_max,
                                                     q_min,
                                                     JLA_rho0);


      // ----------------- DEFINE SMOOTH TRANSITION GAINS FOR SECONDARY TASK
      vpColVector smooth_injection_Lambda;
      smooth_injection_Lambda.resize(number_of_joints);
      // danger limit for joint limit avoidance (percent of warning range)
      double JLA_rho1 = 0.5;
      secondary_task::makeSmoothInjectionGain(smooth_injection_Lambda, kdlJointPosition, q_max, q_min, JLA_rho0, JLA_rho1);


      // ----------------- DEFINE ADAPTIVE GAIN TO DO SECONDARY TASK
      vpColVector Pg; // temporary q2dot
      Pg.resize(number_of_joints);
      Pg = Projector_Switching * jnt_limit_avoidance_activation;
      vpColVector ensure_2nd_task_Lambda;
      ensure_2nd_task_Lambda.resize(number_of_joints);
      // magnitude of secondary task relative to primary task
      double relative_mag_2nd_task = 0.5;
      secondary_task::make2ndTaskGain(ensure_2nd_task_Lambda, q1dot, Pg, relative_mag_2nd_task);

      bool useAdaptiveGain = true;

      // ----------------- COMPUTE FINAL SECONDARY TASK
      for(unsigned int i=0; i < number_of_joints; i++)
      {
        if ( useAdaptiveGain )
          q2dot[i] =  -ensure_2nd_task_Lambda[i] * smooth_injection_Lambda[i] * Pg[i];
        else
          q2dot[i] =  -smooth_injection_Lambda[i] * Pg[i];
      }

      // ----------------- COMPUTE FINAL JOINT VELOCITIES
      if(switching_Lambda == 0) // pure VS, not using error norm operators
      {
        if ( !useLargeProjector && (taskJacRank >=6) )
        {
          qdot = q1dot + q2dot;
        }
        else // redundancy is disabled or not available because of singularities
        {
          qdot = q1dot;
        }
      }
      else // case where the error norm is used (might need a rank check also... need to confirm this...)
      {
        qdot = q1dot + q2dot;
      }
    }


} // end namespace secondary_task

