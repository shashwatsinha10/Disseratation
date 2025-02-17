#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <eigen3/Eigen/Dense>
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/utils/DQ_Constants.h>

using namespace DQ_robotics;

// Function to load a CSV file into an Eigen matrix
MatrixXd load_csv(const std::string& filename){
    std::ifstream file(filename);
    if (!file.is_open()){
        throw std::runtime_error("Could not open file " + filename);
    }
    std::vector<std::vector<double>> data; // temporary container for data
    std::string line;

    // Read the csv line by line
    while(std::getline(file, line)){
        std::stringstream line_stream(line);
        std::string cell;
        std::vector<double> row_data;

        // Split each line by commas and convert to double
        while (std::getline(line_stream, cell, ',')){
            row_data.push_back(std::stod(cell));// convert and store in row
        }
        data.push_back(row_data); // Add row to data
    }
    file.close();

    // Convert std::vector to eigen matrix
    size_t rows = data.size();
    size_t cols = data[0].size();
    MatrixXd matrix(rows, cols);

    for (size_t i = 0; i < rows; ++i){
        for (size_t j = 0; j < cols; ++j){
            matrix(i, j) = data[i][j];
        }
    }
    return matrix;
}

// Function to print an Eigen matrix
void print_matrix(const MatrixXd& matrix) {
    std::cout << matrix << std::endl;
}

// Function to normalize demonstrated motion data
MatrixXd normalize_demonstrated_motion (const MatrixXd& demonstrated_motion){
    // Get the number of rows and columns dynamically
    int rows = demonstrated_motion.rows();
    int cols = demonstrated_motion.cols();

    // Initialize the normalized demonstration matrix with the same size
    MatrixXd normalized_demonstration(rows, cols);

    for (int i = 0; i < cols; ++i) {
        // Extract one demonstration point (a column vector)
        VectorXd demo_point = demonstrated_motion.col(i);

        // Create a DQ object from the extracted column
        DQ demo_point_dq(demo_point);

        // Normalize the demonstration point
        DQ normalized_demo_point = demo_point_dq.normalize();
        
        // Convert the normalized DQ to an 8-element vector
        VectorXd normalized_vec = normalized_demo_point.vec8();

        // Store the normalized vector in the corresponding column
        normalized_demonstration.col(i) = normalized_vec;
    }

    return normalized_demonstration;
}

// Function to compute imitated path
MatrixXd computeImitatedMotion(const DQ& new_goal, const MatrixXd& normalized_demonstration){
    // Get the number of column
    int rows = normalized_demonstration.rows();
    int cols = normalized_demonstration.cols();
    MatrixXd  imitated_path (rows, cols);
    
    // Loop through the demonstration motion data in reverse order
    for (int i = cols - 1; i >= 1; --i){
        // Convert the (i-1)-th column vector to a DQ object
        DQ demo_dq(normalized_demonstration.col(i - 1));
        // Convert the last column (end) vector to a DQ object
        DQ demo_end_dq(normalized_demonstration.col(cols - 1));
        // Compute segma using equation (2): segma = inv(demo_dq) * demo_end_dq
        DQ segma = demo_dq.inv() * demo_end_dq;
        // Compute the imitated dual quaternion using equation (3): imi_dq = new_goal * inv(segma)
        DQ imi_dq = new_goal * segma.inv();
        // Normalize the imitated dual quaternion: norm_imi_dq = imi_dq * inv(norm(imi_dq))
        //DQ norm_imi_dq = imi_dq.normalize();
        DQ norm_imi_dq = imi_dq * imi_dq.norm().inv();
        // Convert the normalized DQ to an 8-element vector and store it
        imitated_path.col(i - 1) = norm_imi_dq.vec8();
    }
    // Store the new goal as the last column in the imitated path
    imitated_path.col(cols - 1) = new_goal.vec8();

    return imitated_path;
}

// Function to perform sclerp interpolation between two dual quaternions
MatrixXd sclerp(const DQ& q1_dq, const DQ& q2_dq, double stepsize) {
    // Normalize the input dual quaternions
    DQ norm_q1_dq = q1_dq.normalize();
    DQ norm_q2_dq = q2_dq.normalize();

    // Compute the relative transformation
    DQ mul = norm_q1_dq.inv() * norm_q2_dq;

    // Compute the number of steps: Columns
    int cols = static_cast<int>(1.0 / stepsize) + 1;

    VectorXd sample_vec = norm_q1_dq.vec8();
    int rows = sample_vec.size();

    // Initialize the interpolation matrix
    MatrixXd M(rows, cols);

    int i = 0;
    for (double tau = 0; tau <= 1; tau += stepsize) {
        DQ q_intm = norm_q1_dq * (mul.pow(tau));

        // Convert the interpolated DQ to an 8-element vector
        VectorXd q_intm_vec = q_intm.vec8();

        M.col(i) = q_intm_vec;
        i++;
    }
    std::cout << "Interpolation completed" << std::endl;  
    return M;
}

// Function to compute the error between the two dual quaternions
std::pair<double, double> computeError(const DQ& dq1, const DQ& dq2) {
    // Compute the translation error
    DQ translation_diff = dq1.translation() - dq2.translation();
    double e_p = translation_diff.vec8().norm();

    // Compute the orientation error
    DQ rotation_diff = dq1.rotation() - dq2.rotation();
    DQ rotation_sum = dq1.rotation() + dq2.rotation();
    double e_o = std::min(rotation_diff.vec8().norm(), rotation_sum.vec8().norm());

    // Return the errors as pair:
    return {e_p, e_o};
}


int main(){
    // intialize the vrep interface
    DQ_VrepInterface vi;
    vi.disconnect_all();
    vi.connect(19997, 100, 10);
    std::cout << "Starting V-REP simulation..." << std::endl;
    vi.start_simulation();

    // get the robot model 
    DQ_SerialManipulatorMDH panda = FrankaEmikaPandaRobot::kinematics();

    std::vector<std::string> joint_names = {
        "Franka_joint1",
        "Franka_joint2",
        "Franka_joint3",
        "Franka_joint4",
        "Franka_joint5",
        "Franka_joint6",
        "Franka_joint7" 
    };
    
    // set up the minimum and maximum joint limits
    VectorXd q_min(7);
    q_min << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;

    VectorXd q_max(7);
    q_max << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;

    // Goal - apply fkm on this goal
    VectorXd goal(7);
    goal << 0.3000, 1, 0.2618, -0.8727, 0, 1.3963, 0;

    // fkm
    DQ new_goal = panda.fkm(goal);

    //std::cout << "New goal            : " << new_goal << std::endl;

    // Normalize the new goal
    DQ normalized_new_goal = new_goal.normalize();

    // Print the new goal (dual quaternion)
    //std::cout << "Normalized New goal: " << normalized_new_goal << std::endl;
    MatrixXd demonstrated_motion; 
    try {
        // Load the matrix from the CSV file
        demonstrated_motion = load_csv("/home/sinha/Documents/dissertation/cpp/demo5.csv");

        // Print the loaded matrix
        //std::cout << "Loaded matrix from demo5.csv:" << std::endl;
        //print_matrix(demonstrated_motion);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }

    // Normalize the demonstrated motion data
    MatrixXd normalized_demonstration = normalize_demonstrated_motion(demonstrated_motion);
    // Print the normalized demonstration data
    //std::cout << "Normalized demonstrated motion data:" << std::endl;
    //print_matrix(normalized_demonstration);

    // Compute the imitated path
    MatrixXd imitated_path = computeImitatedMotion(normalized_new_goal, normalized_demonstration);
    // Print the imitated path 
    //std::cout << "Imitated path:" << std::endl;
    //print_matrix(imitated_path);

    // Set tolerance between current and guiding point
    // tol_p: tolerance in translation (position), tol_o: tolerance in orientation
    double tol_p = 1e-1; // 0.1 for translation tolerance
    double tol_o = 1e-1; // 0.1 for orientation tolerance

    // Initialize guiding point
    VectorXd guiding_point = imitated_path.col(0); // Extract the first column 
    // Convert the guiding point to a DQ object
    DQ guiding_point_dq(guiding_point); // Create a dual quaternion object from the vector

    // Initialize variables
    int uu = 2; // Iteration counter for interpolation
    int i = imitated_path.cols() / 4; // Set the number of the first guiding point (1/4th of total path)

    // Initialize error between current pose and the goal
    VectorXd e_goal = VectorXd::Zero(8); // 8-element vector initialized to zero
    e_goal(0) = 1.0; // Set the first element to 1.0

    // Initialize errors for translation (position) and orientation
    double e_p = 0.0; // Error in translation (position)
    double e_o = 0.0; // Error in orientation

    // Initialize counters
    int temp = 1; // Counter for plotting
    int c = 2;    // Counter for recording the current pose matrix

    // Set the threshold for the controller
    double threshold = 0.08; // Threshold for controller

    // Initialize the total iteration counter
    int count_iteration = 1; // Counter for the total iterations

    MatrixXd interpolationMatrix; // Placeholder for the interpolation matrix

    VectorXd q_current(7);
    DQ x_current_dq;
    while(e_goal.norm() > 0.01) {
        // Update the current joint position
        q_current = vi.get_joint_positions(joint_names);
        x_current_dq = panda.fkm(q_current);

        // check if the current pose is within the tolerance
        if(e_p < tol_p && e_o < tol_o && i < imitated_path.cols() - 1){
            i += 1;  // Increment guiding point index
            guiding_point = imitated_path.col(i);  // Update guiding point
            guiding_point_dq = DQ(guiding_point);  // Convert to DQ object

            // Interpolation step size
            double step = 0.25;

            interpolationMatrix = sclerp(x_current_dq, guiding_point_dq, step);
            uu = 2;  // Reset interpolation counter
        }

        // Compute the eror between the guiding point and the current pose:
        auto [new_e_p, new_e_o] = computeError(guiding_point_dq, x_current_dq);
        e_p = new_e_p;
        e_o = new_e_o;

        // Kinematic contro
        std::cout << "Starting control loop:" << std::endl;

        // Intermediate control goal
        VectorXd xd = interpolationMatrix.col(uu-1);
        // Initialize the error vector
        VectorXd e = VectorXd::Zero(8);
        e(0) = 1.0;

        while (e.norm() > threshold){
            // Update the current joint position
            VectorXd q = vi.get_joint_positions(joint_names);

            // Compute the current end effector pose
            DQ x = panda.fkm(q);

            // Compute te error between the current pose and the intermediate contol gain
            e = (x-DQ(xd)).vec8();

            // Compute the pose Jacobian
            MatrixXd J = panda.pose_jacobian(q);

            // Compute the control inpute (u) using a simple kinematics controller
            VectorXd u = -0.01 * J.completeOrthogonalDecomposition().solve(e);

            // Update the joint positions
            q += u;

            // Send the updated joint positions to V-Rep
            vi.set_joint_positions(joint_names, q);

            count_iteration++;

            // Recompute the error between the current pose and the guiding point
            auto [new_e_p, new_e_o] = computeError(guiding_point_dq, x);
            e_p = new_e_p;
            e_o = new_e_o;

        }
        
        if (uu < interpolationMatrix.cols()) uu++;
        if (i == demonstrated_motion.cols() - 1) threshold = 0.01;
        if (count_iteration >= 15000) break;
        e_goal = (x_current_dq - normalized_new_goal).vec8();
    }
    vi.stop_simulation();
    vi.disconnect();
}