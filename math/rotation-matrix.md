# Rotation Matrices

 **Table of content:**
 - [Fundamentals](#fundamentals)
 - [Practical Examples](#practical-examples)
 <a id="fundamentals"></a>
## Fundamentals

### 1. **Basic Concepts**:

- **Rotation Matrix**: A matrix that performs a rotation about an axis in three-dimensional space.
- **Orthogonal Matrix**: A matrix is orthogonal if the transpose of the matrix is equal to the inverse of the matrix. Rotation matrices are a subset of orthogonal matrices.
- **Determinant**: The determinant of a rotation matrix is always +1, reflecting a preservation of orientation and scale.

### 2. **Rotation Matrices in 2D**:

In 2D, a rotation matrix that rotates a vector counterclockwise by an angle $`\theta`$ is given by:
$`R = \begin{pmatrix}
\cos(\theta) & -\sin(\theta) \\
\sin(\theta) & \cos(\theta)
\end{pmatrix} `$

#### C++ Code Example:
```cpp
#include <iostream>
#include <cmath>

void rotate2D(double x, double y, double theta, double& x_rot, double& y_rot) {
    x_rot = x * cos(theta) - y * sin(theta);
    y_rot = x * sin(theta) + y * cos(theta);
}

int main() {
    double x = 1, y = 0, theta = M_PI / 2;  // 90 degree rotation
    double x_rot, y_rot;
    rotate2D(x, y, theta, x_rot, y_rot);
    std::cout << "Rotated Coordinates: (" << x_rot << ", " << y_rot << ")" << std::endl;
    return 0;
}
```

### 3. **Rotation Matrices in 3D**:

In 3D, rotation matrices are more complex. There are rotation matrices for rotation about the x, y, and z-axes:

- Rotation about the x-axis by angle $`\theta`$:
\[ R_x(\theta) = \begin{pmatrix}
1 & 0 & 0 \\
0 & \cos(\theta) & -\sin(\theta) \\
0 & \sin(\theta) & \cos(\theta)
\end{pmatrix} \]

- Rotation about the y-axis by angle $`\theta`$:
\[ R_y(\theta) = \begin{pmatrix}
\cos(\theta) & 0 & \sin(\theta) \\
0 & 1 & 0 \\
-\sin(\theta) & 0 & \cos(\theta)
\end{pmatrix} \]

- Rotation about the z-axis by angle $`\theta`$:
\[ R_z(\theta) = \begin{pmatrix}
\cos(\theta) & -\sin(\theta) & 0 \\
\sin(\theta) & \cos(\theta) & 0 \\
0 & 0 & 1
\end{pmatrix} \]

#### C++ Code Example:
```cpp
#include <iostream>
#include <vector>
#include <cmath>

// Function to compute rotation matrix for rotation around Z-axis
std::vector<std::vector<double>> rotationMatrixZ(double theta) {
    return {
        {cos(theta), -sin(theta), 0},
        {sin(theta), cos(theta), 0},
        {0, 0, 1}
    };
}

int main() {
    double theta = M_PI / 2;  // 90 degree rotation
    auto R = rotationMatrixZ(theta);
    // Print the rotation matrix
    for (auto row : R) {
        for (double val : row) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }
    return 0;
}
```

### 4. **Properties of Rotation Matrices**:

- **Composition**: The product of two rotation matrices is another rotation matrix, representing the composition of the two rotations.
- **Inverses**: The inverse of a rotation matrix is its transpose, and represents a rotation in the opposite direction.
- **Determinant**: The determinant of a rotation matrix is always +1.

 <a id="practical-examples"></a>
## Practical Examples
The understanding of rotation matrices is instrumental in solving a variety of real-world problems in robotics. Here are some practical tasks and how the knowledge of rotation matrices aids in solving them:

### Task 1: Robot Arm Positioning
**Problem**: A robotic arm needs to pick up an object from one location and place it in another. The precise positioning of the end-effector (the part of the robot that interacts with the environment) is crucial for successful pick-and-place operations.

**Solution**:
1. Utilize **Forward Kinematics** to compute the position and orientation of the robot's end-effector given the joint angles.
2. If the desired end-effector position and orientation are known, use **Inverse Kinematics** to calculate the required joint angles.
3. Rotation matrices are used to model the rotations at each joint, allowing for accurate computation of the end-effector's pose.

**Code example**:
```C++
#include <iostream>
#include <cmath>
#include <vector>

// Function to compute rotation matrix for rotation around Z-axis
std::vector<std::vector<double>> rotationMatrixZ(double theta) {
    return {
        {cos(theta), -sin(theta), 0},
        {sin(theta), cos(theta), 0},
        {0, 0, 1}
    };
}

// Function to perform matrix-vector multiplication
std::vector<double> matVecMultiply(const std::vector<std::vector<double>>& mat, const std::vector<double>& vec) {
    std::vector<double> result(3, 0);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i] += mat[i][j] * vec[j];
        }
    }
    return result;
}

int main() {
    // Assume a 2D robot arm with a single joint rotating around the Z-axis
    double joint_angle = M_PI / 4;  // 45 degree rotation
    std::vector<double> end_effector_position = {1, 0, 1};  // Initial position of the end effector
    
    // Compute the rotation matrix for the joint
    auto R = rotationMatrixZ(joint_angle);
    
    // Compute the new position of the end effector
    auto new_position = matVecMultiply(R, end_effector_position);
    
    std::cout << "New Position: (" << new_position[0] << ", " << new_position[1] << ")" << std::endl;
    return 0;
}

```

### Task 2: Mobile Robot Navigation
**Problem**: A mobile robot needs to navigate through an environment to a specified location while avoiding obstacles.

**Solution**:
1. Use **Localization** techniques to estimate the robot's current pose.
2. Employ **Motion Planning** algorithms to plan a path to the target location.
3. Rotation matrices help in representing and updating the robot's orientation, as well as transforming sensor data to a common coordinate system for obstacle avoidance.

**Code Example**:
```C++
#include <iostream>
#include <cmath>
#include <vector>

// Function to compute rotation matrix for rotation around Z-axis
std::vector<std::vector<double>> rotationMatrixZ(double theta) {
    return {
        {cos(theta), -sin(theta), 0},
        {sin(theta), cos(theta), 0},
        {0, 0, 1}
    };
}

// Function to perform matrix-matrix multiplication
std::vector<std::vector<double>> matMatMultiply(const std::vector<std::vector<double>>& mat1, const std::vector<std::vector<double>>& mat2) {
    std::vector<std::vector<double>> result(3, std::vector<double>(3, 0));
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
    return result;
}

int main() {
    // Assume the robot initially faces along the X-axis
    std::vector<std::vector<double>> orientation = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    
    double rotation_angle = M_PI / 2;  // 90 degree rotation
    auto R = rotationMatrixZ(rotation_angle);  // Compute the rotation matrix
    
    // Update the robot's orientation
    orientation = matMatMultiply(orientation, R);
    
    std::cout << "New Orientation Matrix:" << std::endl;
    for (auto row : orientation) {
        for (double val : row) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }
    
    return 0;
}

```

### Task 3: 3D Object Recognition and Pose Estimation
**Problem**: A robot needs to recognize objects in its environment and estimate their 3D pose for interaction purposes, like grasping.

**Solution**:
1. Acquire 3D data of the environment using sensors like LIDAR or depth cameras.
2. Use rotation matrices to align the acquired data, facilitating **Point Cloud Registration**.
3. Employ machine learning algorithms for object recognition and pose estimation, where rotation matrices help in computing and representing the 3D orientation of recognized objects.

### Task 4: Drone Stabilization and Control
**Problem**: A drone needs to maintain a stable flight and follow a specified trajectory.

**Solution**:
1. Utilize **Attitude and Heading Reference Systems (AHRS)** to continuously estimate the drone's orientation.
2. Employ **Control Systems** to adjust the drone's motors to maintain stability and follow the desired trajectory.
3. Rotation matrices are used to model and compute the drone's orientation, aiding in the control and stabilization process.

### Task 5: Human-Robot Interaction
**Problem**: A robot needs to understand human gestures to interact effectively.

**Solution**:
1. Use sensors to capture the motion of a human.
2. Employ rotation matrices to analyze and interpret the captured motion data, enabling **Gesture Recognition**.
3. Develop algorithms to map recognized gestures to robot actions, facilitating intuitive human-robot interaction.

Each of these tasks involves complex algorithms and systems, but at their core, they all leverage the fundamental concept of rotation matrices to manipulate and understand orientations in 3D space.
