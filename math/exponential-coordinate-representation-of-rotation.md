# Exponential Coordinate Representation of Rotation
 **Table of content:**
 - [Fundamentals](#fundamentals)
 - [Practical Examples](#practical-examples)
 <a id="fundamentals"></a>
## Fundamentals

Exponential coordinate representation is a compact way to express 3D rotations and is particularly useful in robotics for tasks like manipulation and navigation. In this representation, a rotation is defined using an axis of rotation and the angle of rotation about that axis. Here are the fundamentals:

### Axis-Angle Representation:
A 3D rotation can be represented by an axis of rotation and an angle of rotation about that axis. This is known as axis-angle representation, denoted as $`(\mathbf{a}, \theta)`$ where:
- $`\mathbf{a}`$ is a unit vector along the axis of rotation.
- $`\theta`$ is the angle of rotation in radians.

### Exponential Coordinates:
The axis-angle representation can be further compacted into exponential coordinates using the Rodrigues' rotation formula. The exponential coordinates for a rotation are given by:
$`\mathbf{w} = \theta \cdot \mathbf{a}`$

Here, $`\mathbf{w}`$ is a vector whose magnitude is the angle of rotation $`\theta`$, and whose direction is along the axis of rotation $`\mathbf{a}`$.

### Rotation Matrix:
The rotation matrix $`R`$ corresponding to the exponential coordinates $`\mathbf{w}`$ can be derived using the Rodrigues' rotation formula:
$`R = I + (\sin \theta) \cdot \mathbf{a}^\times + (1 - \cos \theta) \cdot \mathbf{a}^\times \cdot \mathbf{a}^\times`$

where:
- $`I`$ is the identity matrix.
- $`\mathbf{a}^\times`$ is the skew-symmetric matrix of the axis vector $`\mathbf{a}`$.

### Skew-Symmetric Matrix:
The skew-symmetric matrix $`\mathbf{a}^\times`$ for a given vector $`\mathbf{a} = [a_x, a_y, a_z]^T`$ is given by:
$`\mathbf{a}^\times = \begin{pmatrix} 0 & -a_z & a_y \\ a_z & 0 & -a_x \\ -a_y & a_x & 0 \end{pmatrix}`$

### Conversion Between Representations:
You can convert between the exponential coordinates $`\mathbf{w}`$, the axis-angle representation $`(\mathbf{a}, \theta)`$, and the rotation matrix $`R`$ using the formulas provided above.

### Applications in Robotics:
Exponential coordinate representation is useful in various robotic applications including:
- Manipulator kinematics.
- Path planning and trajectory generation.
- Control algorithms for achieving desired orientations.

### Code Example (C++):
```c++
#include <iostream>
#include <vector>
#include <cmath>

// Function to compute the skew-symmetric matrix of a vector
std::vector<std::vector<double>> skewSymmetricMatrix(const std::vector<double>& a) {
    return {
        {0, -a[2], a[1]},
        {a[2], 0, -a[0]},
        {-a[1], a[0], 0}
    };
}

// Function to compute the rotation matrix from exponential coordinates
std::vector<std::vector<double>> computeRotationMatrix(const std::vector<double>& w) {
    double theta = std::sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);  // Angle of rotation
    std::vector<double> a = {w[0]/theta, w[1]/theta, w[2]/theta};  // Axis of rotation
    auto a_cross = skewSymmetricMatrix(a);

    // Compute the rotation matrix using Rodrigues' formula
    std::vector<std::vector<double>> R = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R[i][j] += (sin(theta) * a_cross[i][j] +
                        (1 - cos(theta)) * (a_cross[i][0]*a_cross[0][j] + a_cross[i][1]*a_cross[1][j] + a_cross[i][2]*a_cross[2][j]));
        }
    }

    return R;
}

int main() {
    std::vector<double> w = {0, 0, M_PI/4};  // Exponential coordinates for a 45-degree rotation about Z-axis
    auto R = computeRotationMatrix(w);

    std::cout << "Rotation Matrix:" << std::endl;
    for (auto row : R) {
        for (double val : row) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
```

In this code example, a function `computeRotationMatrix` is implemented to convert exponential coordinates to a rotation matrix using Rodrigues' rotation formula.

 <a id="practical-examples"></a>
## Practical Examples
The exponential coordinate representation of rotation is fundamental in various robotics applications, especially in robot manipulator control, motion planning, and SLAM (Simultaneous Localization and Mapping). Here are some tasks and solutions where these fundamentals are utilized:

### Task 1: Robot Manipulator Control
Controlling a robot manipulator to reach a desired orientation.

**Solution**:
1. Determine the current orientation of the robot manipulator end-effector.
2. Determine the desired orientation.
3. Compute the exponential coordinates to represent the rotation from the current orientation to the desired orientation.
4. Use a control law to drive the robot manipulator to the desired orientation.

```c++
#include <iostream>
#include <vector>
#include <cmath>

// Function to compute the skew-symmetric matrix of a vector
std::vector<std::vector<double>> skewSymmetricMatrix(const std::vector<double>& a) {
    return {
        {0, -a[2], a[1]},
        {a[2], 0, -a[0]},
        {-a[1], a[0], 0}
    };
}

// Function to compute the rotation matrix from exponential coordinates
std::vector<std::vector<double>> computeRotationMatrix(const std::vector<double>& w) {
    double theta = std::sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);  // Angle of rotation
    std::vector<double> a = {w[0]/theta, w[1]/theta, w[2]/theta};  // Axis of rotation
    auto a_cross = skewSymmetricMatrix(a);

    // Compute the rotation matrix using Rodrigues' formula
    std::vector<std::vector<double>> R = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R[i][j] += (sin(theta) * a_cross[i][j] +
                        (1 - cos(theta)) * (a_cross[i][0]*a_cross[0][j] + a_cross[i][1]*a_cross[1][j] + a_cross[i][2]*a_cross[2][j]));
        }
    }

    return R;
}

// Assume a simplistic control law to drive the robot to the desired orientation
void controlRobotManipulator(const std::vector<std::vector<double>>& R_desired) {
    // For simplicity, this function just prints the desired rotation matrix
    std::cout << "Controlling robot to reach the desired orientation:" << std::endl;
    for (auto row : R_desired) {
        for (double val : row) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }
}

int main() {
    std::vector<double> w = {0, 0, M_PI/4};  // Exponential coordinates for a 45-degree rotation about Z-axis
    auto R_desired = computeRotationMatrix(w);

    controlRobotManipulator(R_desired);

    return 0;
}
```

### Task 2: Motion Planning
Planning the motion of a robot to avoid obstacles while moving from a start to a goal orientation.

**Solution**:
1. Represent the start and goal orientations using exponential coordinates.
2. Plan a smooth path in the space of exponential coordinates.
3. Execute the path using a control law to drive the robot along the planned path.

(Note: The code for this task would be quite extensive as it involves path planning algorithms, collision checking, and a control loop for executing the path.)

### Task 3: SLAM (Simultaneous Localization And Mapping)
Estimating the orientation of a robot as it explores an unknown environment.

**Solution**:
1. Acquire sensor data to measure the robot's rotation.
2. Convert the measured rotation to exponential coordinates.
3. Update the robot's orientation estimate using the exponential coordinates.

(Note: The code for this task would also be extensive as it involves sensor data processing, a state estimation algorithm such as a Kalman filter or a particle filter, and map building.)

These tasks demonstrate how the exponential coordinate representation of rotation can be utilized in different robotics applications.
