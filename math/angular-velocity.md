## Angular Velocity
 **Table of content:**
 - [Fundamentals](#fundamentals)
 - [Practical Examples](#practical-examples)

 <a id="fundamentals"></a>
## Fundamentals

### Definition:
Angular velocity, usually denoted by $`\omega`$ or $`\Omega`$, is a vector quantity that specifies the angular speed of an object and the axis about which the object is rotating. The magnitude of angular velocity is the rate of rotation in radians per unit time, and its direction follows the right-hand rule: if you curl the fingers of your right hand in the direction of rotation, your thumb points in the direction of the angular velocity vector.

### Mathematical Representation:
The angular velocity vector $`\omega`$ in 3D space can be represented as:
$`\omega = [\omega_x, \omega_y, \omega_z]^T`$
where $`\omega_x, \omega_y, \omega_z`$ are the components of angular velocity around the x, y, and z axes respectively.

### Relation to Rotation Matrices:
Angular velocity is closely related to the concept of rotation. Given a rotation matrix $`R`$, the angular velocity $`\omega`$ can be related to the derivative of $`R`$ with respect to time as follows:
$`\dot{R} = R \cdot \omega^\times`$
where $`\omega^\times`$ is the skew-symmetric matrix form of $`\omega`$, and $`\dot{R}`$ is the time derivative of $`R`$.

### Skew-Symmetric Matrix:
The skew-symmetric matrix $`\omega^\times`$ associated with angular velocity $`\omega`$ is given by:
$`\omega^\times = \begin{pmatrix}
0 & -\omega_z & \omega_y \\
\omega_z & 0 & -\omega_x \\
-\omega_y & \omega_x & 0
\end{pmatrix}`$

### Integration and Differentiation:
- **Integration**: Integrating angular velocity over time gives the rotation angle $`\theta`$ around the rotation axis.
- **Differentiation**: Differentiating the rotation angle with respect to time yields the angular velocity.

### Applications in Robotics:
1. **Kinematics**: Angular velocity is fundamental for understanding the kinematics of robots, especially the relationship between joint velocities and the velocity of the end-effector.
2. **Control**: Angular velocity is used in control algorithms to achieve desired orientations or to follow a specified rotational trajectory.
3. **Localization**: In mobile robotics, understanding the angular velocity helps in estimating the robot's pose (position and orientation) over time.
4. **Sensor Data Processing**: Gyroscopes provide measurements of angular velocity which can be integrated over time to estimate the orientation of a robot.

### Code Snippet (C++):
```c++
#include <iostream>
#include <vector>

// Function to compute the skew-symmetric matrix of a vector
std::vector<std::vector<double>> skewSymmetricMatrix(const std::vector<double>& omega) {
    return {
        {0, -omega[2], omega[1]},
        {omega[2], 0, -omega[0]},
        {-omega[1], omega[0], 0}
    };
}

int main() {
    std::vector<double> angular_velocity = {0, 0, 1};  // Angular velocity around Z-axis
    auto omega_cross = skewSymmetricMatrix(angular_velocity);

    std::cout << "Skew-Symmetric Matrix:" << std::endl;
    for (auto row : omega_cross) {
        for (double val : row) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
```

This code snippet demonstrates how to compute the skew-symmetric matrix associated with a given angular velocity vector in C++.

 <a id="practical-examples"></a>
## Practical Examples
Angular velocity plays a critical role in various robotic applications. Here are some tasks and their solutions leveraging the concept of angular velocity:

### Task 1: Estimating Robot Orientation using Gyroscope Data
A common task in robotics is to estimate the orientation of a robot using data from an onboard gyroscope.

**Solution**:
1. Obtain the angular velocity $`\omega`$ from the gyroscope.
2. Integrate the angular velocity over time to get the change in orientation.
3. Update the robot's current orientation estimate.

```c++
#include <iostream>
#include <vector>
#include <cmath>

// Function to integrate angular velocity to get change in orientation (assuming constant angular velocity)
std::vector<double> integrateAngularVelocity(const std::vector<double>& omega, double dt) {
    // For simplicity, assume the robot starts at orientation (0, 0, 0)
    static std::vector<double> orientation = {0, 0, 0};

    // Update orientation by integrating angular velocity
    for (int i = 0; i < 3; ++i) {
        orientation[i] += omega[i] * dt;
    }

    return orientation;
}

int main() {
    std::vector<double> angular_velocity = {0, 0, 0.1};  // Angular velocity around Z-axis (rad/s)
    double time_step = 0.1;  // Time step (s)

    // Simulate 10 time steps
    for (int i = 0; i < 10; ++i) {
        auto orientation = integrateAngularVelocity(angular_velocity, time_step);
        std::cout << "Orientation: (" << orientation[0] << ", " << orientation[1] << ", " << orientation[2] << ")" << std::endl;
    }

    return 0;
}
```

### Task 2: Control of a Robotic Arm
A robotic arm needs to be controlled to follow a specified rotational trajectory.

**Solution**:
1. Determine the desired angular velocity $`\omega_{desired}`$ to follow the trajectory.
2. Measure the current angular velocity $`\omega_{current}`$ of the robotic arm.
3. Compute the control signal to drive $`\omega_{current}`$ to $`\omega_{desired}`$.

```c++
#include <iostream>
#include <vector>

// Function to compute control signal
double computeControlSignal(double omega_desired, double omega_current, double Kp) {
    // Proportional control to minimize the error
    double error = omega_desired - omega_current;
    return Kp * error;
}

int main() {
    double omega_desired = 1.0;  // Desired angular velocity (rad/s)
    double omega_current = 0.5;  // Current angular velocity (rad/s)
    double Kp = 1.0;  // Proportional gain

    // Compute control signal
    double control_signal = computeControlSignal(omega_desired, omega_current, Kp);
    std::cout << "Control Signal: " << control_signal << std::endl;

    return 0;
}
```

### Task 3: Simulating a Spinning Wheel
Simulate the motion of a spinning wheel given an initial angular velocity.

**Solution**:
1. Given the initial angular velocity $`\omega_{initial}`$, compute the orientation of the wheel at each time step.

```c++
#include <iostream>
#include <cmath>

int main() {
    double omega_initial = 1.0;  // Initial angular velocity (rad/s)
    double time_step = 0.1;  // Time step (s)

    // Simulate 10 time steps
    for (int i = 0; i < 10; ++i) {
        double theta = omega_initial * i * time_step;  // Compute orientation
        std::cout << "Orientation: " << theta << " rad" << std::endl;
    }

    return 0;
}
```

In these examples, angular velocity is used to estimate orientation, control a robotic arm, and simulate the motion of a spinning wheel. The code snippets illustrate how to implement these solutions in C++.
