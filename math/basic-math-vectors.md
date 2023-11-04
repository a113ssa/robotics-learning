### Table of Contents
- [Theory](#theory)
- [Application examples](#application-examples)
- [C++ Code examples](#cpp-code-examples)

<a id="theory"></a>
# Theory

## Vector Spaces

A vector space is a collection of objects called vectors, which can be added together and multiplied by scalars (numbers), and still produce another vector in the same space. Here are some essential properties and operations:

1. **Vector Addition**:
   If $`\vec u`$ and $`\vec{v}`$ are vectors in a vector space $`V`$, their sum $`\vec{u} + \vec{v}`$ is also in $`V`$.
   
2. **Scalar Multiplication**:
   If $`\vec{v}`$ is a vector in $`V`$ and $`c`$ is a scalar, then $`c\vec{v}`$ is also in $`V`$.

The formula for vector addition in $`\mathbb{R}^2`$ (2-dimensional real space) for vectors $`\vec{u} = (u_1, u_2)`$ and $`\vec{v} = (v_1, v_2)`$ is:

 $`\vec{u} + \vec{v} = (u_1 + v_1, u_2 + v_2)`$

For scalar multiplication:

$`c\vec{v} = (cv_1, cv_2)`$

## Span and Basis

- **Span**: The span of a set of vectors $`\{ \vec{v}_1, \vec{v}_2, ..., \vec{v}_k \}`$ in $`V`$ is the set of all linear combinations of these vectors.
- **Basis**: A basis of $`V`$ is a set of vectors in $`V`$ that are linearly independent and span the entire vector space.

The formula for a linear combination of vectors is:

$` a_1\vec{v}_1 + a_2\vec{v}_2 + ... + a_k\vec{v}_k`$

where $`a_1, a_2, ..., a_k`$ are scalars.

## Euclidean Spaces

Euclidean space $`\mathbb{R}^n`$ is an n-dimensional space consisting of all n-tuples of real numbers. Each vector in $`\mathbb{R}^n`$ has n components.

- **Dimension**: The number of vectors in the basis for $`V`$ is called the dimension of $`V`$, denoted as $`\text{dim}(V)`$.

## Norms

A norm is a function that assigns a strictly positive length or size to all vectors in a vector space, except for the zero vector, which is assigned a length of zero. For a vector $`\vec{v} = (v_1, v_2, ..., v_n)`$ in $`\mathbb{R}^n`$, the Euclidean norm (or 2-norm) is defined as:

$`||\vec{v}||_2 = \sqrt{v_1^2 + v_2^2 + ... + v_n^2}`$

## Inner Product (Dot Product)

The inner product (or dot product) is a way of multiplying two vectors together to get a scalar. For vectors $`\vec{u} = (u_1, u_2, ..., u_n)`$ and $`\vec{v} = (v_1, v_2, ..., v_n)`$ in $`\mathbb{R}^n`$, the inner product is defined as:

$`\langle \vec{u}, \vec{v} \rangle = u_1v_1 + u_2v_2 + ... + u_nv_n`$

- **Orthogonality**: Two vectors are orthogonal if their dot product is zero, i.e., $`\langle \vec{u}, \vec{v} \rangle = 0`$.
- **Angle Between Vectors**: The angle $`\theta`$ between two vectors can be found using the dot product:
  
  $`\cos(\theta) = \frac{\langle \vec{u}, \vec{v} \rangle}{||\vec{u}||_2 \cdot ||\vec{v}||_2}`$

The fundamentals of vector spaces, norms, and inner products are critical to various aspects of robotics, from motion planning and control to perception and machine learning. Here's how they are applied in practical robotics projects:

<a id="application-examples"></a>
# Application examples

## Kinematics and Dynamics
- **Vector Spaces**: Robotic arms and manipulators' positions and orientations are described in terms of vectors. The space formed by all possible configurations of a robot's joints is a vector space known as the configuration space.
- **Basis**: In robot kinematics, the basis vectors can represent the primary axes of movement for the robot. For example, in a Cartesian coordinate system, the robot's forward, lateral, and vertical movements can be defined by basis vectors.

## Motion Planning
- **Euclidean Spaces**: The planning of a robot's path involves navigating through Euclidean spaces, often in 2D (for ground robots) or 3D (for aerial or underwater robots). Collision detection and avoidance algorithms use vector mathematics to calculate distances and intersections.
- **Norms**: Norms are used to measure the distance between points in space. For example, when a robot is navigating, the Euclidean norm (2-norm) measures the straight-line distance to the target.

## Control Systems
- **Vector Operations**: When designing controllers for robots, engineers use vectors to represent forces, velocities, and accelerations. The control algorithms involve adding and scaling these vectors to produce the desired movement.
- **Inner Products**: In control theory, the inner product is often used to calculate power and energy consumption by taking the dot product of force and velocity vectors.

## Perception and Sensing
- **Inner Product**: When processing sensor data, the inner product can be used to compare signals or features. For instance, LIDAR and sonar sensors can use inner products to detect obstacles or match patterns for localization (as seen in algorithms like SLAM - Simultaneous Localization and Mapping).
- **Orthogonality**: Orthogonal vectors are used in sensor arrays to distinguish between different sensor modalities or to isolate noise from signals.

## Computer Vision and Machine Learning
- **Norms**: In vision systems, norms can be used to normalize feature vectors, making them invariant to scale, which is crucial for object recognition.
- **Vector Spaces**: Machine learning, especially deep learning that's often used in robotic perception, is heavily based on vector space concepts. Neural networks operate in high-dimensional vector spaces where each dimension can represent a feature of the data.

## Force and Torque Calculations
- **Vectors**: Forces and torques applied by or on a robot are represented as vectors. For example, when a robotic arm interacts with the environment, vectors are used to calculate the necessary forces and torques for manipulation tasks.

## Mechanical Design and Simulation
- **Span**: Engineers use the concept of span to ensure that robotic mechanisms can reach all necessary points in their working envelope.
- **Euclidean Spaces**: Simulation of robots within their operating environment requires an understanding of Euclidean spaces to accurately model movements and interactions.

## Example Problem Solved by These Fundamentals:
Consider a robot arm that needs to pick up an object and place it at a specific location. Vector spaces and linear algebra are used to calculate the arm's path (trajectory planning). The norms are used to ensure that the arm moves the minimum distance necessary (efficiency), and inner products might be involved in optimizing the arm's orientation to grasp the object securely (force closure).

<a id="cpp-code-examples"></a>
# C++ Code examples

## Example 1: Robot Arm Kinematics (C++ Code)
Consider a robot arm that needs to calculate its end effector position based on joint angles (forward kinematics). Vectors are used to represent the positions, and matrix multiplication is used to transform these vectors through the robot's linkages.

```cpp
#include <Eigen/Dense>
#include <iostream>

// Assume a simple 2-link planar arm for demonstration
Eigen::Vector2d forwardKinematics(double theta1, double theta2, double link1, double link2) {
    // Calculate the position of the end effector
    double x = link1 * cos(theta1) + link2 * cos(theta1 + theta2);
    double y = link1 * sin(theta1) + link2 * sin(theta1 + theta2);

    // Return the position as a 2D vector
    Eigen::Vector2d position(x, y);
    return position;
}

int main() {
    double joint1 = M_PI / 4; // 45 degrees
    double joint2 = M_PI / 4; // 45 degrees
    double link1_length = 1.0; // meters
    double link2_length = 0.5; // meters

    Eigen::Vector2d end_effector_pos = forwardKinematics(joint1, joint2, link1_length, link2_length);
    std::cout << "The position of the end effector is: " << end_effector_pos.transpose() << std::endl;

    return 0;
}
```

In the above code, the `Eigen` library is used for vector and matrix operations, which is a common C++ library used in robotics and scientific computing.

## Example 2: Path Planning and Distance Calculation (C++ Code)
A path planning algorithm might use Euclidean distance (norms) to evaluate the shortest path a robot should take to reach its goal without colliding with obstacles.

```cpp
#include <cmath>
#include <iostream>

// Calculate the Euclidean distance between two points in space
double euclideanDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

int main() {
    double start_x = 1.0;
    double start_y = 1.0;
    double goal_x = 4.0;
    double goal_y = 5.0;

    double distance = euclideanDistance(start_x, start_y, goal_x, goal_y);
    std::cout << "The distance to the goal is: " << distance << " units." << std::endl;

    return 0;
}
```

## Example 3: Object Recognition and Dot Product (C++ Code)
In computer vision, used in robotics, the inner product might be used to compare feature vectors for object recognition.

```cpp
#include <vector>
#include <numeric>
#include <iostream>

// Calculate the dot product of two vectors
double dotProduct(const std::vector<double>& v1, const std::vector<double>& v2) {
    return std::inner_product(v1.begin(), v1.end(), v2.begin(), 0.0);
}

int main() {
    std::vector<double> feature_vector1 = {1.0, 2.0, 3.0};
    std::vector<double> feature_vector2 = {4.0, 5.0, 6.0};

    double similarity = dotProduct(feature_vector1, feature_vector2);
    std::cout << "The similarity score is: " << similarity << std::endl;

    return 0;
}
```

In this example, the dot product might be a part of a larger algorithm used to compare the similarity of feature descriptors in image processing, often used in robotics for tasks such as object recognition or localization.

### Real-world Robotics Software
In the real-world robotics industry, frameworks like ROS (Robot Operating System) employ these mathematical concepts extensively. Developers write nodes in C++ or Python that handle tasks such as:

- **Sensor data processing**: where they manipulate vectors to represent data from LIDAR, cameras, IMUs, etc.
- **Actuator control**: sending commands in the form of vectors to motors or servos.
- **State estimation**: using vectors to estimate positions and orientations (pose) in space.
