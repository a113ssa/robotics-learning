Rigid-Body Motions in the Plane are fundamental in robotics, computer graphics, and many fields where spatial transformations are essential. The two primary rigid-body motions are **rotation** and **translation**. In a 2D plane, these motions are usually represented using a 2x2 rotation matrix for rotation and a 2D vector for translation. However, it's more convenient to use a single 3x3 matrix to represent both rotation and translation using homogeneous coordinates. This representation is known as a **transformation matrix**.

Here's a breakdown of the concepts along with C++ code examples and tasks that can be tackled with this knowledge:

### 1. Rotation:
A rotation in 2D is represented by a rotation matrix:

<br>

$`
R = \begin{pmatrix}
\cos(\theta) & -\sin(\theta) \\
\sin(\theta) & \cos(\theta)
\end{pmatrix}
`$

<br>

C++ code example:
```c++
#include <cmath>
#include <iostream>
#include <vector>

// Function to rotate a point
std::vector<double> rotate_point(const std::vector<double>& point, double theta) {
    double x_new = point[0]*cos(theta) - point[1]*sin(theta);
    double y_new = point[0]*sin(theta) + point[1]*cos(theta);
    return {x_new, y_new};
}

// Example usage
int main() {
    std::vector<double> point = {1, 0};
    double theta = M_PI / 2;  // 90 degree rotation
    std::vector<double> rotated_point = rotate_point(point, theta);
    std::cout << "Rotated point: (" << rotated_point[0] << ", " << rotated_point[1] << ")\n";
    return 0;
}
```

### 2. Translation:
A translation in 2D is represented by a 2D vector:

<br>

$`
T = \begin{pmatrix}
x \\
y
\end{pmatrix}
`$

<br>

C++ code example:
```c++
// Function to translate a point
std::vector<double> translate_point(const std::vector<double>& point, double x, double y) {
    double x_new = point[0] + x;
    double y_new = point[1] + y;
    return {x_new, y_new};
}
```

### 3. Combined Transformation:
Combining rotation and translation into a single transformation matrix:

<br>

$`
T = \begin{pmatrix}
\cos(\theta) & -\sin(\theta) & x \\
\sin(\theta) & \cos(\theta) & y \\
0 & 0 & 1
\end{pmatrix}
`$

<br>

The code for this transformation was provided in the earlier C++ example.

### Task and Problem Examples:
1. **Robot Kinematics**: Calculate the position of a robot's end-effector given the joint angles and link lengths. This requires applying a sequence of rotations and translations to find the end-effector's position.

2. **Path Planning**: Given a start and end position, plan a path for a robot to follow. This often involves transforming between different coordinate frames.

3. **Collision Detection**: Check if moving an object along a certain path will result in a collision with other objects. This requires transforming the object's shape along the path and checking for overlaps with other shapes.

4. **Game Development**: Move, rotate, and scale game characters and objects in a 2D game. Transformation matrices are essential for handling the spatial relationships between objects in the game world.

5. **Simulation**: Simulate the motion of objects in a 2D environment, such as cars or drones, by applying sequences of rotations and translations to model their motion over time.

With a firm grasp of rigid-body motions in the plane, you'll be well-equipped to tackle a wide range of problems in robotics, game development, simulation, and more.
