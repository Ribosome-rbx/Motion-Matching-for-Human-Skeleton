//
// Created by Dongho Kang on 05.12.21.
//
#include "mocap/MocapUtils.h"

namespace crl::mocap {

glm::mat4 rotation_matrix(float angle, Axis axis) {
    glm::mat4 matrix(1.0);  // identity matrix
    float rangle = glm::radians(angle);
    // We want to unique situation when in matrix are -0.0f, so we perform
    // additional checking
    float sin_a = glm::sin(rangle);
    if (fabs(sin_a) < std::numeric_limits<float>::epsilon())
        sin_a = 0.0f;
    float cos_a = glm::cos(rangle);
    if (fabs(cos_a) < std::numeric_limits<float>::epsilon())
        cos_a = 0.0f;
    float msin_a = fabs(sin_a) < std::numeric_limits<float>::epsilon() ? 0.0f : (-1.0f) * sin_a;

    if (axis == Axis::X) {
        ((float *)glm::value_ptr(matrix))[5] = cos_a;
        ((float *)glm::value_ptr(matrix))[6] = sin_a;
        ((float *)glm::value_ptr(matrix))[9] = msin_a;
        ((float *)glm::value_ptr(matrix))[10] = cos_a;
    } else if (axis == Axis::Y) {
        ((float *)glm::value_ptr(matrix))[0] = cos_a;
        ((float *)glm::value_ptr(matrix))[2] = msin_a;
        ((float *)glm::value_ptr(matrix))[8] = sin_a;
        ((float *)glm::value_ptr(matrix))[10] = cos_a;
    } else {
        ((float *)glm::value_ptr(matrix))[0] = cos_a;
        ((float *)glm::value_ptr(matrix))[1] = sin_a;
        ((float *)glm::value_ptr(matrix))[4] = msin_a;
        ((float *)glm::value_ptr(matrix))[5] = cos_a;
    }

    return matrix;
}

glm::mat4 rotate(glm::mat4 matrix, float angle, Axis axis) {
    return matrix * rotation_matrix(angle, axis);
}

glm::mat4 translate(glm::mat4 matrix, glm::vec3 translation) {
    ((float *)glm::value_ptr(matrix))[12] += translation.x;
    ((float *)glm::value_ptr(matrix))[13] += translation.y;
    ((float *)glm::value_ptr(matrix))[14] += translation.z;
    return matrix;
}

std::string mat4tos(const glm::mat4 &matrix) {
    std::string result;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++)
            result += std::to_string(((float *)glm::value_ptr(matrix))[4 * j + i]) + ", ";

        result += "\n";
    }
    return result;
}

std::string vec3tos(const glm::vec3 &vector) {
    std::string result;
    for (int i = 0; i < 3; i++) {
        result += std::to_string(((float *)glm::value_ptr(vector))[i]);
        if (i != 2)
            result += ", ";
    }

    result += "\n";

    return result;
}

// Transform_t getTransformFromMotionData(double zRot, double xRot, double yRot) {
//     // Create rotation quaternion from Euler angles
//     Eigen::Quaternion q = crl::getRotationQuaternion(RAD(zRot), crl::V3D(0, 0, 1)) *  //
//                         //   crl::getRotationQuaternion(RAD(xRot), crl::V3D(1, 0, 0)) *  //
//                         //   crl::getRotationQuaternion(RAD(yRot), crl::V3D(0, 1, 0));
//                              crl::getRotationQuaternion(RAD(yRot), crl::V3D(0, 1, 0)) *  //
//                              crl::getRotationQuaternion(RAD(xRot), crl::V3D(1, 0, 0));
//     // Create transform
//     Transform_t t(q);

//     return t;
// }

// Transform_t getTransformFromMotionData(double xTrans, double yTrans, double zTrans, double zRot, double xRot, double yRot) {
//     Transform_t translation;
//     translation = Eigen::Translation3d(xTrans, yTrans, zTrans);
//     Transform_t rotation;
//     rotation = getTransformFromMotionData(zRot, xRot, yRot);

//     // First rotate then translate
//     return translation * rotation;
// }

V3D estimateAngularVelocity(const Quaternion &qStart, const Quaternion &qEnd, double dt) {
    // qEnd = rot(w_p, dt) * qStart
    Quaternion qRot = qEnd * qStart.inverse();

    V3D rotAxis = Vector3d(qRot.vec());
    if (rotAxis.norm() < 1e-10)
        return V3D(0, 0, 0);
    rotAxis.normalize();
    double rotAngle = getRotationAngle(qRot, rotAxis);

    // this rotation angle is the result of applying the angular velocity for
    // some time dt...
    return rotAxis * rotAngle / dt;
}

}