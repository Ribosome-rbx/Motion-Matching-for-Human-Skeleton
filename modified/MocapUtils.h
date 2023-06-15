#pragma once

// Lib includes
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

// STL includes
#include <cmath>
#include <limits>

// CRL includes
#include <crl-basic/utils/mathUtils.h>

namespace crl::mocap {

typedef Eigen::Transform<double, 3, Eigen::Affine> Transform_t;

/** Enumeration class for axis */
enum class Axis { X, Y, Z };

/** Creates rotation matrix
 *  @param  angle  The rotation angle
 *  @param  axis   The rotation axis
 *  @return  The rotation matrix
 */
glm::mat4 rotation_matrix(float angle, Axis axis);

/** Rotates matrix
 *  @param  matrix  The matrix to be rotated
 *  @param  angle   The rotation angle
 *  @param  axis    The rotation axis
 *  @return  The rotation matrix
 */
glm::mat4 rotate(glm::mat4 matrix, float angle, Axis axis);

/** Translates matrix
 *  @param  matrix  The matrix to be rotated
 *  @param  translation   The translation vector
 *  @return  The translated matrix
 */
glm::mat4 translate(glm::mat4 matrix, glm::vec3 translation);

/** Converts matrix to string
 *  @param  matrix  The matrix to be converted
 *  @return  The created string
 */
std::string mat4tos(const glm::mat4 &matrix);

/** Converts vector to string
 *  @param  vector  The vector to be converted
 *  @return  The created string
 */
std::string vec3tos(const glm::vec3 &vector);

/** Return a rotation-only affine transform object (Eigen)
 *   \param xRot
 *   \param xRot
 *   \param xRot
 */
// Transform_t getTransformFromMotionData(double zRot, double xRot, double yRot);

/** Returns a full affine transform object (Eigen)
 *   \param xTrans
 *   \param yTrans
 *   \param zTrans
 *   \param xRot
 *   \param xRot
 *   \param xRot
 */
// Transform_t getTransformFromMotionData(double xTrans, double yTrans, double zTrans, double zRot, double xRot, double yRot);

/**
 * Returns the angular velocity that explains how we got from qStart to qEnd in
 * dt time (qEnd = qDueToAngVelOverDT * qStart)
 */
V3D estimateAngularVelocity(const Quaternion &qStart, const Quaternion &qEnd, double dt);

}  // namespace crl::mocap