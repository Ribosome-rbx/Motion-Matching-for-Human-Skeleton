#pragma once

// Lib includes
#include "crl-basic/utils/geoms.h"
#include "crl-basic/utils/mathUtils.h"

// STL includes
#include <memory>
#include <string>
#include <vector>

#include "mocap/MocapUtils.h"

namespace crl::mocap {

/** Class created for storing single joint data from bvh file */
class BvhJoint {
public:
    /** A enumeration type useful for set order of channels for every joint */
    enum class Channel { XPOSITION, YPOSITION, ZPOSITION, ZROTATION, XROTATION, YROTATION };

    /** A string names for each channel */
    const std::vector<std::string> channel_name_str = {"XPOSITION", "YPOSITION", "ZPOSITION", "ZROTATION", "XROTATION", "YROTATION"};

    /** Adds single frame motion data
     * and creates local to parent transform
     *
     *  \param  data    The motion data to be added
     */
    void addFrameMotionData(const std::vector<float> &data);

    /** Gets the parent joint of this joint
     *  \return  The parent joint
     */
    std::shared_ptr<BvhJoint> getParent() const {
        return parent_;
    }

    /** Gets the name of this joint
     *  \return  The joint's name
     */
    std::string getName() const {
        return name_;
    }

    /**
     */
    unsigned getIndex() const {
        return index_;
    }

    /** Gets the offset of this joint
     *  \return  The joint's offset
     */
    Eigen::Vector3d getOffset() const {
        return offset_;
    }

    /** Gets the channels order of this joint
     *  \return  The joint's channels order
     */
    std::vector<Channel> getChannelsOrder() const {
        return channels_order_;
    }

    /** Gets the all children joints of this joint
     *  \return  The joint's children
     */
    std::vector<std::shared_ptr<BvhJoint>> getChildren() const {
        return children_;
    }

    /** Gets the channels data of this joint for all frames
     *  \return  The joint's channel data
     */
    const std::vector<std::vector<float>> &getChannelData() const {
        return channel_data_;
    }

    /** Gets the channel data of this joint for selected frame
     *  \param   frame   The frame for which channel data will be returned
     *  \return  The joint's channel data for selected frame
     */
    const std::vector<float> &getChannelDataAtFrame(unsigned frame) const {
        return channel_data_[frame];
    }

    /** Gets the channel data of this joint for selected frame and channel
     *  \param   frame        The frame for which channel data will be returned
     *  \param   channel_num  The number of channel which data will be returned
     *  \return  The joint's channel data for selected frame and channel
     */
    float getChannelDataAtFrame(unsigned frame, unsigned channel_num) const {
        return channel_data_[frame][channel_num];
    }

    /**
     *   Returns true if the joint has an end effector
     */
    bool hasEndEffector() const {
        return hasEef_;
    }

    /** Gets the number of channels of this joint
     *  \return  The joint's channels number
     */
    size_t getNumberOfChannels() const {
        return channels_order_.size();
    }

    /** Sets the this joint parent joint
     *  \param   arg    The parent joint of this joint
     */
    void setParent(const std::shared_ptr<BvhJoint> arg) {
        parent_ = arg;
    }

    /** Sets the this joint name
     *  \param   arg    The name of this joint
     */
    void setName(const std::string arg) {
        name_ = arg;
    }

    /** Set the joint index in the vector of joints
     */
    void setIndex(unsigned index) {
        index_ = index;
    }

    /** Sets the offset of this joint
     *  \param   x    x component of the offset
     *  \param   y    y component of the offset
     *  \param   z    z component of the offset
     */
    void setOffset(double x, double y, double z) {
        offset_[0] = x;
        offset_[1] = y;
        offset_[2] = z;
    }

    /** Sets the this joint offset - overload using Eigen
     *  \param   offset vector (in parent coordinates)
     */
    void setOffset(const Eigen::Vector3d &offset) {
        offset_ = offset;
    }

    /** Sets the this joint channels order
     *  \param   arg    The channels order of this joint
     */
    void setChannelsOrder(const std::vector<Channel> &arg) {
        channels_order_ = arg;
    }

    /** Sets the this joint children
     *  \param   arg    The children of this joint
     */
    void setChildren(const std::vector<std::shared_ptr<BvhJoint>> &children) {
        children_ = children;
    }

    /** Sets the this joint channels data
     *  \param   arg    The channels data of this joint
     */
    void setChannelData(const std::vector<std::vector<float>> &arg) {
        channel_data_ = arg;
    }

    /** Gets channels name of this joint
     *  \return The joint's channels name
     */
    const std::vector<std::string> getChannelsName() const {
        std::vector<std::string> channel_names;

        for (int i = 0; i < channels_order_.size(); i++)
            channel_names.push_back(channel_name_str[static_cast<int>(channels_order_[i])]);

        return channel_names;
    }

    /** 
     * Get the local to world transform for the local coordinates
     * frame of this joint
     */
    Transform_t getLtwTransformAtFrame(unsigned frame) const {
        if (frame > ltwTransforms_.size()) {
            std::cerr << "Joint::getLtwTransformAtFrame - ERROR: transform not "
                         "available!"
                      << std::endl;
        }
        return ltwTransforms_[frame];
    }

    /**
     * Transformation from parent to current joint. (without offset)
     */
    Transform_t getLtpTransformAtFrame(unsigned frame) const {
        if (frame > ltpTransforms_.size()) {
            std::cerr << "Joint::getLtpTransformAtFrame - ERROR: transform not "
                         "available!"
                      << std::endl;
        }
        return ltpTransforms_[frame];
    }

    /** Return a rotation-only affine transform object (Eigen)
     *   \param xRot
     *   \param xRot
     *   \param xRot
     */
    Transform_t getTransformFromMotionData(double zRot, double yRot, double xRot);

    /** Returns a full affine transform object (Eigen)
     *   \param xTrans
     *   \param yTrans
     *   \param zTrans
     *   \param xRot
     *   \param xRot
     *   \param xRot
     */
    Transform_t getTransformFromMotionData(double xTrans, double yTrans, double zTrans, double zRot, double xRot, double yRot);


    /** Get the world position of this joint at the selected frame
     *   \param frame    index of the selected frame
     */
    crl::V3D getJointWorldPosAtFrame(unsigned frame);

    /** Get world position of the given point in the coordinate frame of the
     * joint \param frame    index of the selected frame \param local    point
     * in local coordinates
     */
    crl::V3D getWorldPosAtFrame(unsigned frame, const crl::V3D &local) {
        // TODO: add sanity checks on frame? --> gonna have an out of bounds
        // error anyways Get ltw transform
        Transform_t ltw = ltwTransforms_[frame];

        // Apply to point and return
        // TODO: add debug statement for testing
        crl::V3D world = ltw * local;
        return world * scale_;
    }

    void addEndEffector(const Eigen::Vector3d &eefOffset) {
        hasEef_ = true;
        endEffectors_.push_back(eefOffset);
    }

    const std::vector<crl::V3D> &getEndEffectors() const {
        return endEffectors_;
    }

    bool getRayIntersectionPoint(const crl::Ray &ray, crl::P3D &intersectionPoint, unsigned frameNo);

    /** Selected flag */
    bool selected{false};

    /** If true, when the joints are drawn, if an end effector (if any) is
     * touching the ground (distance from the floor within threshold) a green
     * sphere is draw around it
     */
    bool showContacts{false};

    /** If true, the color of the sphere drawn at each end effector changes
     *   according to its velocity in the world frame
     */
    bool showEefVelocity{true};

    /**
     * If true, the joint belongs to a limb
     */
    bool isLimb{false};
    std::string limbName;

    /**
     * If true when the joints are drawn the coordinate frame is also rendered
     */
    bool showCoordinateFrame{false};

    /**
     * TODO: ditch for another method - using this just for convenience
     */
    bool useForYawAngleEstimation{false};

    void setOffsetScale(double scale) {
        scale_ = scale;
    }

    double getOffsetScale() {
        return scale_;
    }

public:
    // don't draw this joint if this is true
    bool hidden = false;

private:
    /** Offset scaling */
    double scale_ = 1.00;
    /** Joint Index in the vector of joints in the mocap */
    unsigned index_{0};
    /** Parent joint in file hierarchy */
    std::shared_ptr<BvhJoint> parent_;
    /** Joint name in the bvh hierachy */
    std::string name_;
    /** Num channels */
    size_t numChannels{0};
    /** Fixed offset (link) from the parent joint */
    Eigen::Vector3d offset_{0, 0, 0};
    /** Order of joint's input channels */
    std::vector<Channel> channels_order_;
    /** Pointers to joints that are children of this in hierarchy */
    std::vector<std::shared_ptr<BvhJoint>> children_;
    /** Channel data for this joint at each frame
     *  The ith vector contains the channel data of the ith frame.
     */
    std::vector<std::vector<float>> channel_data_;
    /** Local to world (ltw) for this joint for each frame */
    std::vector<Transform_t> ltwTransforms_;
    /** Local to parent (ltp) transform for each frame */
    std::vector<Transform_t> ltpTransforms_;
    /** True if the joint has an end effector **/
    bool hasEef_{false};
    /** End effectors positions - in local coordinates */
    std::vector<crl::V3D> endEffectors_;
};

typedef std::shared_ptr<BvhJoint> JointPtr;

}  // namespace crl::mocap