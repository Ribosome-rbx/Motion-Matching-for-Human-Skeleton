#pragma once

#include "mocap/MocapMarkers.h"
#include "mocap/MocapSkeleton.h"
#include "mocap/MocapSkeletonState.h"

namespace crl::mocap {

/**
 * this corresponds to one mocap (bvh) file.
 */
template <typename MocapModelType>
class MocapClip {
public:
    using StateType = typename MocapModelType::MocapModelStateType;

protected:
    std::string name_;
    double frameTimeStep_ = 1.0 / 60;
    uint frameCount_ = 0;

    // at the moment, we just assume there's only one skeleton in mocap.
    MocapModelType *model_ = nullptr;

    // 1 frame = 1 motion
    std::vector<StateType> motions_;

public:
    virtual ~MocapClip() {
        delete model_;
    }

    bool operator<(const MocapClip &other) const {
        return this->name_ < other.name_;
    }

    MocapModelType *getModel() {
        return model_;
    }

    const StateType &getState(uint frameIdx) const {
        return motions_[frameIdx];
    }

    uint getFrameCount() const {
        return frameCount_;
    }

    std::string getName() const {
        return name_;
    }

    double getFrameTimeStep() const {
        return frameTimeStep_;
    }

    void draw(const gui::Shader &shader, uint frameIdx, float alpha = 1.0) {
        if (frameIdx >= frameCount_) {
            Logger::consolePrint(V3D(1, 0, 0), "Wrong frame index %d > total frame count in clip = %d\n", frameIdx, frameCount_);
            frameIdx %= frameCount_;
        }

        // set skeleton
        if (model_) {
            model_->setState(&getState(frameIdx));
            model_->draw(shader, alpha);
        }
    }
};

class BVHClip : public MocapClip<MocapSkeleton> {
public:
    explicit BVHClip(const std::filesystem::path &filePath) {
        loadFromFile(filePath);
    }

    ~BVHClip() override = default;

private:
    void loadFromFile(const std::filesystem::path &mocapPath) {
        if (mocapPath.extension() == ".bvh")
            loadFromBVH(mocapPath);
        else
            throw std::runtime_error("Not a supported file format: " + mocapPath.extension().string());
    }

    void loadFromBVH(const std::filesystem::path &mocapPath) {
        BvhParser parser;
        Bvh data;
        if (parser.parse(mocapPath.string(), &data)) {
            throw std::runtime_error("Cannot parse bvh file: " + mocapPath.string());
        }

        // load skeleton
        model_ = new MocapSkeleton(mocapPath.string().c_str());

        // load motion
        auto bvhJoints = data.getJoints();
        double dt = data.getFrameTime();

        name_ = mocapPath.filename().string();
        frameTimeStep_ = dt;

        // save frame into motion
        // need to start index = 1 to compute velocity by FD
        motions_.reserve(data.getFramesCount());
        for (uint i = 1; i < data.getFramesCount(); i++) {
            auto bvhRoot = data.getRootJoint();

            auto previousT = bvhRoot->getLtwTransformAtFrame(i - 1);
            auto currentT = bvhRoot->getLtwTransformAtFrame(i);

            motions_.emplace_back(model_);

            // root state
            V3D rootVel = V3D(currentT.translation() - previousT.translation()) / dt;
            V3D rootAngVel = estimateAngularVelocity(Quaternion(previousT.rotation()).normalized(), Quaternion(currentT.rotation()).normalized(), dt);

            motions_.back().setRootPosition(P3D() + currentT.translation());
            motions_.back().setRootOrientation(Quaternion(currentT.rotation()).normalized());
            motions_.back().setRootVelocity(rootVel);
            motions_.back().setRootAngularVelocity(rootAngVel);

            // joint state
            for (uint j = 0; j < bvhJoints.size(); j++) {
                auto previousLtp = bvhJoints[j]->getLtpTransformAtFrame(i - 1);
                auto currentLtp = bvhJoints[j]->getLtpTransformAtFrame(i);

                Quaternion qRel = Quaternion(currentLtp.rotation()).normalized();
                V3D tRel = V3D(currentLtp.translation());
                // TODO: is angvelrel correct here?
                V3D angVelRel = estimateAngularVelocity(Quaternion(previousLtp.rotation()).normalized(), Quaternion(currentLtp.rotation()).normalized(), dt);
                V3D velRel = V3D(currentLtp.translation() - previousLtp.translation()) / dt;

                motions_.back().setJointRelativeOrientation(qRel, j);
                motions_.back().setJointTranslation(tRel, j);
                motions_.back().setJointRelativeAngVelocity(angVelRel, j);
                motions_.back().setJointRelativeVelocity(velRel, j);
            }

            // update frame count
            frameCount_++;
        }
    }
};

class C3DClip : public MocapClip<MocapMarkers> {
public:
    explicit C3DClip(const std::string &filePath) {
        loadFromFile(filePath);
    }

    ~C3DClip() override = default;

private:
    void loadFromFile(const std::filesystem::path &mocapPath) {
        if (mocapPath.extension() == ".c3d")
            loadFromC3D(mocapPath);
        else
            throw std::runtime_error("Not a supported file format: " + mocapPath.extension().string());
    }

    void loadFromC3D(const std::filesystem::path &mocapPath) {
        crl::mocap::C3DFile c3dfile;
        bool result = c3dfile.load(mocapPath.string().c_str());
        if (!result) {
            throw std::runtime_error("Cannot load c3d file: " + mocapPath.string());
        }

        // props
        name_ = mocapPath.filename().string();
        frameTimeStep_ = 1.0 / c3dfile.header.video_sampling_rate;

        // unit
        std::string pointUnits = c3dfile.getParamString("POINT:UNITS");
        double unit = 1.0;
        if (pointUnits == "mm")
            unit = 1e-3;

        // markers
        std::vector<std::string> labels = c3dfile.point_label;
        model_ = new MocapMarkers(mocapPath.string().c_str());
        // note. first_frame is usually 1 and last_frame is N frame.
        for (int i = c3dfile.header.first_frame; i <= c3dfile.header.last_frame; i++) {
            motions_.emplace_back(model_);
        }

        for (const auto &label : labels) {
            // marker motion
            const auto &markerTraj = c3dfile.getMarkerTrajectories(label.c_str());

            // we start from the second frame for computing derivative
            for (int i = 0; i < markerTraj.x.size(); i++) {
                // store state
                MocapMarkerState state;

                // pos
                // change to y-up axis
                P3D p(markerTraj.y[i], markerTraj.z[i], markerTraj.x[i]);
                if (markerTraj.x[i] == 0 && markerTraj.y[i] == 0 && markerTraj.z[i] == 0) {
                    // occlusion! (just use previous frame)
                    state.pos = motions_[i - 1].getMarkerState(label).pos;
                } else {
                    state.pos = p * unit;
                }

                // vel
                if (i > 0)
                    state.velocity = V3D(motions_[i - 1].getMarkerState(label).pos, state.pos) / frameTimeStep_;
                motions_[i].setMarkerState(state, label);
            }
        }

        // remove the very first motion (invalid velocity)
        motions_.erase(motions_.begin());

        // update frame count
        frameCount_ = motions_.size();
    }
};

}  // namespace crl::mocap
