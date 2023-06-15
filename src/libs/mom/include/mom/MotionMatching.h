#pragma once

#include <deque>

#include "crl-basic/gui/plots.h"
#include "crl-basic/utils/trajectory.h"
#include "mom/MotionDatabase.h"

namespace crl::mocap {

/**
 * inertialization parameters.
 */
struct InertializationInfo {
    V3D x0v = V3D(0, 0, 0);
    double x0 = 0, v0 = 0, a0 = 0;
    double t1 = 0;               // when inertialization finishes
    double A = 0, B = 0, C = 0;  // coefficients
};

/**
 * Implementation of motion matching and inertialization blending.
 *
 * References:
 * [1] D. Holden, et al., "Learned Motion Matching", 2020.
 * [2] D. Bollo, "Inertialization: High-Performance Animation Transitions in’Gears of War’.", 2016.
 *
 * TODO:
 * - check query trajectory index 20/40/60 (should it be 21/41/61?)
 *
 * Done:
 * - double check frame index after motion matching and stitching. remember, we overlap very last frame of old frame with very first frame of new frame.
 */
class MotionMatching {
public:
    // keyboard control
    bool KEY_W = false;
    bool KEY_A = false;
    bool KEY_S = false;
    bool KEY_D = false;
    bool KEY_J = false;
    bool KEY_C = false;
    bool KEY_P = false; // dance

    // commands
    double speedForward = 0;
    double speedSideways = 0;
    double turningSpeed = 0;

    // threshold for swing/contact
    double feetHeightThreshold = 0.055;
    double feetSpeedThreshold = 0.8;

    // transition time t1
    // note. actual transition time is min(t1, -5 * x0 / v0)
    double transitionTime = 0.4;

    // option
    bool inertialization = true;
    // if queueSize = 0, we just inertialize motion as post-process
    // if queueSize > 0, we save 0, 1, ..., queueSize (so in total queueSize+1) frames
    uint queueSize = 60;

    // extracted speed profile
    Trajectory3D speedProfile;
    Trajectory1D heightProfile;

    // camera direction rotate
    std::list<V3D> cameraDirections;

    // customized draw
    V3D goalVel, cameraDir;
    Trajectory3D PosTraj;
    Trajectory3D VelTraj;
    std::vector<P3D> historyPos;
    std::vector<V3D> historyVel;
    std::vector<P3D> paintTraj;
    int trajInd = 0;

    // if this is empty, we extract height and speed of root
    std::string referenceJointName = "";  // Spine1

    bool shouldStop_ = false;
    bool isDance_ = false;
    bool forceMotionMatching_ = false;

private:
    MocapSkeleton *skeleton_ = nullptr;
    MotionDatabase *database_ = nullptr;
    MotionDatabase *oldDatabase_ = nullptr;
    MotionDatabase *lastMatchedDatabaseTmp_ = nullptr;
    MotionDatabase *jumpDatabase_ = nullptr;
    MotionDatabase *walkDatabase_ = nullptr;
    MotionDatabase *stopDatabase_ = nullptr;
    MotionDatabase *danceDatabase_ = nullptr;

    // time related
    double motionTime_ = 0;
    double dt_ = 1.0 / 60;  // this is timestep of one frame (updated from DB)

    // inertialization info
    InertializationInfo rootPosInertializationInfo_;
    InertializationInfo rootQInertializationInfo_;
    std::vector<InertializationInfo> jointInertializationInfos_;

    // pose of character (updated when motion matching happens)
    P3D t_wc = P3D(0, 0, 0);
    Quaternion Q_wc = Quaternion::Identity();

    // pose of new motion at t=0
    P3D t_wt0 = P3D(0, 0, 0);
    Quaternion Q_wt0 = Quaternion::Identity();

    // pose of new motion at t=0 after stitching
    P3D t_wt0prime = P3D(0, 0, 0);
    Quaternion Q_wt0prime = Quaternion::Identity();

    // motion idx
    MotionIndex currMotionIdx_ = {-1, -1};
    MotionIndex lastMotionIdx_ = {-1, -1};

    // queue
    // we want to compute initialized motion at once (when motion matching happens) and extracting velocity and foot contact profile.
    std::deque<MocapSkeletonState> queue_;

    // plot
    crl::gui::RealTimeLinePlot2D<crl::V3D> characterSpeedPlots_;
    crl::gui::RealTimeLinePlot2D<crl::V3D> speedProfilePlots_;
    crl::gui::RealTimeLinePlot2D<crl::dVector> eeSpeedPlots_;

public:
    MotionMatching(MocapSkeleton *skeleton, MotionDatabase *walkDatabase, MotionDatabase * jumpDatabase, MotionDatabase * stopDatabase, MotionDatabase * danceDatabase)
        : characterSpeedPlots_("Character Speed", "[sec]", "[m/s] or [rad/s]"),
          speedProfilePlots_("Speed Profile", "[sec]", "[m/s] or [rad/s]"),
          eeSpeedPlots_("Feet Speed", "[sec]", "[m/s]") {
        this->skeleton_ = skeleton;
        this->walkDatabase_ = walkDatabase;
        this->jumpDatabase_ = jumpDatabase;
        this->stopDatabase_ = stopDatabase;
        this->danceDatabase_ = danceDatabase;
        this->database_ = this->walkDatabase_;
        this->oldDatabase_ = this->walkDatabase_;
        this->lastMatchedDatabaseTmp_ = this->walkDatabase_;
        jointInertializationInfos_.resize(skeleton->getMarkerCount());
        this->shouldStop_ = false;
        // this->isCartoonOn_ = false;
        // set initial motion
        resetMotion();

        // plots
        characterSpeedPlots_.addLineSpec({"forward", [](const auto &d) { return (float)d.x(); }});
        characterSpeedPlots_.addLineSpec({"sideways", [](const auto &d) { return (float)d.y(); }});
        characterSpeedPlots_.addLineSpec({"turning", [](const auto &d) { return (float)d.z(); }});
        speedProfilePlots_.addLineSpec({"forward", [](const auto &d) { return (float)d.x(); }});
        speedProfilePlots_.addLineSpec({"sideways", [](const auto &d) { return (float)d.y(); }});
        speedProfilePlots_.addLineSpec({"turning", [](const auto &d) { return (float)d.z(); }});
        // eeSpeedPlots_.addLineSpec({"fl", [](const auto &d) { return (float)d[0]; }});
        eeSpeedPlots_.addLineSpec({"hl", [](const auto &d) { return (float)d[1]; }});
        // eeSpeedPlots_.addLineSpec({"fr", [](const auto &d) { return (float)d[2]; }});
        eeSpeedPlots_.addLineSpec({"hr", [](const auto &d) { return (float)d[3]; }});
    }

    ~MotionMatching() = default;

    double getFrameTimeStep() {
        return dt_;
    }

    /**
     * reset motion (restart from the origin)
     */
    void resetMotion(std::string initDataset = "walk1_subject1.bvh", uint initFrame = 0) {
        this->shouldStop_ = false;
        this->database_ = this->walkDatabase_;
        this->oldDatabase_ = this->walkDatabase_;
        this->lastMatchedDatabaseTmp_ = this->walkDatabase_;
        t_wc = P3D(0, 0, 0);
        Q_wc = Quaternion::Identity();

        // pose of new motion at t=0
        t_wt0 = P3D(0, 0, 0);
        Q_wt0 = Quaternion::Identity();

        // pose of new motion at t=0 after stitching
        t_wt0prime = P3D(0, 0, 0);
        Q_wt0prime = Quaternion::Identity();

        // set initial motion
        currMotionIdx_ = lastMotionIdx_ = {0, 0};
        for (uint i = 0; i < this->database_->getClipCount(); i++) {
            if (database_->getClipByClipIndex(i)->getName() == initDataset && database_->getClipByClipIndex(i)->getFrameCount() - 60 > initFrame) {
                currMotionIdx_ = lastMotionIdx_ = {i, initFrame};
            }
        }
        skeleton_->setState(&database_->getMotionByMotionIndex(currMotionIdx_));

        // and reset inertialization infos
        rootPosInertializationInfo_ = InertializationInfo();
        rootQInertializationInfo_ = InertializationInfo();
        for (uint i = 0; i < jointInertializationInfos_.size(); i++) {
            jointInertializationInfos_[i] = InertializationInfo();
        }

        // reset queue
        queue_.clear();

        // reset foot fall pattern
        motionTime_ = 0;

        // reset plots
        characterSpeedPlots_.clearData();
        speedProfilePlots_.clearData();
        eeSpeedPlots_.clearData();

        // now fill up queue from starting
        if (queueSize > 0) {
            saveFutureMotionIntoQueue();
        }
    }

    void switchDatabase(){
        if (KEY_J || KEY_C){
            this->oldDatabase_ = this->lastMatchedDatabaseTmp_;
            this->database_ = this->jumpDatabase_;
        }
        else if (this->shouldStop_)
        {
            this->oldDatabase_ = this->lastMatchedDatabaseTmp_;
            this->database_ = this->stopDatabase_;
        }
        else if (this->isDance_)
        {
            this->oldDatabase_ = this->lastMatchedDatabaseTmp_;
            this->database_ = this->danceDatabase_;
        }    
        else{
            this->oldDatabase_ = this->lastMatchedDatabaseTmp_;
            this->database_ = this->walkDatabase_;
        } 
    }

    /**
     * compute inertialization and update. here, old motion is current motion
     * sequence and new motion is the best match searched with a query vector.
     */
    void matchMotion(const crl::gui::TrackingCamera &camera) {
        dVector xq;
        MotionIndex bestMatch;
        if(KEY_J || KEY_C) xq = createQueryVector3d(camera);
        else xq = createQueryVector(camera);
        if (this->shouldStop_)
        {
            switchDatabase();
            bestMatch = {0, 0};
        }
        else
        {
            bestMatch = database_->searchBestMatchByQuery(xq);
        }

        Logger::consolePrint("Best match: (%d, %d: %s) -> (%d, %d: %s) \n",  //
                             currMotionIdx_.first, currMotionIdx_.second - 1, oldDatabase_->getClipByClipIndex(currMotionIdx_.first)->getName().c_str(),
                             bestMatch.first, bestMatch.second, database_->getClipByClipIndex(bestMatch.first)->getName().c_str());
        // here we need to match with previous frame!
        matchMotion({currMotionIdx_.first, currMotionIdx_.second - 1}, bestMatch);
        lastMatchedDatabaseTmp_ = this->database_;
    }

    /**
     * compute inertialization and update (for given motions).
     */
    void matchMotion(MotionIndex oldMotionIdx, MotionIndex newMotionIdx) {
        // old motion stitched
        MocapSkeletonState oldMotionMinus1 = computeStitchedMotion(oldDatabase_->getMotionByMotionIndex({oldMotionIdx.first, oldMotionIdx.second - 1}));
        MocapSkeletonState oldMotion = computeStitchedMotion(oldDatabase_->getMotionByMotionIndex(oldMotionIdx));

        // update Q_wc and t_wc
        t_wc = P3D(skeleton_->root->state.pos.x, 0, skeleton_->root->state.pos.z);
        Q_wc = calc_facing_quat(skeleton_->root->state.orientation);

        // update Q_wt0 and t_wt0
        MocapSkeletonState newMotion = database_->getMotionByMotionIndex(newMotionIdx);
        Q_wt0 = newMotion.getRootOrientation();
        t_wt0 = newMotion.getRootPosition();

        // // update Q_wt0prime and t_wt0prime
        // // double roll, pitch, yaw;
        // // computeEulerAnglesFromQuaternion(Q_wt0,  //
        // //                                  skeleton_->forwardAxis, skeleton_->forwardAxis.cross(skeleton_->upAxis), skeleton_->upAxis, roll, pitch, yaw);
        Q_wt0prime =  Q_wc * calc_facing_quat(Q_wt0).inverse() * Q_wt0;
        // Q_wt0prime = Q_wc * getRotationQuaternion(pitch, skeleton_->forwardAxis.cross(skeleton_->upAxis)) * getRotationQuaternion(roll, skeleton_->forwardAxis);
        t_wt0prime = t_wc + V3D(0, t_wt0.y, 0);

        // compute inertialization
        newMotion.setRootOrientation(Q_wt0prime);
        newMotion.setRootPosition(t_wt0prime);
        computeInertialization(oldMotionMinus1, oldMotion, newMotion, transitionTime, dt_);

        // update motion idx
        lastMotionIdx_ = newMotionIdx;
        currMotionIdx_ = newMotionIdx;
        currMotionIdx_.second++;  // we play from next frame

        // clear queue first
        queue_.clear();

        // save future motions into queue if queueSize > 0
        if (queueSize > 0) {
            saveFutureMotionIntoQueue();
        }
    }

    /**
     * advance one frame
     */
    void advance() {
        if (queue_.size() > 0) {
            // if queue is not empty, play saved motions
            // remember! queue.front() is next frame!
            skeleton_->setState(&queue_.front());
            // if (queue_.size() == 0 && this->isDance_)
            if (queue_.size() == 1 && this->shouldStop_)
            {
                // this->switchDatabase();
                currMotionIdx_.second--;
            }
            else if (queue_.size() == 1 && this->isDance_)
            {
                this->forceMotionMatching_ = true;
            }
            else
            {
                queue_.pop_front();
            }
            
        } else {
            // if not, we need to compute new motion
            MocapSkeletonState newMotion_t = computeStitchedMotion(database_->getMotionByMotionIndex(currMotionIdx_));

            if (inertialization) {
                if (currMotionIdx_.first != lastMotionIdx_.first)
                    // well this will never happen but just in case...
                    throw std::runtime_error(
                        "MotionMatcher::advance() error: something wrong in "
                        "index. (current motion database = " +
                        std::to_string(currMotionIdx_.first) + " != " + "matched motion database = " + std::to_string(lastMotionIdx_.first) + ")");

                double t = (currMotionIdx_.second - lastMotionIdx_.second) * dt_;

                // here tMinus1 motion is current state of skeleton
                MocapSkeletonState newMotion_tMinus1(skeleton_);
                auto inertializedMotion = inertializeState(newMotion_t, newMotion_tMinus1, t, dt_);
                skeleton_->setState(&inertializedMotion);
            } else {
                skeleton_->setState(&newMotion_t);
            }
        }

        // plot
        {
            Quaternion characterQ =
                calc_facing_quat(skeleton_->root->state.orientation);

            V3D characterVel = skeleton_->root->state.velocity;
            characterVel = characterQ.inverse() * characterVel;
            V3D characterAngularVel = skeleton_->root->state.angularVelocity;
            characterAngularVel = characterQ.inverse() * characterAngularVel;

            V3D speed;
            speed.x() = characterVel.dot(skeleton_->forwardAxis);
            speed.y() = characterVel.dot(skeleton_->forwardAxis.cross(skeleton_->upAxis));
            speed.z() = characterAngularVel.dot(skeleton_->upAxis);
            characterSpeedPlots_.addData(motionTime_, speed);
        }

        // increase index
        currMotionIdx_.second++;
        motionTime_ += dt_;
    }

    void drawDebugInfo(const gui::Shader &shader, const crl::gui::TrackingCamera &camera) {
        // draw query info
        {
            // future trajectory
            dVector xq = createQueryVector(camera);
            P3D characterPos = P3D(skeleton_->root->state.pos.x, 0, skeleton_->root->state.pos.z);
            Quaternion characterQ = calc_facing_quat(skeleton_->root->state.orientation);
            V3D tt1(xq[0], 0, xq[1]);
            tt1 = characterQ * tt1;
            V3D tt2(xq[2], 0, xq[3]);
            tt2 = characterQ * tt2;
            V3D tt3(xq[4], 0, xq[5]);
            tt3 = characterQ * tt3;
            drawSphere(characterPos + tt1, 0.02, shader, V3D(0, 0, 0), 0.5);
            drawSphere(characterPos + tt2, 0.02, shader, V3D(0, 0, 0), 0.5);
            drawSphere(characterPos + tt3, 0.02, shader, V3D(0, 0, 0), 0.5);

            // future heading
            V3D td1(xq[6], 0, xq[7]);
            td1 = characterQ * td1;
            V3D td2(xq[8], 0, xq[9]);
            td2 = characterQ * td2;
            V3D td3(xq[10], 0, xq[11]);
            td3 = characterQ * td3;
            drawArrow3d(characterPos + tt1, td1 * 0.15, 0.01, shader, V3D(0, 0, 0), 0.5);
            drawArrow3d(characterPos + tt2, td2 * 0.15, 0.01, shader, V3D(0, 0, 0), 0.5);
            drawArrow3d(characterPos + tt3, td3 * 0.15, 0.01, shader, V3D(0, 0, 0), 0.5);

            // foot position
            // V3D ft1(xq[12], xq[13], xq[14]);
            // ft1 = characterQ * ft1;
            V3D ft2(xq[12], xq[13], xq[14]);
            ft2 = characterQ * ft2;
            // V3D ft3(xq[18], xq[19], xq[20]);
            // ft3 = characterQ * ft3;
            V3D ft4(xq[15], xq[16], xq[17]);
            ft4 = characterQ * ft4;

            // foot velocity
            // V3D ft1dot(xq[24], xq[25], xq[26]);
            // ft1dot = characterQ * ft1dot;
            V3D ft2dot(xq[18], xq[19], xq[20]);
            ft2dot = characterQ * ft2dot;
            // V3D ft3dot(xq[30], xq[31], xq[32]);
            // ft3dot = characterQ * ft3dot;
            V3D ft4dot(xq[21], xq[22], xq[23]);
            ft4dot = characterQ * ft4dot;

            // contact
            // V3D f1color(1, 1, 0);
            V3D f2color(1, 1, 0);
            // V3D f3color(1, 1, 0);
            V3D f4color(1, 1, 0);
            // if (ft1.y() < feetHeightThreshold && ft1dot.norm() < feetSpeedThreshold)
            //     f1color = V3D(0, 1, 1);
            if (ft2.y() < feetHeightThreshold && ft2dot.norm() < feetSpeedThreshold)
                f2color = V3D(0, 1, 1);
            // if (ft3.y() < feetHeightThreshold && ft3dot.norm() < feetSpeedThreshold)
            //     f3color = V3D(0, 1, 1);
            if (ft4.y() < feetHeightThreshold && ft4dot.norm() < feetSpeedThreshold)
                f4color = V3D(0, 1, 1);

            // drawSphere(characterPos + ft1, 0.02, shader, f1color, 0.5);
            drawSphere(characterPos + ft2, 0.02, shader, f2color, 0.5);
            // drawSphere(characterPos + ft3, 0.02, shader, f3color, 0.5);
            drawSphere(characterPos + ft4, 0.02, shader, f4color, 0.5);

            // drawArrow3d(characterPos + ft1, ft1dot * 0.15, 0.01, shader, V3D(1, 0.55, 0), 0.5);
            drawArrow3d(characterPos + ft2, ft2dot * 0.15, 0.01, shader, V3D(1, 0.55, 0), 0.5);
            // drawArrow3d(characterPos + ft3, ft3dot * 0.15, 0.01, shader, V3D(1, 0.55, 0), 0.5);
            drawArrow3d(characterPos + ft4, ft4dot * 0.15, 0.01, shader, V3D(1, 0.55, 0), 0.5);

            // root velocity
            // let's just draw from character pos (for projecting to ground)
            V3D htdot(xq[24], xq[25], xq[26]);
            htdot = characterQ * htdot;
            drawSphere(characterPos, 0.02, shader, V3D(1, 0, 1), 0.5);
            drawArrow3d(characterPos, htdot * 0.15, 0.01, shader, V3D(1, 0, 1), 0.5);
        }

        // draw best match info (currently playing sequence)
        {
            const auto &currentClip = database_->getClipByClipIndex(currMotionIdx_.first);
            for (uint i = currMotionIdx_.second + 20; i <= currMotionIdx_.second + 60; i += 20) {
                if (i >= currentClip->getFrameCount())
                    break;

                // future motion
                auto motionAfter_i = computeStitchedMotion(currentClip->getState(i));

                // pos
                P3D pos = motionAfter_i.getRootPosition();
                pos.y = 0;
                drawSphere(pos, 0.02, shader, V3D(1, 0, 0), 0.5);

                // heading
                Quaternion q = motionAfter_i.getRootOrientation();
                q = calc_facing_quat(q);

                V3D heading = q * skeleton_->forwardAxis;
                heading.y() = 0;
                heading.normalize();

                drawArrow3d(pos, heading * 0.15, 0.01, shader, V3D(1, 0, 0), 0.5);
            }
        }
    }

    void plotDebugInfo() {
        characterSpeedPlots_.draw();
        speedProfilePlots_.draw();
        eeSpeedPlots_.draw();
    }

    bool setPropertoDance()
    {
        this->KEY_A = false;
        this->KEY_D = false;
        this->KEY_S = false;
        this->KEY_W = false;
        this->KEY_J = false;
        this->KEY_C = false;
        this->shouldStop_ = false;
        return true;
    }

    void cameraRotation(crl::gui::TrackingCamera &camera){
        V3D velocity = skeleton_->root->state.velocity;
        velocity[1] = 0;
        velocity = velocity.normalized();

        V3D old_velocity = V3D(P3D(camera.direction[0], 0, camera.direction[2])).normalized();

        if(old_velocity.dot(velocity) < 0.9){
            cameraDirections.clear();
            // spring damper to rotate camera smoothly
            double dtTraj = 1.0 / 60;  // dt
            double t = 0;
            double halflife = 0.5f;
            
            while (t <= 1.0) {
                V3D goal_vel = velocity; // V0 in the world frame
                V3D curr_vel = old_velocity; // V0 in the world frame
                P3D curr_pos = P3D(0,0,0); // random init
                V3D init_a = goal_vel - curr_vel; // initial acceleration in the world frame
                
                // compute future camera pose
                spring_character_update(curr_pos[0], curr_vel[0], init_a[0], goal_vel[0], halflife, t);
                spring_character_update(curr_pos[2], curr_vel[2], init_a[2], goal_vel[2], halflife, t);

                // store trajectory at time t
                cameraDirections.push_back(curr_vel);

                t += dtTraj;
            }
            cameraDirections.pop_front();
        }
        if(cameraDirections.size() > 0){
            V3D cur_velocity = cameraDirections.front();
            cameraDirections.pop_front();
            camera.direction[0] = cur_velocity[0];
            camera.direction[1] = 0;
            camera.direction[2] = cur_velocity[2];
        }
    }

private:
    /**
     * Compute inertialization infos (coefficients, x0, v0, a0, t1, etc.)
     * 
     * t1 is user's desired transition time (t1 = 0.5 is desirable...)
     * dt is one frame timestep
     */
    void computeInertialization(const MocapSkeletonState &oldMinus1Motion, const MocapSkeletonState &oldMotion, const MocapSkeletonState &newMotion, double t1,
                                double dt) {
        // root
        {
            // position
            V3D oldMinus1Pos = V3D(oldMinus1Motion.getRootPosition());
            V3D oldPos = V3D(oldMotion.getRootPosition());
            V3D newPos = V3D(newMotion.getRootPosition());

            rootPosInertializationInfo_ = computeInertialization(oldMinus1Pos, oldPos, newPos, dt, t1);

            // orientation
            Quaternion oldMinus1Q = oldMinus1Motion.getRootOrientation();
            Quaternion oldQ = oldMotion.getRootOrientation();
            Quaternion newQ = newMotion.getRootOrientation();

            rootQInertializationInfo_ = computeInertialization(oldMinus1Q, oldQ, newQ, dt, t1);
        }

        // joints
        for (uint i = 0; i < skeleton_->getMarkerCount(); i++) {
            Quaternion oldMinus1Q = oldMinus1Motion.getJointRelativeOrientation(i);
            Quaternion oldQ = oldMotion.getJointRelativeOrientation(i);
            Quaternion newQ = newMotion.getJointRelativeOrientation(i);

            jointInertializationInfos_[i] = computeInertialization(oldMinus1Q, oldQ, newQ, dt, t1);
        }
    }

    /**
     * initialize state at time t based on computed inertialization infos.
     * 
     * motion_t is a motion from new sequence at time t
     * t is a time since the last motion matching happened
     * dt is a frame timestep (for computing velocity by finite difference)
     */
    MocapSkeletonState inertializeState(const MocapSkeletonState &motion_t, const MocapSkeletonState &motion_tMinus1, double t, double dt) {
        // we will return inertializedMotion_t
        MocapSkeletonState inertializedMotion_t = motion_tMinus1;

        // root
        {
            // position
            V3D rootPosV = V3D(motion_t.getRootPosition());
            P3D pos_tminus1 = inertializedMotion_t.getRootPosition();

            V3D inertializedPosV = inertializeVector(rootPosInertializationInfo_, rootPosV, t);
            inertializedMotion_t.setRootPosition(P3D() + inertializedPosV);
            inertializedMotion_t.setRootVelocity((inertializedPosV - V3D(pos_tminus1)) / dt);

            // orientation
            Quaternion rootQ = motion_t.getRootOrientation();
            Quaternion q_tminus1 = inertializedMotion_t.getRootOrientation();

            Quaternion inertializedQ = inertializeQuaternion(rootQInertializationInfo_, rootQ, t);
            inertializedMotion_t.setRootOrientation(inertializedQ);
            inertializedMotion_t.setRootAngularVelocity(estimateAngularVelocity(q_tminus1, inertializedQ, dt));
        }

        // joints
        for (uint i = 0; i < skeleton_->getMarkerCount(); i++) {
            Quaternion jointRelQ = motion_t.getJointRelativeOrientation(i);
            Quaternion q_tminus1 = inertializedMotion_t.getJointRelativeOrientation(i);
            auto &inertializeInfo = jointInertializationInfos_[i];

            Quaternion inertializedQ = inertializeQuaternion(inertializeInfo, jointRelQ, t);
            inertializedMotion_t.setJointRelativeOrientation(inertializedQ, i);
            inertializedMotion_t.setJointRelativeAngVelocity(estimateAngularVelocity(q_tminus1, inertializedQ, dt), i);
        }

        return inertializedMotion_t;
    }

    InertializationInfo computeInertialization(Quaternion &oldPrevQ, Quaternion &oldQ, Quaternion &newQ, double dt, double t1) {
        Quaternion q0 = oldQ * newQ.inverse();
        Quaternion q_minus1 = oldPrevQ * newQ.inverse();

        // compute x0
        if (q0.vec().norm() < 1e-10) {
            // then we can just say q0 is identity...
            // just play coming sequences as it is
            InertializationInfo info;
            return info;
        }

        // if q0 is not identity
        V3D x0v = q0.vec().normalized();
        double x0 = getRotationAngle(q0, x0v);

        // compute x_minus1
        double x_minus1 = 2 * atan2(q_minus1.vec().dot(x0v), q_minus1.w());

        // compute v0 and a0
        // note.
        // if x0 >= 0 then v0 should be < 0 and a0 should be > 0
        // if x0 < 0 then v0 should be > 0 and a0 should be < 0
        // if this is not the case, just clamp v0 or a0

        // v0 = (x0 - x_minus1) / dt
        double v0 = (x0 - x_minus1) / dt;

        if ((x0 > 0 && v0 < 0) || (x0 < 0 && v0 > 0))
            // t1 = min(t1, -5 * x0/v0)
            t1 = std::min(t1, -5 * x0 / v0);

        // a0 = (-8 * v0 * t1 - 20 * x0) / (t1^2)
        double a0 = (-8 * v0 * t1 - 20 * x0) / (t1 * t1);

        if (x0 > 0) {
            if (v0 > 0)
                v0 = 0;
            if (a0 < 0)
                a0 = 0;
        } else {
            if (v0 < 0)
                v0 = 0;
            if (a0 > 0)
                a0 = 0;
        }

        // A = - (a0 * t1^2 + 6 * v0 * t1 + 12 * x0) / (2 * t1^5)
        double A = -(a0 * t1 * t1 + 6 * v0 * t1 + 12 * x0) / (2 * t1 * t1 * t1 * t1 * t1);
        // B = (3 * a0 * t1^2 + 16 * v0 * t1 + 30 * x0) / (2 * t1^4)
        double B = (3 * a0 * t1 * t1 + 16 * v0 * t1 + 30 * x0) / (2 * t1 * t1 * t1 * t1);
        // C = - (3 * a0 * t1^2 + 12 * v0 * t1 + 20 * x0) / (2 * t1^3)
        double C = -(3 * a0 * t1 * t1 + 12 * v0 * t1 + 20 * x0) / (2 * t1 * t1 * t1);

        InertializationInfo info;
        info.x0v = x0v;
        info.A = A;
        info.B = B;
        info.C = C;
        info.x0 = x0;
        info.v0 = v0;
        info.a0 = a0;
        info.t1 = t1;
        return info;
    }

    InertializationInfo computeInertialization(V3D &oldPrevV3D, V3D &oldV3D, V3D &newV3D, double dt, double t1) {
        V3D x0v = oldV3D - newV3D;
        V3D x_minus1v = oldPrevV3D - newV3D;

        double x0 = x0v.norm();
        if (x0 < 1e-10) {
            // can be consider there's no change at all
            InertializationInfo info;
            return info;
        }

        // compute x_minus1
        double x_minus1 = x_minus1v.dot(x0v.normalized());

        // compute v0 and a0
        // note.
        // if x0 >= 0 then v0 should be < 0 and a0 should be > 0
        // if x0 < 0 then v0 should be > 0 and a0 should be < 0
        // if this is not the case, just clamp v0 or a0

        // v0 = (x0 - x_minus1) / dt
        double v0 = (x0 - x_minus1) / dt;

        if ((x0 > 0 && v0 < 0) || (x0 < 0 && v0 > 0))
            // t1 = min(t1, -5 * x0/v0)
            t1 = std::min(t1, -5 * x0 / v0);

        // a0 = (-8 * v0 * t1 - 20 * x0) / (t1^2)
        double a0 = (-8 * v0 * t1 - 20 * x0) / (t1 * t1);

        if (x0 > 0) {
            if (v0 > 0)
                v0 = 0;
            if (a0 < 0)
                a0 = 0;
        } else {
            if (v0 < 0)
                v0 = 0;
            if (a0 > 0)
                a0 = 0;
        }

        // A = - (a0 * t1^2 + 6 * v0 * t1 + 12 * x0) / (2 * t1^5)
        double A = -(a0 * t1 * t1 + 6 * v0 * t1 + 12 * x0) / (2 * t1 * t1 * t1 * t1 * t1);
        // B = (3 * a0 * t1^2 + 16 * v0 * t1 + 30 * x0) / (2 * t1^4)
        double B = (3 * a0 * t1 * t1 + 16 * v0 * t1 + 30 * x0) / (2 * t1 * t1 * t1 * t1);
        // C = - (3 * a0 * t1^2 + 12 * v0 * t1 + 20 * x0) / (2 * t1^3)
        double C = -(3 * a0 * t1 * t1 + 12 * v0 * t1 + 20 * x0) / (2 * t1 * t1 * t1);

        InertializationInfo info;
        info.x0v = x0v;
        info.A = A;
        info.B = B;
        info.C = C;
        info.x0 = x0;
        info.v0 = v0;
        info.a0 = a0;
        info.t1 = t1;
        return info;
    }

    /**
     * inertialize quaternion newQ_t at time t.
     */
    Quaternion inertializeQuaternion(InertializationInfo &info, Quaternion &newQ_t, double t) {
        if (t >= info.t1)
            // inertialization is done just return newQ
            return newQ_t;

        // xt = A * t^5 + B * t^4 + C * t^3 + a0/2 * t^2 + v0 * t + x0
        double x = info.A * t * t * t * t * t  //
                   + info.B * t * t * t * t    //
                   + info.C * (t * t * t)      //
                   + 0.5 * info.a0 * t * t     //
                   + info.v0 * t               //
                   + info.x0;                  //

        Quaternion dq = Quaternion(cos(0.5 * x),                 //
                                   sin(0.5 * x) * info.x0v.x(),  //
                                   sin(0.5 * x) * info.x0v.y(),  //
                                   sin(0.5 * x) * info.x0v.z())
                            .normalized();

        return dq * newQ_t;
    }

    /**
     * inertialize vector newV_t at time t.
     */
    V3D inertializeVector(InertializationInfo &info, V3D &newV_t, double t) {
        if (t >= info.t1)
            // inertialization done just return newV
            return newV_t;

        // xt = A * t^5 + B * t^4 + C * t^3 + a0/2 * t^2 + v0 * t + x0
        double x = info.A * t * t * t * t * t  //
                   + info.B * t * t * t * t    //
                   + info.C * (t * t * t)      //
                   + 0.5 * info.a0 * t * t     //
                   + info.v0 * t               //
                   + info.x0;                  //

        V3D dv = V3D(info.x0v.normalized()) * x;

        return newV_t + dv;
    }

    // functions for spring damper system
    double halflife_to_damping(double halflife, double eps = 1e-5f)
    {
        return (4.0f * 0.69314718056f) / (halflife + eps);
    }
    double fast_negexp(double x)
    {
        return 1.0f / (1.0f + x + 0.48f*x*x + 0.235f*x*x*x);
    }
    void spring_character_update(
        double& x, // current position
        double& v, // current velocity
        double& a, // initial acceleration, die down while dt grows
        double v_goal, // goal velocity
        double halflife, // expected time to finish half converging (reach the mid-point between v and v_goal)
        double dt)
    {
        double y = halflife_to_damping(halflife) / 2.0f;	
        double j0 = v - v_goal;
        double j1 = a + j0*y;
        double eydt = fast_negexp(y*dt);

        x = eydt*(((-j1)/(y*y)) + ((-j0 - j1*dt)/y)) + 
            (j1/(y*y)) + j0/y + v_goal * dt + x;
        v = eydt*(j0 + j1*dt) + v_goal;
        a = eydt*(a - j1*y*dt);
    }

    bool isPropertoStop()
    {
        if (!(this->KEY_A || this->KEY_D || this->KEY_S || this->KEY_W || this->KEY_J || this->isDance_ || this->KEY_C))
            return true;
        return false;
    }
    /**
     * create query vector from current skeleton's state and user command.
     */
    dVector createQueryVector(const crl::gui::TrackingCamera &camera) {
        Quaternion rootQ = skeleton_->root->state.orientation;
        P3D rootPos = skeleton_->root->state.pos;
        V3D rootVel = skeleton_->root->state.velocity;

        // becareful! skeleton's forward direction is x axis not z axis!
        // double roll, pitch, yaw;
        // computeEulerAnglesFromQuaternion(rootQ,  //
        //                                  skeleton_->forwardAxis, skeleton_->forwardAxis.cross(skeleton_->upAxis), skeleton_->upAxis, roll, pitch, yaw);

        // current character frame
        // Quaternion characterQ = getRotationQuaternion(yaw, skeleton_->upAxis);
        Quaternion characterQ = calc_facing_quat(rootQ);
        P3D characterPos(rootPos.x, 0, rootPos.z);
        V3D characterVel(P3D(rootVel[0], 0, rootVel[2]));
        double characterYaw = calc_facing_yaw(rootQ);

        // generate trajectory (of character)
        Trajectory3D queryPosTrajectory;
        // Trajectory1D queryHeadingTrajectory;
        Trajectory3D queryVelTrajectory;
        
        if (paintTraj.size() == 0) {
            // camera facing direction
            glm::vec3 orientation = camera.getOrientation();
            V3D camera_dir(orientation.x, 0, orientation.z);
            cameraDir = camera_dir;

            // set move direction based keyboard input
            Eigen::Vector2d key_dir(0,0);
            if (KEY_W) key_dir[0] += 1;
            if (KEY_A) key_dir[1] -= 1;
            if (KEY_S) key_dir[0] -= 1;
            if (KEY_D) key_dir[1] += 1;
            key_dir = key_dir.normalized();
            Eigen::Matrix3d rot_matrix;
            rot_matrix << key_dir[0], 0, -key_dir[1],
                            0, 1, 0,
                            key_dir[1], 0, key_dir[0];

            // in the world frame
            V3D goal_dir = (rot_matrix * camera_dir.normalized()).normalized();
            V3D goal_vel = goal_dir * this->speedForward;
            goalVel = goal_vel;

            P3D pos = characterPos;
            V3D vel = characterVel;

            double dtTraj = 1.0 / 60;  // trajectory dt
            double t = 0;
            double halflife = 0.3f;

            while (t <= 1.0) {
                V3D curr_vel = vel; // V0 in the world frame
                P3D curr_pos = pos; // X0 in the world frame
                V3D init_a = goal_vel - curr_vel; // initial acceleration in the world frame
                
                // compute traj infomation
                spring_character_update(curr_pos[0], curr_vel[0], init_a[0], goal_vel[0], halflife, t);
                spring_character_update(curr_pos[1], curr_vel[1], init_a[1], goal_vel[1], halflife, t);
                spring_character_update(curr_pos[2], curr_vel[2], init_a[2], goal_vel[2], halflife, t);


                // store trajectory at time t
                queryPosTrajectory.addKnot(t, V3D(curr_pos));
                queryVelTrajectory.addKnot(t, curr_vel);

                t += dtTraj;
            }
        }
        else{
            // banned visualization
            cameraDir = V3D(P3D());
            goalVel = V3D(P3D());

            // init nearest trajInd
            if (trajInd != 0){
                double dist = INFINITY;
                for (int i=0; i<paintTraj.size(); i++){
                    V3D d = V3D(paintTraj[i], characterPos);
                    if(d.norm() < dist){
                        dist = d.norm();
                        trajInd = i;
                    }
                }
            }
            
            P3D curr_pos = characterPos; // x0
            double const_vel = this->speedForward;

            double dtTraj = 1.0 / 60;  // trajectory dt
            double t = 0;

            while (t <= 1.0) {
                V3D curr_vel;
                double target_dist = dtTraj * const_vel;

                while (true)
                {
                    P3D target_pos = paintTraj[trajInd];
                    V3D target_dir = V3D(curr_pos, target_pos);
                    if(target_dir.norm() > target_dist){
                        curr_vel = target_dir.normalized() * const_vel; 
                        curr_pos = curr_pos + curr_vel.normalized() * target_dist;
                        target_dist = 0;
                        break;
                    } 
                    else{
                        target_dist -= target_dir.norm();
                        curr_pos = target_pos;
                        trajInd ++;
                    }

                    if (trajInd >= paintTraj.size())
                        trajInd = 0;
                }
                
                // store trajectory at time t
                queryPosTrajectory.addKnot(t, V3D(curr_pos));
                queryVelTrajectory.addKnot(t, curr_vel);

                t += dtTraj;
            }
        }
        PosTraj = queryPosTrajectory;
        VelTraj = queryVelTrajectory;

        // build query vector
        dVector xq(27);
        bool stop20 = false;
        bool stop40 = false;
        bool stop60 = false;
        double stop_threshold = 0.8;
        // 1/2: 2D projected future trajectory
        // TODO: check if 20/40/60 * dt is correct
        // after 20 frames
        P3D characterPosAfter20 = P3D() + queryPosTrajectory.evaluate_linear(20 * dt_);
        V3D tt1(characterPos, characterPosAfter20);
        tt1 = characterQ.inverse() * tt1;

        // Quaternion characterQAfter20 = getRotationQuaternion(queryHeadingTrajectory.evaluate_linear(20 * dt_), skeleton_->upAxis);
        // V3D td1 = characterQAfter20 * skeleton_->forwardAxis;
        // td1 = characterQ.inverse() * td1;
        V3D td1 = queryVelTrajectory.evaluate_linear(20.0 * dt_);
        td1 = characterQ.inverse() * td1;
        td1.y() = 0;
        if (td1.norm() < stop_threshold){ // set to be same as previous point
            tt1 = V3D(characterPos, characterPos);
            td1 = V3D(P3D());
            stop20 = true;
        } 
        else td1.normalize();

        // after 40 frames
        P3D characterPosAfter40 = P3D() + queryPosTrajectory.evaluate_linear(40.0 * dt_);
        V3D tt2(characterPos, characterPosAfter40);
        tt2 = characterQ.inverse() * tt2;

        // Quaternion characterQAfter40 = getRotationQuaternion(queryHeadingTrajectory.evaluate_linear(40 * dt_), skeleton_->upAxis);
        // V3D td2 = characterQAfter40 * skeleton_->forwardAxis;
        // td2 = characterQ.inverse() * td2;
        V3D td2 = queryVelTrajectory.evaluate_linear(40.0 * dt_);
        td2 = characterQ.inverse() * td2;
        td2.y() = 0;
        if (td2.norm() < stop_threshold){ // set to be same as previous point
            tt2 = tt1;
            td2 = V3D(P3D());
            stop40 = true;
        } 
        else td2.normalize();

        // after 60 frames
        P3D characterPosAfter60 = P3D() + queryPosTrajectory.evaluate_linear(60 * dt_);
        V3D tt3(characterPos, characterPosAfter60);
        tt3 = characterQ.inverse() * tt3;

        // Quaternion characterQAfter60 = getRotationQuaternion(queryHeadingTrajectory.evaluate_linear(60 * dt_), skeleton_->upAxis);
        // V3D td3 = characterQAfter60 * skeleton_->forwardAxis;
        // td3 = characterQ.inverse() * td3;
        V3D td3 = queryVelTrajectory.evaluate_linear(60.0 * dt_);
        td3 = characterQ.inverse() * td3;
        td3.y() = 0;
        if (td3.norm() < stop_threshold){ // set to be same as previous point
            tt3 = tt2;
            td3 = V3D(P3D());
            stop60 = true;
        } 
        else td3.normalize();

        if (stop20 && stop40 && stop60 && isPropertoStop())// && !this->isCartoonOn_
        {
            this->shouldStop_ = true;
        }
        // 3/4: feet positions and velocities w.r.t character frame (in R^12)
        auto *hlJoint = skeleton_->getMarkerByName("LeftFoot");
        auto *hrJoint = skeleton_->getMarkerByName("RightFoot");

        // get world position of two feet
        P3D hlFeetPos = hlJoint->state.pos;
        P3D hrFeetPos = hrJoint->state.pos;

        // vector between hip and feet, transfer from world to local frame
        V3D ft2 = characterQ.inverse() * V3D(characterPos, hlFeetPos);
        V3D ft4 = characterQ.inverse() * V3D(characterPos, hrFeetPos);

        // get world velocity of two feet
        V3D ft2dot = hlJoint->state.velocity;
        V3D ft4dot = hrJoint->state.velocity;

        // transfer velocity from world to local frame
        ft2dot = characterQ.inverse() * ft2dot;
        ft4dot = characterQ.inverse() * ft4dot;

        // 5: hip (root) joint velocity w.r.t character frame (in R^3)
        V3D htdot = skeleton_->root->state.velocity;
        htdot = characterQ.inverse() * htdot;

        // save to feature vector
        xq[0] = tt1.x();
        xq[1] = tt1.z();
        xq[2] = tt2.x();
        xq[3] = tt2.z();
        xq[4] = tt3.x();
        xq[5] = tt3.z();

        xq[6] = td1.x();
        xq[7] = td1.z();
        xq[8] = td2.x();
        xq[9] = td2.z();
        xq[10] = td3.x();
        xq[11] = td3.z();

        // xq[12] = ft1.x();
        // xq[13] = ft1.y();
        // xq[14] = ft1.z();
        xq[12] = ft2.x();
        xq[13] = ft2.y();
        xq[14] = ft2.z();
        // xq[18] = ft3.x();
        // xq[19] = ft3.y();
        // xq[20] = ft3.z();
        xq[15] = ft4.x();
        xq[16] = ft4.y();
        xq[17] = ft4.z();

        // xq[24] = ft1dot.x();
        // xq[25] = ft1dot.y();
        // xq[26] = ft1dot.z();
        xq[18] = ft2dot.x();
        xq[19] = ft2dot.y();
        xq[20] = ft2dot.z();
        // xq[30] = ft3dot.x();
        // xq[31] = ft3dot.y();
        // xq[32] = ft3dot.z();
        xq[21] = ft4dot.x();
        xq[22] = ft4dot.y();
        xq[23] = ft4dot.z();

        xq[24] = htdot.x();
        xq[25] = htdot.y();
        xq[26] = htdot.z();

        return xq;
    }

    /**
     * create query vector from current skeleton's state and user command.
     */
    dVector createQueryVector3d(const crl::gui::TrackingCamera &camera) {
        Quaternion rootQ = skeleton_->root->state.orientation;
        P3D rootPos = skeleton_->root->state.pos;
        V3D rootVel = skeleton_->root->state.velocity;

        // becareful! skeleton's forward direction is x axis not z axis!
        // double roll, pitch, yaw;
        // computeEulerAnglesFromQuaternion(rootQ,  //
        //                                  skeleton_->forwardAxis, skeleton_->forwardAxis.cross(skeleton_->upAxis), skeleton_->upAxis, roll, pitch, yaw);

        // current character frame
        // Quaternion characterQ = getRotationQuaternion(yaw, skeleton_->upAxis);
        Quaternion characterQ = calc_facing_quat(rootQ);
        P3D characterPos = rootPos;
        V3D characterVel(P3D(rootVel[0], 0, rootVel[2]));
        double characterYaw = calc_facing_yaw(rootQ);

        // generate trajectory (of character)
        Trajectory3D queryPosTrajectory;
        // Trajectory1D queryHeadingTrajectory;
        Trajectory3D queryVelTrajectory;
        
        {
            // camera facing direction
            glm::vec3 orientation = camera.getOrientation();
            V3D camera_dir(orientation.x, 0, orientation.z);
            cameraDir = camera_dir;

            V3D goal_vel = characterVel;
            P3D pos = characterPos;
            V3D vel = characterVel;

            if(KEY_J){
                goal_vel.y() -= 3.0; // minus jump goal velocity
                goalVel = goal_vel;
                vel.y() += 3.0; // add jump init velocity
            }
            if(KEY_C){
                Eigen::Vector2d key_dir(0,0);
                if (KEY_W) key_dir[0] += 1;
                if (KEY_A) key_dir[1] -= 1;
                if (KEY_S) key_dir[0] -= 1;
                if (KEY_D) key_dir[1] += 1;
                key_dir = key_dir.normalized();
                Eigen::Matrix3d rot_matrix;
                rot_matrix << key_dir[0], 0, -key_dir[1],
                                0, 1, 0,
                                key_dir[1], 0, key_dir[0];

                // in the world frame
                V3D goal_dir = (rot_matrix * camera_dir.normalized()).normalized();
                goal_vel = goal_dir * 0.8; // creep velocity is fixed to 1
                goalVel = goal_vel;
                if (vel.norm() > 1.5) vel = vel.normalized() * 0.8; // match hip velocity
                // if (pos.y > 0.5) vel.y() -= 3.0; // match hip height
            }

            double dtTraj = 1.0 / 60;  // trajectory dt
            double t = 0;
            double halflife = 0.3f;

            while (t <= 1.0) {
                V3D curr_vel = vel; // V0 in the world frame
                P3D curr_pos = pos; // X0 in the world frame
                V3D init_a = goal_vel - curr_vel; // initial acceleration in the world frame
                
                // compute traj infomation
                spring_character_update(curr_pos[0], curr_vel[0], init_a[0], goal_vel[0], halflife, t);
                spring_character_update(curr_pos[1], curr_vel[1], init_a[1], goal_vel[1], halflife, t);
                spring_character_update(curr_pos[2], curr_vel[2], init_a[2], goal_vel[2], halflife, t);

                // std::cout<< vel.norm() << std::endl;
                if(KEY_C) curr_pos.y = 0.40;
                curr_pos.y = curr_pos.y > 0? curr_pos.y : 0;
                // store trajectory at time t
                queryPosTrajectory.addKnot(t, V3D(curr_pos));
                queryVelTrajectory.addKnot(t, curr_vel);

                t += dtTraj;
            }
        }

        PosTraj = queryPosTrajectory;
        VelTraj = queryVelTrajectory;

        // build query vector
        dVector xq(33);

        bool stop20 = false; 
        bool stop40 = false;
        bool stop60 = false;
        double stop_threshold = 0.0;
        // 1/2: 2D projected future trajectory
        // TODO: check if 20/40/60 * dt is correct
        // after 20 frames
        P3D characterPosAfter20 = P3D() + queryPosTrajectory.evaluate_linear(20 * dt_);
        V3D tt1(characterPos, characterPosAfter20);
        tt1 = characterQ.inverse() * tt1;

        // Quaternion characterQAfter20 = getRotationQuaternion(queryHeadingTrajectory.evaluate_linear(20 * dt_), skeleton_->upAxis);
        // V3D td1 = characterQAfter20 * skeleton_->forwardAxis;
        // td1 = characterQ.inverse() * td1;
        V3D td1 = queryVelTrajectory.evaluate_linear(20.0 * dt_);
        td1 = characterQ.inverse() * td1;
        // td1.y() = 0;
        if (td1.norm() < stop_threshold){ // set to be same as previous point
            tt1 = V3D(characterPos, characterPos); 
            td1 = V3D(P3D());
            stop20 = true;
        } 
        else td1.normalize();

        // after 40 frames
        P3D characterPosAfter40 = P3D() + queryPosTrajectory.evaluate_linear(40.0 * dt_);
        V3D tt2(characterPos, characterPosAfter40);
        tt2 = characterQ.inverse() * tt2;

        // Quaternion characterQAfter40 = getRotationQuaternion(queryHeadingTrajectory.evaluate_linear(40 * dt_), skeleton_->upAxis);
        // V3D td2 = characterQAfter40 * skeleton_->forwardAxis;
        // td2 = characterQ.inverse() * td2;
        V3D td2 = queryVelTrajectory.evaluate_linear(40.0 * dt_);
        td2 = characterQ.inverse() * td2;
        // td2.y() = 0;
        if (td2.norm() < stop_threshold){ // set to be same as previous point
            tt2 = tt1;
            td2 = V3D(P3D());
            stop40 = true;
        } 
        else td2.normalize();

        // after 60 frames
        P3D characterPosAfter60 = P3D() + queryPosTrajectory.evaluate_linear(60 * dt_);
        V3D tt3(characterPos, characterPosAfter60);
        tt3 = characterQ.inverse() * tt3;

        // Quaternion characterQAfter60 = getRotationQuaternion(queryHeadingTrajectory.evaluate_linear(60 * dt_), skeleton_->upAxis);
        // V3D td3 = characterQAfter60 * skeleton_->forwardAxis;
        // td3 = characterQ.inverse() * td3;
        V3D td3 = queryVelTrajectory.evaluate_linear(60.0 * dt_);
        td3 = characterQ.inverse() * td3;
        // td3.y() = 0;
        if (td3.norm() < stop_threshold){ // set to be same as previous point
            tt3 = tt2;
            td3 = V3D(P3D());
            stop60 = true;
        } 
        else td3.normalize();

        // 3/4: feet positions and velocities w.r.t character frame (in R^12)
        auto *hlJoint = skeleton_->getMarkerByName("LeftFoot");
        auto *hrJoint = skeleton_->getMarkerByName("RightFoot");

        // get world position of two feet
        P3D hlFeetPos = hlJoint->state.pos;
        P3D hrFeetPos = hrJoint->state.pos;

        // vector between hip and feet, transfer from world to local frame
        V3D ft2 = characterQ.inverse() * V3D(characterPos, hlFeetPos);
        V3D ft4 = characterQ.inverse() * V3D(characterPos, hrFeetPos);

        // get world velocity of two feet
        V3D ft2dot = hlJoint->state.velocity;
        V3D ft4dot = hrJoint->state.velocity;

        // transfer velocity from world to local frame
        ft2dot = characterQ.inverse() * ft2dot;
        ft4dot = characterQ.inverse() * ft4dot;

        // 5: hip (root) joint velocity w.r.t character frame (in R^3)
        V3D htdot = skeleton_->root->state.velocity;
        htdot = characterQ.inverse() * htdot;

        // save to feature vector
        xq[0] = tt1.x();
        xq[1] = tt1.y();
        xq[2] = tt1.z();
        xq[3] = tt2.x();
        xq[4] = tt2.y();
        xq[5] = tt2.z();
        xq[6] = tt3.x();
        xq[7] = tt3.y();
        xq[8] = tt3.z();

        xq[9] = td1.x();
        xq[10] = td1.y();
        xq[11] = td1.z();
        xq[12] = td2.x();
        xq[13] = td2.y();
        xq[14] = td2.z();
        xq[15] = td3.x();
        xq[16] = td3.y();
        xq[17] = td3.z();

        // xq[12] = ft1.x();
        // xq[13] = ft1.y();
        // xq[14] = ft1.z();
        xq[18] = ft2.x();
        xq[19] = ft2.y();
        xq[20] = ft2.z();
        // xq[18] = ft3.x();
        // xq[19] = ft3.y();
        // xq[20] = ft3.z();
        xq[21] = ft4.x();
        xq[22] = ft4.y();
        xq[23] = ft4.z();

        // xq[24] = ft1dot.x();
        // xq[25] = ft1dot.y();
        // xq[26] = ft1dot.z();
        xq[24] = ft2dot.x();
        xq[25] = ft2dot.y();
        xq[26] = ft2dot.z();
        // xq[30] = ft3dot.x();
        // xq[31] = ft3dot.y();
        // xq[32] = ft3dot.z();
        xq[27] = ft4dot.x();
        xq[28] = ft4dot.y();
        xq[29] = ft4dot.z();

        xq[30] = htdot.x();
        xq[31] = htdot.y();
        xq[32] = htdot.z();

        return xq;
    }

    // Quaternion computeCharacterQ(Quaternion &rootQ, V3D xAxis, V3D zAxis) {
    //     double roll, pitch, yaw;
    //     computeEulerAnglesFromQuaternion(rootQ,  //
    //                                      xAxis, zAxis, zAxis.cross(xAxis), roll, pitch, yaw);
    //     return getRotationQuaternion(yaw, zAxis.cross(xAxis));
    // }

    /**
     * motion stitched to current skeleton pose
     * @note input is the raw state from the animation database
     */
    MocapSkeletonState computeStitchedMotion(const MocapSkeletonState &state) {
        MocapSkeletonState stitchedMotion = state;
        Quaternion Q_wtt = stitchedMotion.getRootOrientation();
        P3D t_wtt = stitchedMotion.getRootPosition();

        Quaternion Q_wttprime = Q_wt0prime * Q_wt0.inverse() * Q_wtt;
        P3D t_wttprime = t_wt0prime + Q_wt0prime * Q_wt0.inverse() * (V3D(t_wt0, t_wtt));
        stitchedMotion.setRootOrientation(Q_wttprime);
        stitchedMotion.setRootPosition(t_wttprime);
        return stitchedMotion;
    }

    /**
     * save future motions from next frame which will be played by advance() 
     * function. 
     */
    void saveFutureMotionIntoQueue() {
        // back up for t-1
        MocapSkeletonState newMotion_tMinus1(skeleton_);

        // save inertialized motion to queue (from next frame)
        int localQueueSize = this->queueSize;
        if (this->shouldStop_)
        {
            localQueueSize = database_->getClipByClipIndex(0)->getFrameCount() - 1;
        }
        else if (this->isDance_)
        {
            int currentMatchClipFrameCount = database_->getClipByClipIndex(currMotionIdx_.first)->getFrameCount();
            localQueueSize = currentMatchClipFrameCount - currMotionIdx_.second;
        }
        for (uint i = 0; i < localQueueSize; i++) {
        // for (uint i = 0; i < queueSize; i++) {
            if (currMotionIdx_.second + i >= database_->getClipByClipIndex(currMotionIdx_.first)->getFrameCount())
                throw std::runtime_error(
                    "MotionMatcher::matchMotion() error: something "
                    "wrong in index (current index = " +
                    std::to_string(currMotionIdx_.second) + " + " + std::to_string(i) +
                    " >= data size = " + std::to_string(database_->getClipByClipIndex(currMotionIdx_.first)->getFrameCount()) + ")");

            MocapSkeletonState newMotion_t = computeStitchedMotion(database_->getMotionByMotionIndex({currMotionIdx_.first, currMotionIdx_.second + i}));

            double t = i * dt_;

            if (inertialization) {
                auto inertializedMotion = inertializeState(newMotion_t, newMotion_tMinus1, t, dt_);

                // save inertialized motion to queue
                queue_.push_back(inertializedMotion);
                newMotion_tMinus1 = inertializedMotion;
            } else {
                MocapSkeletonState newMotion_t = computeStitchedMotion(database_->getMotionByMotionIndex({currMotionIdx_.first, currMotionIdx_.second + i}));

                queue_.push_back(newMotion_t);
            }
        }
    }
};

}  // namespace crl::mocap