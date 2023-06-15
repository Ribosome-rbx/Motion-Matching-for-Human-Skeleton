//
// Created by Dongho Kang on 07.12.21.
//

#ifndef CRL_MOCAP_MOTIONDATABASE_H
#define CRL_MOCAP_MOTIONDATABASE_H

#define USE_AABB_SEARCH

#include "mocap/MocapClip.h"

namespace crl::mocap {

/**
 * motion index in database.
 */
typedef std::pair<int, int> MotionIndex;

/**
 * feature vector x and its corresponding idx.
 */
struct MocapFeature {
    // feature vector
    dVector x;
    // motion idx in MocapDataset
    uint motionIdx = -1;
    uint datasetIdx = -1;
};


static Matrix3x3 calc_facing_rotm(Quaternion &rootQ) {
    Matrix3x3 rotm = rootQ.normalized().toRotationMatrix();
    Vector3d up{0, 1, 0};

    Vector3d facing = -(rotm.col(1) - rotm.col(1).dot(up) * up).normalized();
    Vector3d side = facing.cross(up);

    Matrix3x3 new_rotm;
    new_rotm << facing, up, side;
    return new_rotm;
}

/**
 * @brief Use this whenever you want "Character Q"
 * 
 * @param rootQ raw root quaternion from mocapClip
 * @return Quaternion Charater Q
 */
inline Quaternion calc_facing_quat(Quaternion &rootQ) {
    return Quaternion(calc_facing_rotm(rootQ));
}

/**
 * @brief Use this to get "character yaw angle"
 * @note since atan2 was used, result ranges in [-pi, pi]
 * @param rootQ raw root quaternion from mocapClip
 * @return double character yaw angle
 */
inline double calc_facing_yaw(Quaternion &rootQ) {
    Matrix3x3 rotm = calc_facing_rotm(rootQ);
    return std::atan2(rotm(2,0), rotm(0,0));
}


/**
 * @brief 2 layer AABB tree
 * 
 */
class AABB {
public:
    double Dist2ThisAABB(const dVector &Vq, const dVector &stdVar, const dVector &postWeight) {
        return ((lb-Vq).cwiseMax(Eigen::VectorXd::Zero(Vq.size())) + (Vq-ub).cwiseMax(Eigen::VectorXd::Zero(Vq.size()))).cwiseQuotient(stdVar).cwiseProduct(postWeight).norm();
    }
    void CalcBB() {
        /*!> top layer AABB */
        if(children.size()) {
            lb = children[0].lb;
            ub = children[0].ub;
            for(auto c: children) {
                lb = lb.cwiseMin(c.lb);
                ub = ub.cwiseMax(c.ub);
            }
        }
        /*!> inner AABB */
        else {
            lb = features[0]->x;
            ub = features[0]->x;
            for(auto f: features) {
                lb = lb.cwiseMin(f->x);
                ub = ub.cwiseMax(f->x);
            }
        }
    }
    std::vector<MocapFeature *> features;
    std::vector<AABB> children;
    int flag;
private:
    dVector lb;
    dVector ub;
};



/**
 * we build motion database X.
 */
class MotionDatabase {
public:
    explicit MotionDatabase(std::string &dataDirectoryPath, bool use_y) {
        this->use_y_ = use_y;
        if (use_y)
            featureDim_ = 33;
        else
            featureDim_ = 27;
        // import clips
        importFilesFromDirectory(dataDirectoryPath);

        // index dataset
        indexDataset();
    }

    /**
     * search best match by query vector xq
     */
    MotionIndex searchBestMatchByQuery(const dVector &xq) {
        double minLoss = INFINITY;
        MotionIndex minIdx = {-1, -1};

#ifdef USE_AABB_SEARCH
        for (auto &b: aabb_) {
            if (b.Dist2ThisAABB(xq, sigma_,  this->use_y_?weight33_:weight27_) < minLoss) {
                for (auto &bb: b.children) {
                    if (bb.Dist2ThisAABB(xq, sigma_, this->use_y_?weight33_:weight27_) < minLoss) {
                        for (auto f: bb.features) {
                            double dist = (f->x - xq).cwiseQuotient(sigma_).cwiseProduct(this->use_y_?weight33_:weight27_).norm();
                            if (dist < minLoss) {
                                minLoss = dist;
                                minIdx = {f->datasetIdx, f->motionIdx};
                            }
                        }
                    }
                }
            }
        }
#else
        // normalized query
        dVector xqNormalized = (xq - mu_).array() / sigma_.array();

        int traj_feat_number = this->use_y_?18:12;
        int other_feat_number = this->featureDim_ - traj_feat_number;

        for (const auto &f : features_) {
            dVector xNormalized = (f.x - mu_).array() / sigma_.array();

            dVector loss_vec = xNormalized - xqNormalized;
            dVector traj_loss_vec(traj_feat_number), feature_loss_vec(other_feat_number);
            for (size_t i = 0; i < traj_feat_number; i++)
            {
                traj_loss_vec[i] = loss_vec[i];
            }
            for (size_t i = traj_feat_number; i < this->featureDim_; i++)
            {
                feature_loss_vec[i-traj_feat_number] = loss_vec[i];
            }
            double traj_loss = traj_loss_vec.norm(), feature_loss = feature_loss_vec.norm();
            double loss = 0.80*traj_loss + 0.20*feature_loss;
                        
            if (loss < minLoss) {
                minLoss = loss;
                minIdx = {f.datasetIdx, f.motionIdx};
            }
        }
#endif

        return minIdx;
    }

    uint getClipCount() {
        return clips_.size();
    }

    uint getTotalFrameCount() {
        uint cnt = 0;
        for (auto &c : clips_) {
            cnt += c->getFrameCount();
        }
        return cnt;
    }

    const std::unique_ptr<BVHClip> &getClipByClipIndex(uint clipIdx) const {
        if (clipIdx >= clips_.size())
            throw std::runtime_error("getMotionByMotionIndex error: wrong index");
        return clips_[clipIdx];
    }

    const MocapSkeletonState &getMotionByMotionIndex(MotionIndex idx) {
        if (idx.first >= clips_.size())
            throw std::runtime_error("getMotionByMotionIndex error: wrong index");
        if (idx.second >= clips_[idx.first]->getFrameCount())
            throw std::runtime_error("getMotionByMotionIndex error: wrong index");
        return clips_[idx.first]->getState(idx.second);
    }

private:
    void importFilesFromDirectory(std::string &dataDirectoryPath) {
        // load mocap clips from data directory.
        for (const auto &entry : fs::directory_iterator(dataDirectoryPath)) {
            try {
                if (entry.path().extension() == ".bvh") {
                    clips_.push_back(std::make_unique<crl::mocap::BVHClip>(entry.path()));
                }
            } catch (...) {
                crl::Logger::consolePrint("Failed to load %s file.\n", entry.path().c_str());
            }
        }

        // sort by name.
        std::sort(clips_.begin(), clips_.end(),                                                                     //
                  [](const std::unique_ptr<crl::mocap::BVHClip> &a, const std::unique_ptr<crl::mocap::BVHClip> &b)  //
                  { return a->getName() < b->getName(); });
    }

    /**
     * index the whole dataset into features and normalize
     */
    void indexDataset() {
        features_.clear();
        mu_ = dVector(featureDim_);
        mu_.setZero();
        sigma_ = dVector(featureDim_);
        sigma_.setConstant(1);

        // reserve features
        features_.reserve(getTotalFrameCount());
        
        /*!> total number of top level AABB 
        * Note that AABB cannot cross clips */
        int numTopAABB = 0;
        for (auto &c : clips_) {
            numTopAABB += (int)std::ceil((c->getFrameCount()-60)/64.0);
        }
        // std::cout << "OuterAABB counts = " << numTopAABB << std::endl;
        aabb_.reserve(numTopAABB);
        // std::cout << "aabb_.size() = " << aabb_.size() << std::endl;

        // store feature vector of each frame of dataset
        //
        // 1. 2D projected future trajectory (after 20/40/60 frames) w.r.t. character frame (in R^6)
        // 2. future heading vectors (after 20/40/60 frames) w.r.t character frame (in R^6)
        // 3. feet positions w.r.t character frame (in R^12)
        // 4. feet velocities w.r.t character frame (in R^12)
        // 5. hip (root) joint velocity w.r.t character frame (in R^3)
        //
        // character frame is a frame of the root projected to xz plane i.e.
        // the coordinate frame has orientation psi (yaw of root) and origin at (x, 0, z)
        //
        // note. once we finish indexing, we need to normalize this vectors by
        // std. of whole dataset

        for (uint i = 0; i < clips_.size(); i++) {
            auto &c = clips_[i];
            auto *skeleton = c->getModel();
            // frame
            // const V3D worldUp = skeleton->upAxis;
            // const V3D worldForward = skeleton->forwardAxis;
            const V3D worldForward(0,1,0);

            aabb_.push_back(AABB()); /*!>  new outer AABB */
            aabb_.back().children.push_back(AABB()); /*!< new inner AABB */

            // becareful! we save 61 frames into one feature.
            for (uint j = 0; j < c->getFrameCount() - 60; j++) {
                auto &m = c->getState(j);

                // feature vector
                dVector x(featureDim_);

                // temporarily set state first (for forward kinematic)
                skeleton->setState(&m);

                Quaternion rootQ = m.getRootOrientation();
                P3D rootPos = m.getRootPosition();

                // becareful! skeleton's forward direction is x axis not z axis!
                // double roll, pitch, yaw;
                // computeEulerAnglesFromQuaternion(rootQ,  //
                //                                  worldForward, worldForward.cross(worldUp), worldUp, roll, pitch, yaw);

                // current character frame
                // Quaternion characterQ = getRotationQuaternion(yaw, worldUp);
                Quaternion characterQ = calc_facing_quat(rootQ);
                P3D characterPos;
                if (this->use_y_)
                    characterPos = P3D(rootPos.x, rootPos.y, rootPos.z);
                else
                    characterPos = P3D(rootPos.x, 0, rootPos.z);

                // 1/2: 2D projected future trajectory and heading
                // after 20 frames
                P3D rootPosAfter20 = c->getState(j + 20).getRootPosition();
                V3D tt1(characterPos, rootPosAfter20);
                tt1 = characterQ.inverse() * tt1;

                // Quaternion rootQAfter20 = c->getState(j + 20).getRootOrientation();
                // V3D td1 = rootQAfter20 * worldForward;
                V3D td1 = c->getState(j + 20).getRootVelocity();
                td1 = characterQ.inverse() * td1;
                if (!this->use_y_)
                    td1.y() = 0;
                td1.normalize();

                // after 40 frames
                P3D rootPosAfter40 = c->getState(j + 40).getRootPosition();
                V3D tt2(characterPos, rootPosAfter40);
                tt2 = characterQ.inverse() * tt2;

                // Quaternion rootQAfter40 = c->getState(j + 40).getRootOrientation();
                // V3D td2 = rootQAfter40 * worldForward;
                V3D td2 = c->getState(j + 40).getRootVelocity();
                td2 = characterQ.inverse() * td2;
                if (!this->use_y_)
                    td2.y() = 0;
                td2.normalize();

                // after 60 frames
                P3D rootPosAfter60 = c->getState(j + 60).getRootPosition();
                V3D tt3(characterPos, rootPosAfter60);
                tt3 = characterQ.inverse() * tt3;

                // Quaternion rootQAfter60 = c->getState(j + 60).getRootOrientation();
                // V3D td3 = rootQAfter60 * worldForward;
                V3D td3 = c->getState(j + 60).getRootVelocity();
                td3 = characterQ.inverse() * td3;
                if (!this->use_y_)
                    td3.y() = 0;
                td3.normalize();

                // 3/4: feet positions and velocities w.r.t character frame (in R^12)
                // auto *flJoint = skeleton->getMarkerByName("LeftHand");
                auto *hlJoint = skeleton->getMarkerByName("LeftFoot");
                // auto *frJoint = skeleton->getMarkerByName("RightHand");
                auto *hrJoint = skeleton->getMarkerByName("RightFoot");

                // P3D flFeetPos = flJoint->state.getWorldCoordinates(flJoint->endSites[0].endSiteOffset);
                P3D hlFeetPos = hlJoint->state.getWorldCoordinates(P3D());
                // P3D frFeetPos = frJoint->state.getWorldCoordinates(frJoint->endSites[0].endSiteOffset);
                P3D hrFeetPos = hrJoint->state.getWorldCoordinates(P3D());

                // V3D ft1 = characterQ.inverse() * V3D(characterPos, flFeetPos);
                V3D ft2 = characterQ.inverse() * V3D(characterPos, hlFeetPos);
                // V3D ft3 = characterQ.inverse() * V3D(characterPos, frFeetPos);
                V3D ft4 = characterQ.inverse() * V3D(characterPos, hrFeetPos);

                // V3D ft1dot = flJoint->state.getVelocityForPoint_local(flJoint->endSites[0].endSiteOffset);
                V3D ft2dot = hlJoint->state.getVelocityForPoint_local(P3D());
                // V3D ft3dot = frJoint->state.getVelocityForPoint_local(frJoint->endSites[0].endSiteOffset);
                V3D ft4dot = hrJoint->state.getVelocityForPoint_local(P3D());

                // ft1dot = characterQ.inverse() * ft1dot;
                ft2dot = characterQ.inverse() * ft2dot;
                // ft3dot = characterQ.inverse() * ft3dot;
                ft4dot = characterQ.inverse() * ft4dot;

                // 5: hip (root) joint velocity w.r.t character frame (in R^3)
                V3D htdot = m.getRootVelocity();
                htdot = characterQ.inverse() * htdot;

                if (!this->use_y_)
                {
                    // save to feature vector
                    x[0] = tt1.x();
                    x[1] = tt1.z();
                    x[2] = tt2.x();
                    x[3] = tt2.z();
                    x[4] = tt3.x();
                    x[5] = tt3.z();

                    x[6] = td1.x();
                    x[7] = td1.z();
                    x[8] = td2.x();
                    x[9] = td2.z();
                    x[10] = td3.x();
                    x[11] = td3.z();

                    // x[12] = ft1.x();
                    // x[13] = ft1.y();
                    // x[14] = ft1.z();
                    x[12] = ft2.x();
                    x[13] = ft2.y();
                    x[14] = ft2.z();
                    // x[18] = ft3.x();
                    // x[19] = ft3.y();
                    // x[20] = ft3.z();
                    x[15] = ft4.x();
                    x[16] = ft4.y();
                    x[17] = ft4.z();

                    // x[24] = ft1dot.x();
                    // x[25] = ft1dot.y();
                    // x[26] = ft1dot.z();
                    x[18] = ft2dot.x();
                    x[19] = ft2dot.y();
                    x[20] = ft2dot.z();
                    // x[30] = ft3dot.x();
                    // x[31] = ft3dot.y();
                    // x[32] = ft3dot.z();
                    x[21] = ft4dot.x();
                    x[22] = ft4dot.y();
                    x[23] = ft4dot.z();

                    x[24] = htdot.x();
                    x[25] = htdot.y();
                    x[26] = htdot.z();
                }
                else
                {
                    // save to feature vector
                    x[0] = tt1.x();
                    x[1] = tt1.y();
                    x[2] = tt1.z();
                    x[3] = tt2.x();
                    x[4] = tt2.y();
                    x[5] = tt2.z();
                    x[6] = tt3.x();
                    x[7] = tt3.y();
                    x[8] = tt3.z();

                    x[9] = td1.x();
                    x[10] = td1.y();
                    x[11] = td1.z();
                    x[12] = td2.x();
                    x[13] = td2.y();
                    x[14] = td2.z();
                    x[15] = td3.x();
                    x[16] = td3.y();
                    x[17] = td3.z();

                    // x[12] = ft1.x();
                    // x[13] = ft1.y();
                    // x[14] = ft1.z();
                    x[18] = ft2.x();
                    x[19] = ft2.y();
                    x[20] = ft2.z();
                    // x[18] = ft3.x();
                    // x[19] = ft3.y();
                    // x[20] = ft3.z();
                    x[21] = ft4.x();
                    x[22] = ft4.y();
                    x[23] = ft4.z();

                    // x[24] = ft1dot.x();
                    // x[25] = ft1dot.y();
                    // x[26] = ft1dot.z();
                    x[24] = ft2dot.x();
                    x[25] = ft2dot.y();
                    x[26] = ft2dot.z();
                    // x[30] = ft3dot.x();
                    // x[31] = ft3dot.y();
                    // x[32] = ft3dot.z();
                    x[27] = ft4dot.x();
                    x[28] = ft4dot.y();
                    x[29] = ft4dot.z();

                    x[30] = htdot.x();
                    x[31] = htdot.y();
                    x[32] = htdot.z();
                }

                features_.emplace_back();
                features_.back().x = x;
                features_.back().motionIdx = j;
                features_.back().datasetIdx = i;

                /*!> add node to AABB */
                aabb_.back().children.back().features.push_back(&features_.back());

                if (aabb_.back().children.back().features.size() == 16) {
                    aabb_.back().children.back().CalcBB(); /*!< finish last inner AABB */
                    if (aabb_.back().children.size() == 4) { /*!< outer AABB full */
                        aabb_.back().CalcBB(); /*!< finish last outer AABB */
                        aabb_.push_back(AABB()); /*!< add new outer AABB */
                        aabb_.back().children.reserve(4);
                        aabb_.back().children.push_back(AABB()); /*!< add new inner AABB */
                        aabb_.back().children.back().features.reserve(16);
                    }
                    else {
                        aabb_.back().children.push_back(AABB()); /*!< add a new inner AABB */
                        aabb_.back().children.back().features.reserve(16);
                    }
                }
            }
            aabb_.back().children.back().CalcBB(); /*!< finishing the last innerd AABB, may not be full */
            aabb_.back().CalcBB(); /*!< finishing the last outer AABB, may not be full */
        }

        // normalize features
        computeFeatureMeanAndVar();
    }

    void computeFeatureMeanAndVar() {
        uint N = features_.size();

        // mean of x and x^2
        dVector E_x(featureDim_);
        dVector E_xsquare(featureDim_);
        E_x.setZero();
        E_xsquare.setZero();

        for (uint i = 0; i < N; i++) {
            E_x += features_[i].x;
            E_xsquare += dVector(features_[i].x.array() * features_[i].x.array());
        }

        // mean
        mu_ = E_x = E_x / N;

        // std
        E_xsquare = E_xsquare / N;
        sigma_ = E_xsquare - dVector(mu_.array() * mu_.array());
        sigma_ = sigma_.cwiseSqrt();

        if (printDebugInfo) {
            std::cout << "mu = " << std::endl  //
                      << mu_ << std::endl
                      << std::endl;
            std::cout << "sigma = " << std::endl << sigma_ << std::endl << std::endl;
        }
    }

public:
    // options
    bool printDebugInfo = true;

private:
    std::vector<std::unique_ptr<BVHClip>> clips_;
    std::vector<MocapFeature> features_;
    uint featureDim_;
    bool use_y_;

    // mean and std of features
    dVector mu_, sigma_;
    // Eigen::VectorXd weight_ {{2,2,2,2,2,2, 2,2,2,2,2,2, 1,1,1,1,1,1, 0.2,0.2,0.2,0.2,0.2,0.2, 1,1,1}};
    Eigen::VectorXd weight27_ {{0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,
                                0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2}};
    Eigen::VectorXd weight33_ {{0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,
                                0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2}};

    std::vector<AABB> aabb_;
};

}  // namespace crl::mocap

#endif  //CRL_MOCAP_MOTIONDATABASE_H
