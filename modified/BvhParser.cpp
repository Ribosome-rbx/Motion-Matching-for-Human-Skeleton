// STL includes
#include <fstream>
#include <ios>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>

// Parser include
#include "mocap/bvh/BvhParser.h"

/** Indicate whether bvh parser allows multi hierarchy or not
 * Not fully tested
 */
#define MULTI_HIERARCHY 0
#define VERBOSE 0

// FIXME: fix bad parsing methodology

namespace crl::mocap {

const std::string kChannels = "CHANNELS";
const std::string kEnd = "End";
const std::string kEndSite = "End Site";
const std::string kFrame = "Frame";
const std::string kFrames = "Frames:";
const std::string kHierarchy = "HIERARCHY";
const std::string kJoint = "JOINT";
const std::string kMotion = "MOTION";
const std::string kOffset = "OFFSET";
const std::string kRoot = "ROOT";

const std::string kXpos = "Xposition";
const std::string kYpos = "Yposition";
const std::string kZpos = "Zposition";
const std::string kXrot = "Xrotation";
const std::string kYrot = "Yrotation";
const std::string kZrot = "Zrotation";

const int scale = 100;
Eigen::Vector3d init_offset;
const std::string ROOT_NAME = "Hips";
//##############################################################################
// Main parse function
//##############################################################################
int BvhParser::parse(const fs::path &path, Bvh *bvh) {
#if VERBOSE
    std::cout << "Parsing file : " << path << std::endl;
#endif
    nextJointIndex_ = 0;
    bvh_ = bvh;

    // Open file stream
    std::ifstream file(path, std::ios::in);

    if (file.is_open()) {
        std::string token;

#if MULTI_HIERARCHY == 1
        while (file.good()) {
#endif
            file >> token;
            if (token == kHierarchy) {
                int ret = parseHierarchy(file);
                if (ret)
                    return ret;
            } else {
                std::cerr << "Bad structure of .bvh file. " << kHierarchy << " should be on the top of the file";
                return -1;
            }
#if MULTI_HIERARCHY == 1
        }
#endif
    } else {
        std::cerr << "Cannot open file to parse : " << path;
        return -1;
    }
#if VERBOSE
    std::cout << "Successfully parsed file" << std::endl;
#endif
    return 0;
}

//##############################################################################
// Function parsing hierarchy
//##############################################################################
int BvhParser::parseHierarchy(std::ifstream &file) {
#if VERBOSE
    std::cout << "Parsing hierarchy" << std::endl;
#endif

    std::string token;
    int ret;

    if (file.good()) {
        file >> token;

        //##########################################################################
        // Parsing joints
        //##########################################################################
        if (token == kRoot) {
            std::shared_ptr<BvhJoint> rootJoint;
            ret = parseJoint(file, nullptr, rootJoint);

            if (ret)
                return ret;
#if VERBOSE
            std::cout << "There is " << bvh_->num_channels() << " data channels in the"
                      << " file" << std::endl;
#endif

            bvh_->setRootJoint(rootJoint);
        } else {
            std::cerr << "Bad structure of .bvh file. Expected " << kRoot << ", but found \"" << token << "\"";
            return -1;
        }
    }

    if (file.good()) {
        file >> token;

        //##########################################################################
        // Parsing motion data
        //##########################################################################
        if (token == kMotion) {
            ret = parseMotion(file);

            if (ret)
                return ret;
        } else {
            std::cerr << "Bad structure of .bvh file. Expected " << kMotion << ", but found \"" << token << "\"";
            return -1;
        }
    }
    return 0;
}

//##############################################################################
// Function parsing joint
//##############################################################################
int BvhParser::parseJoint(std::ifstream &file, std::shared_ptr<BvhJoint> parent, std::shared_ptr<BvhJoint> &parsed) {
#if VERBOSE
    std::cout << "Parsing joint" << std::endl;
#endif

    std::shared_ptr<BvhJoint> joint = std::make_shared<BvhJoint>();
    joint->setParent(parent);
    joint->setIndex(nextJointIndex_++);

    std::string name;
    file >> name;
#if VERBOSE
    std::cout << "Joint name : " << name << std::endl;
#endif

    joint->setName(name);

    std::string token;
    std::vector<std::shared_ptr<BvhJoint>> children;
    int ret;

    file >> token;  // Consuming '{'
    file >> token;

    //############################################################################
    // Offset parsing
    //############################################################################
    if (token == kOffset) {
        double x, y, z;

        try {
            file >> x >> y >> z;
            x /= scale, y /= scale, z /= scale;
        } catch (const std::ios_base::failure e) {
            std::cerr << "Failure while parsing offset";
            return -1;
        }

        if(!parent)
            joint->setOffset(0, 0,  0);
        else
            joint->setOffset(x, y, z);

#if VERBOSE
        std::cout << "Offset x: " << x << ", y: " << y << ", z: " << z << std::endl;
#endif
    } else {
        std::cerr << "Bad structure of .bvh file. Expected " << kOffset << ", but "
                  << "found \"" << token << "\"";

        return -1;
    }

    file >> token;

    //############################################################################
    // Channels parsing
    //############################################################################
    if (token == kChannels) {
        ret = parseChannelOrder(file, joint);

#if VERBOSE
        std::cout << "Joint has " << joint->num_channels() << " data channels" << std::endl;
#endif

        if (ret)
            return ret;
    } else {
        std::cerr << "Bad structure of .bvh file. Expected " << kChannels << ", but found \"" << token << "\"" << std::endl;

        return -1;
    }

    file >> token;

    bvh_->addJoint(joint);

    /** Children parsing */
    while (file.good()) {
        /** Child joint parsing */
        if (token == kJoint) {
            std::shared_ptr<BvhJoint> child;
            ret = parseJoint(file, joint, child);

            if (ret)
                return ret;

            children.push_back(child);

        } else if (token == kEnd) {
            // Parse until start of offset declaration
            file >> token >> token >> token;

            /** End site offset parsing */
            if (token == kOffset) {
                double x, y, z;

                try {
                    file >> x >> y >> z;
                } catch (const std::ios_base::failure e) {
                    std::cerr << "Failure while parsing offset";
                    return -1;
                }

                // Add end-effector to the previous joint
                joint->addEndEffector({x, y, z});

#if VERBOSE
                std::cout << "Joint name : EndSite" << std::endl;
                std::cout << "Offset x: " << x << ", y: " << y << ", z: " << z << std::endl;
#endif

                file >> token;  // Consuming "}"
            } else {
                std::cerr << "Bad structure of .bvh file. Expected " << kOffset << ", but found \"" << token << "\"";

                return -1;
            }
        } else if (token == "}") {
            joint->setChildren(children);
            parsed = joint;
            return 0;
        }

        file >> token;
    }

    std::cerr << "Cannot parse joint, unexpected end of file. Last token : " << token;
    return -1;
}

//##############################################################################
// Motion data parse function
//##############################################################################
int BvhParser::parseMotion(std::ifstream &file) {
#if VERBOSE
    std::cout << "Parsing motion" << std::endl;
#endif

    std::string token;
    file >> token;

    int frames_num;

    if (token == kFrames) {
        file >> frames_num;
        bvh_->setNumberOfFrames(frames_num);
#if VERBOSE
        std::cout << "Num of frames : " << frames_num << std::endl;
#endif
    } else {
        std::cerr << "Bad structure of .bvh file. Expected " << kFrames << ", but found \"" << token << "\"" << std::endl;

        return -1;
    }

    file >> token;

    double frame_time;

    if (token == kFrame) {
        file >> token;  // Consuming 'Time:'
        file >> frame_time;
        bvh_->setFrameTime(frame_time);
#if VERBOSE
        std::cout << "Frame time : " << frame_time << std::endl;
#endif

        float number;
        for (int i = 0; i < frames_num; i++) {
            int counter = 0;
            for (auto joint : bvh_->getJoints()) {
                std::vector<float> data;
                for (int j = 0; j < joint->getNumberOfChannels(); j++) {
                    file >> number;
                    if (counter < 3) {
                        number /= scale;
                        number -= init_offset[counter];
                        counter++;
                    }
                    data.push_back(number);
                    // if (j == joint->getNumberOfChannels() - 1){
                    //     // std::swap(data[data.size()-2], data[data.size()-1]);
                    // }
                }

                joint->addFrameMotionData(data);
            }
        }
    } else {
        std::cerr << "Bad structure of .bvh file. Expected " << kFrame << ", but found \"" << token << "\"";

        return -1;
    }

    return 0;
}

//##############################################################################
// Channels order parse function
//##############################################################################
int BvhParser::parseChannelOrder(std::ifstream &file, std::shared_ptr<BvhJoint> joint) {
#if VERBOSE
    std::cout << "Parse channel order" << std::endl;
#endif

    int num;
    file >> num;

#if VERBOSE
    std::cout << "Number of channels : " << num << std::endl;
#endif

    std::vector<BvhJoint::Channel> channels;
    std::string token;

    for (int i = 0; i < num; i++) {
        file >> token;
        if (token == kXpos)
            channels.push_back(BvhJoint::Channel::XPOSITION);
        else if (token == kYpos)
            channels.push_back(BvhJoint::Channel::YPOSITION);
        else if (token == kZpos)
            channels.push_back(BvhJoint::Channel::ZPOSITION);
        else if (token == kXrot)
            channels.push_back(BvhJoint::Channel::XROTATION);
        else if (token == kYrot)
            channels.push_back(BvhJoint::Channel::YROTATION);
        else if (token == kZrot)
            channels.push_back(BvhJoint::Channel::ZROTATION);
        else {
            std::cerr << "Not valid channel!";
            return -1;
        }
    }

    joint->setChannelsOrder(channels);
    return 0;
}

std::string BvhParser::vtos(const std::vector<float> &vector) {
    std::ostringstream oss;

    if (!vector.empty()) {
        // Convert all but the last element to avoid a trailing ","
        std::copy(vector.begin(), vector.end() - 1, std::ostream_iterator<float>(oss, ", "));

        // Now add the last element with no delimiter
        oss << vector.back();
    }

    return oss.str();
}

}  // namespace crl::mocap
