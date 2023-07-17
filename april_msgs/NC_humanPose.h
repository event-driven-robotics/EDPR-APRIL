/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

// This is an automatically generated file.

// Generated from the following "NC_humanPose" msg definition:
//   float64 timestamp
//   float64[] pose
//   float64[] velocity// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARP_ROSMSG_NC_humanPose_h
#define YARP_ROSMSG_NC_humanPose_h

#include <yarp/os/Wire.h>
#include <yarp/os/Type.h>
#include <yarp/os/idl/WireTypes.h>
#include <string>
#include <vector>

namespace yarp {
namespace rosmsg {

class NC_humanPose : public yarp::os::idl::WirePortable
{
public:
    yarp::conf::float64_t timestamp;
    std::vector<yarp::conf::float64_t> pose;
    std::vector<yarp::conf::float64_t> velocity;

    NC_humanPose() :
            timestamp(0.0),
            pose(),
            velocity()
    {
    }

    void clear()
    {
        // *** timestamp ***
        timestamp = 0.0;

        // *** pose ***
        pose.clear();

        // *** velocity ***
        velocity.clear();
    }

    bool readBare(yarp::os::ConnectionReader& connection) override
    {
        // *** timestamp ***
        timestamp = connection.expectFloat64();

        // *** pose ***
        int len = connection.expectInt32();
        pose.resize(len);
        if (len > 0 && !connection.expectBlock((char*)&pose[0], sizeof(yarp::conf::float64_t)*len)) {
            return false;
        }

        // *** velocity ***
        len = connection.expectInt32();
        velocity.resize(len);
        if (len > 0 && !connection.expectBlock((char*)&velocity[0], sizeof(yarp::conf::float64_t)*len)) {
            return false;
        }

        return !connection.isError();
    }

    bool readBottle(yarp::os::ConnectionReader& connection) override
    {
        connection.convertTextMode();
        yarp::os::idl::WireReader reader(connection);
        if (!reader.readListHeader(3)) {
            return false;
        }

        // *** timestamp ***
        timestamp = reader.expectFloat64();

        // *** pose ***
        if (connection.expectInt32() != (BOTTLE_TAG_LIST|BOTTLE_TAG_FLOAT64)) {
            return false;
        }
        int len = connection.expectInt32();
        pose.resize(len);
        for (int i=0; i<len; i++) {
            pose[i] = (yarp::conf::float64_t)connection.expectFloat64();
        }

        // *** velocity ***
        if (connection.expectInt32() != (BOTTLE_TAG_LIST|BOTTLE_TAG_FLOAT64)) {
            return false;
        }
        len = connection.expectInt32();
        velocity.resize(len);
        for (int i=0; i<len; i++) {
            velocity[i] = (yarp::conf::float64_t)connection.expectFloat64();
        }

        return !connection.isError();
    }

    using yarp::os::idl::WirePortable::read;
    bool read(yarp::os::ConnectionReader& connection) override
    {
        return (connection.isBareMode() ? readBare(connection)
                                        : readBottle(connection));
    }

    bool writeBare(yarp::os::ConnectionWriter& connection) const override
    {
        // *** timestamp ***
        connection.appendFloat64(timestamp);

        // *** pose ***
        connection.appendInt32(pose.size());
        if (pose.size()>0) {
            connection.appendExternalBlock((char*)&pose[0], sizeof(yarp::conf::float64_t)*pose.size());
        }

        // *** velocity ***
        connection.appendInt32(velocity.size());
        if (velocity.size()>0) {
            connection.appendExternalBlock((char*)&velocity[0], sizeof(yarp::conf::float64_t)*velocity.size());
        }

        return !connection.isError();
    }

    bool writeBottle(yarp::os::ConnectionWriter& connection) const override
    {
        connection.appendInt32(BOTTLE_TAG_LIST);
        connection.appendInt32(3);

        // *** timestamp ***
        connection.appendInt32(BOTTLE_TAG_FLOAT64);
        connection.appendFloat64(timestamp);

        // *** pose ***
        connection.appendInt32(BOTTLE_TAG_LIST|BOTTLE_TAG_FLOAT64);
        connection.appendInt32(pose.size());
        for (size_t i=0; i<pose.size(); i++) {
            connection.appendFloat64(pose[i]);
        }

        // *** velocity ***
        connection.appendInt32(BOTTLE_TAG_LIST|BOTTLE_TAG_FLOAT64);
        connection.appendInt32(velocity.size());
        for (size_t i=0; i<velocity.size(); i++) {
            connection.appendFloat64(velocity[i]);
        }

        connection.convertTextMode();
        return !connection.isError();
    }

    using yarp::os::idl::WirePortable::write;
    bool write(yarp::os::ConnectionWriter& connection) const override
    {
        return (connection.isBareMode() ? writeBare(connection)
                                        : writeBottle(connection));
    }

    // This class will serialize ROS style or YARP style depending on protocol.
    // If you need to force a serialization style, use one of these classes:
    typedef yarp::os::idl::BareStyle<yarp::rosmsg::NC_humanPose> rosStyle;
    typedef yarp::os::idl::BottleStyle<yarp::rosmsg::NC_humanPose> bottleStyle;

    // The name for this message, ROS will need this
    static constexpr const char* typeName = "NC_humanPose";

    // The checksum for this message, ROS will need this
    static constexpr const char* typeChecksum = "f60fac524517d870dbaac768aeafa188";

    // The source text for this message, ROS will need this
    static constexpr const char* typeText = "\
float64 timestamp\n\
float64[] pose\n\
float64[] velocity\n\
";

    yarp::os::Type getType() const override
    {
        yarp::os::Type typ = yarp::os::Type::byName(typeName, typeName);
        typ.addProperty("md5sum", yarp::os::Value(typeChecksum));
        typ.addProperty("message_definition", yarp::os::Value(typeText));
        return typ;
    }
};

} // namespace rosmsg
} // namespace yarp

#endif // YARP_ROSMSG_NC_humanPose_h
