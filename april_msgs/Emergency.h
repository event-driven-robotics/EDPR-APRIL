/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

// This is an automatically generated file.

// Generated from the following "Emergency" msg definition:
//   string emergency_label
// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARP_ROSMSG_Emergency_h
#define YARP_ROSMSG_Emergency_h

#include <yarp/os/Wire.h>
#include <yarp/os/Type.h>
#include <yarp/os/idl/WireTypes.h>
#include <string>
#include <vector>

namespace april_msgs {

class Emergency : public yarp::os::idl::WirePortable
{
public:
    std::string emergency_label;

    Emergency() :
            emergency_label("")
    {
    }

    void clear()
    {
        // *** emergency_label ***
        emergency_label = "";
    }

    bool readBare(yarp::os::ConnectionReader& connection) override
    {
        // *** emergency_label ***
        int len = connection.expectInt32();
        emergency_label.resize(len);
        if (!connection.expectBlock((char*)emergency_label.c_str(), len)) {
            return false;
        }

        return !connection.isError();
    }

    bool readBottle(yarp::os::ConnectionReader& connection) override
    {
        connection.convertTextMode();
        yarp::os::idl::WireReader reader(connection);
        if (!reader.readListHeader(1)) {
            return false;
        }

        // *** emergency_label ***
        if (!reader.readString(emergency_label)) {
            return false;
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
        // *** emergency_label ***
        connection.appendInt32(emergency_label.length());
        connection.appendExternalBlock((char*)emergency_label.c_str(), emergency_label.length());

        return !connection.isError();
    }

    bool writeBottle(yarp::os::ConnectionWriter& connection) const override
    {
        connection.appendInt32(BOTTLE_TAG_LIST);
        connection.appendInt32(1);

        // *** emergency_label ***
        connection.appendInt32(BOTTLE_TAG_STRING);
        connection.appendInt32(emergency_label.length());
        connection.appendExternalBlock((char*)emergency_label.c_str(), emergency_label.length());

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
    typedef yarp::os::idl::BareStyle<april_msgs::Emergency> rosStyle;
    typedef yarp::os::idl::BottleStyle<april_msgs::Emergency> bottleStyle;

    // The name for this message, ROS will need this
    static constexpr const char* typeName = "Emergency";

    // The checksum for this message, ROS will need this
    static constexpr const char* typeChecksum = "be0773c564beb804e77c237e3c2c40d5";

    // The source text for this message, ROS will need this
    static constexpr const char* typeText = "\
string emergency_label\n\
";

    yarp::os::Type getType() const override
    {
        yarp::os::Type typ = yarp::os::Type::byName(typeName, typeName);
        typ.addProperty("md5sum", yarp::os::Value(typeChecksum));
        typ.addProperty("message_definition", yarp::os::Value(typeText));
        return typ;
    }
};

}

#endif // YARP_ROSMSG_Emergency_h
