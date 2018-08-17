#ifndef __MOCAP_OPTITRACK_VERSION_H__
#define __MOCAP_OPTITRACK_VERSION_H__

#include <string>

namespace mocap_optitrack
{

/// \breif Version class containing the version information and helpers for comparison.
class Version
{
  public:
    Version();
    Version(int major, int minor, int revision, int build);
    Version(const std::string& version);
    ~Version();

    void setVersion(int major, int minor, int revision, int build);
    const std::string& getVersionString();
    bool operator > (const Version& comparison);
    bool operator >= (const Version& comparison);
    bool operator < (const Version& comparison);
    bool operator <= (const Version& comparison);
    bool operator == (const Version& comparison);

    int v_major;
    int v_minor;
    int v_revision;
    int v_build;
    std::string v_string;
};

}

#endif