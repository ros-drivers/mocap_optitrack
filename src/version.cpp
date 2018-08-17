#include <mocap_optitrack/version.h>

#include <sstream>

namespace mocap_optitrack
{

Version::Version()
{
  setVersion(0, 0, 0, 0);
}

Version::Version(int major, int minor, int revision, int build)
{
  setVersion(major, minor, revision, build);
}

Version::Version(const std::string& version)
  : v_string(version)
{
  std::sscanf(version.c_str(), "%d.%d.%d.%d", &v_major, &v_minor, &v_revision, &v_build);
}

Version::~Version()
{
}
void Version::setVersion(int major, int minor, int revision, int build)
{
  v_major = major;
  v_minor = minor;
  v_revision = revision;
  v_build = build;

  std::stringstream sstr;
  sstr << v_major << "." << v_minor << "." << v_revision << "." << v_build;
  v_string  = sstr.str();
}

const std::string& Version::getVersionString()
{
  return v_string;
}

bool Version::operator > (const Version& comparison)
{
  if (v_major > comparison.v_major)
    return true;
  if (v_minor > comparison.v_minor)
    return true;
  if (v_revision > comparison.v_revision)
    return true;
  if (v_build > comparison.v_build)
    return true;
  return false;
}

bool Version::operator >= (const Version& comparison)
{
  return ((*this > comparison) || (*this == comparison));
}

bool Version::operator < (const Version& comparison)
{
  if (v_major < comparison.v_major)
    return true;
  if (v_minor < comparison.v_minor)
    return true;
  if (v_revision < comparison.v_revision)
    return true;
  if (v_build < comparison.v_build)
    return true;
  return false;
}

bool Version::operator <= (const Version& comparison)
{
  return ((*this < comparison) || (*this == comparison));
}

bool Version::operator == (const Version& comparison)
{
  return v_major == comparison.v_major
      && v_minor == comparison.v_minor
      && v_revision == comparison.v_revision
      && v_build == comparison.v_build;
}

}