#ifndef WAYPOINT_RECONFIGURE__VISIBILITY_CONTROL_H_
#define WAYPOINT_RECONFIGURE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define WAYPOINT_RECONFIGURE_EXPORT __attribute__ ((dllexport))
    #define WAYPOINT_RECONFIGURE_IMPORT __attribute__ ((dllimport))
  #else
    #define WAYPOINT_RECONFIGURE_EXPORT __declspec(dllexport)
    #define WAYPOINT_RECONFIGURE_IMPORT __declspec(dllimport)
  #endif
  #ifdef WAYPOINT_RECONFIGURE_BUILDING_LIBRARY
    #define WAYPOINT_RECONFIGURE_PUBLIC WAYPOINT_RECONFIGURE_EXPORT
  #else
    #define WAYPOINT_RECONFIGURE_PUBLIC WAYPOINT_RECONFIGURE_IMPORT
  #endif
  #define WAYPOINT_RECONFIGURE_PUBLIC_TYPE WAYPOINT_RECONFIGURE_PUBLIC
  #define WAYPOINT_RECONFIGURE_LOCAL
#else
  #define WAYPOINT_RECONFIGURE_EXPORT __attribute__ ((visibility("default")))
  #define WAYPOINT_RECONFIGURE_IMPORT
  #if __GNUC__ >= 4
    #define WAYPOINT_RECONFIGURE_PUBLIC __attribute__ ((visibility("default")))
    #define WAYPOINT_RECONFIGURE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define WAYPOINT_RECONFIGURE_PUBLIC
    #define WAYPOINT_RECONFIGURE_LOCAL
  #endif
  #define WAYPOINT_RECONFIGURE_PUBLIC_TYPE
#endif

#endif  // WAYPOINT_RECONFIGURE__VISIBILITY_CONTROL_H_
