#ifndef pointcloud_control_panel_VISIBILITY_CONTROL_HPP
#define pointcloud_control_panel_VISIBILITY_CONTROL_HPP

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
        #define pointcloud_control_panel_EXPORT __attribute__((dllexport))
        #define pointcloud_control_panel_IMPORT __attribute__((dllimport))
    #else
        #define pointcloud_control_panel_EXPORT __declspec(dllexport)
        #define pointcloud_control_panel_IMPORT __declspec(dllimport)
    #endif
    #ifdef pointcloud_control_panel_BUILDING_LIBRARY
        #define pointcloud_control_panel_PUBLIC pointcloud_control_panel_EXPORT
    #else
        #define pointcloud_control_panel_PUBLIC pointcloud_control_panel_IMPORT
    #endif
    #define pointcloud_control_panel_PUBLIC_TYPE pointcloud_control_panel_PUBLIC
    #define pointcloud_control_panel_LOCAL
#else
    #define pointcloud_control_panel_EXPORT __attribute__((visibility("default")))
    #define pointcloud_control_panel_IMPORT
    #if __GNUC__ >= 4
        #define pointcloud_control_panel_PUBLIC __attribute__((visibility("default")))
        #define pointcloud_control_panel_LOCAL __attribute__((visibility("hidden")))
    #else
        #define pointcloud_control_panel_PUBLIC
        #define pointcloud_control_panel_LOCAL
    #endif
    #define pointcloud_control_panel_PUBLIC_TYPE
#endif

#endif // pointcloud_control_panel_VISIBILITY_CONTROL_HPP