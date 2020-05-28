#pragma once

#include <QtGlobal>

#if defined (WIN32)
    #if defined (_MSC_VER)
        #pragma warning(disable: 4251)
    #endif
    #if defined(VERANDA_PERCEPTION_BUILD_DLL)
        #define  VERANDA_PERCEPTION_DLL Q_DECL_EXPORT
    #else
        #define  VERANDA_PERCEPTION_DLL Q_DECL_IMPORT
    #endif
#else
    #define VERANDA_PERCEPTION_DLL
#endif
