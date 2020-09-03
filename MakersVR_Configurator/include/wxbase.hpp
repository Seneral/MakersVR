/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef DEF_WX_BASE
#define DEF_WX_BASE

// wxWidgets disable unused libs
#define wxNO_NET_LIB
#define wxNO_XML_LIB
#define wxNO_EXPAT_LIB
#define wxNO_REGEX_LIB
#define wxNO_ZLIB_LIB
#define wxNO_JPEG_LIB
#define wxNO_PNG_LIB
#define wxNO_TIFF_LIB
#define wxNO_STC_LIB
#define wxNO_HTML_LIB
#define wxNO_QA_LIB
#define wxNO_XRC_LIB
#define wxNO_AUI_LIB
#define wxNO_PROPGRID_LIB
#define wxNO_RIBBON_LIB
#define wxNO_RICHTEXT_LIB
#define wxNO_MEDIA_LIB
#define wxNO_WEBVIEW_LIB
// wxWidgets minimal includes
#include "wx/log.h"

#endif