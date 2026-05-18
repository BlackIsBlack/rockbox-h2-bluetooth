/******************************************************************************
 *
 *  Copyright (C) 2026 Skye Green
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#include "a2d_int.h"
#include "common/bt_target.h"

#include "stack/a2d_api.h"
#include "stack/a2d_codecs.h"

#if (defined(A2D_INCLUDED) && A2D_INCLUDED == TRUE)

tA2D_CODEC A2D_GetCodecType(const uint8_t *p_info) {
    uint8_t losc;
    uint8_t media_type;
    uint8_t codec_type;
    uint32_t vendor_id;
    uint16_t codec_id;

    if (p_info == NULL) {
        return A2D_CODEC_INVALID;
    }

    losc = *p_info++;
    if (losc < 2) {
        A2D_TRACE_DEBUG("A2D_GetCodecType: losc too short (got %d)", losc);
        return A2D_CODEC_INVALID;
    }
    A2D_TRACE_DEBUG("A2D_GetCodecType: losc is %d", losc);

    media_type = *p_info++;
    codec_type = *p_info++;
    A2D_TRACE_DEBUG("A2D_GetCodecType: media_type is %d", media_type);
    A2D_TRACE_DEBUG("A2D_GetCodecType: codec_type is %d", codec_type);
    if (media_type != A2D_MEDIA_TYPE_AUDIO) {
        A2D_TRACE_DEBUG("A2D_GetCodecType: media type not Audio (got %d)", media_type);
        return A2D_CODEC_INVALID;
    }

    if (codec_type != 0xFF) {
        return 0x003F0000 | codec_type;
    }

    if (losc < 8) {
        A2D_TRACE_DEBUG("A2D_GetCodecType: LOSC too short (got %d)", losc);
        return A2D_CODEC_INVALID;
    }

    vendor_id = (*p_info & 0x000000FF) | (*(p_info + 1) << 8 & 0x0000FF00) |
                    (*(p_info + 2) << 16 & 0x00FF0000) |
                    (*(p_info + 3) << 24 & 0xFF000000);
    p_info += 4;
    codec_id = (*p_info & 0x00FF) | (*(p_info + 1) << 8 & 0xFF00);
    p_info += 2;

    if (vendor_id > 0xFFFF || vendor_id == 0x003F) {
        return A2D_CODEC_INVALID;
    }

    return (vendor_id << 16) | codec_id;
}

#endif /* #if (defined(A2D_INCLUDED) && A2D_INCLUDED == TRUE) */
