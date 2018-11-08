/*
 * Copyright 2018
 *
 * This file is part of CasparCG (www.casparcg.com).
 *
 * CasparCG is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CasparCG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with CasparCG. If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Krzysztof Zegzula, zegzulakrzysztof@gmail.com
 */

#include "../StdAfx.h"

#include "ndi.h"

#include <memory>

#include <Windows.h>

#include <common/except.h>

#include <boost/filesystem.hpp>

namespace caspar { namespace newtek { namespace ndi {

NDIlib_framesync_instance_t (*fs_create)(NDIlib_recv_instance_t p_receiver) = nullptr;
void (*fs_destroy)(NDIlib_framesync_instance_t p_instance)                  = nullptr;
void (*fs_capture_audio)(NDIlib_framesync_instance_t p_instance,
                         NDIlib_audio_frame_v2_t*    p_audio_data,
                         int                         sample_rate,
                         int                         no_channels,
                         int                         no_samples)                                    = nullptr;

void (*fs_free_audio)(NDIlib_framesync_instance_t p_instance, NDIlib_audio_frame_v2_t* p_audio_data) = nullptr;

void (*fs_capture_video)(NDIlib_framesync_instance_t p_instance,
                         NDIlib_video_frame_v2_t*    p_video_data,
                         NDIlib_frame_format_type_e  field_type) = nullptr;

void (*fs_free_video)(NDIlib_framesync_instance_t p_instance, NDIlib_video_frame_v2_t* p_video_data) = nullptr;
const std::wstring& dll_name();
NDIlib_v3*          load_library();

const std::wstring& dll_name()
{
    static std::wstring name = L"Processing.NDI.Lib.x64.dll";

    return name;
}

NDIlib_v3* load_library()
{
    static NDIlib_v3* ndi_lib = nullptr;

    if (ndi_lib)
        return ndi_lib;
    auto runtime_dir = _wgetenv(L"NDI_RUNTIME_DIR_V3");

    if (runtime_dir == NULL)
        return nullptr;
    auto dll_path = boost::filesystem::path(runtime_dir) / dll_name();
    auto module   = LoadLibrary(dll_path.c_str());

    if (!module)
        return nullptr;

    static std::shared_ptr<void> lib(module, FreeLibrary);

    wchar_t actualFilename[256];
    GetModuleFileNameW(module, actualFilename, sizeof(actualFilename));
    CASPAR_LOG(info) << L"Loaded " << actualFilename;

    auto NDIlib_v3_load = GetProcAddress(module, "NDIlib_v3_load");
    ndi_lib             = (NDIlib_v3*)(NDIlib_v3_load());

    // these functions have to be loaded this way because they aren't in NDIlib_v3 struct
    fs_create  = reinterpret_cast<decltype(fs_create)>(GetProcAddress(module, "NDIlib_framesync_create"));
    fs_destroy = reinterpret_cast<decltype(fs_destroy)>(GetProcAddress(module, "NDIlib_framesync_destroy"));
    fs_capture_audio =
        reinterpret_cast<decltype(fs_capture_audio)>(GetProcAddress(module, "NDIlib_framesync_capture_audio"));
    fs_free_audio = reinterpret_cast<decltype(fs_free_audio)>(GetProcAddress(module, "NDIlib_framesync_free_audio"));
    fs_capture_video =
        reinterpret_cast<decltype(fs_capture_video)>(GetProcAddress(module, "NDIlib_framesync_capture_video"));
    fs_free_video = reinterpret_cast<decltype(fs_free_video)>(GetProcAddress(module, "NDIlib_framesync_free_video"));
    return ndi_lib;
}

std::vector<int32_t> audio_16_to_32(const short* audio_data, int size)
{
    auto output32 = std::vector<int32_t>();

    output32.reserve(size);
    for (int n = 0; n < size; ++n)
        output32.push_back((audio_data[n] & 0xFFFFFFFF) << 16);

    return output32;
}

std::vector<float> audio_32_to_32f(const int* audio_data, int size)
{
    auto output32 = std::vector<float>();

    output32.reserve(size);
    for (int n = 0; n < size; ++n)
        output32.push_back((1.0f * audio_data[n]) / INT32_MAX);

    return output32;
}

}}} // namespace caspar::newtek::ndi
