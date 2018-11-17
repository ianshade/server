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
#include <mutex>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#include <stdlib.h>
#endif

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

const std::wstring& dll_name()
{
    static std::wstring name = u16(NDILIB_LIBRARY_NAME);

    return name;
}

static std::mutex                              find_instance_mutex;
static std::shared_ptr<NDIlib_find_instance_t> find_instance;

NDIlib_v3* load_library()
{
    static NDIlib_v3* ndi_lib = nullptr;

    if (ndi_lib)
        return ndi_lib;
    const char* runtime_dir = getenv(NDILIB_REDIST_FOLDER);

    if (runtime_dir == NULL)
        return nullptr;

    auto dll_path = boost::filesystem::path(runtime_dir) / NDILIB_LIBRARY_NAME;

#ifdef _WIN32
    auto module = LoadLibrary(dll_path.c_str());

    if (!module)
        return nullptr;

    static std::shared_ptr<void> lib(module, FreeLibrary);

    //    wchar_t actualFilename[256];
    //    GetModuleFileNameW(module, actualFilename, sizeof(actualFilename));
    CASPAR_LOG(info) << L"Loaded " << dll_path;

    auto NDIlib_v3_load = GetProcAddress(module, "NDIlib_v3_load");
    if (!NDIlib_v3_load)
        return nullptr;

#else
    // Try to load the library
    void* hNDILib = dlopen(dll_path.c_str(), RTLD_LOCAL | RTLD_LAZY);

    // The main NDI entry point for dynamic loading if we got the library
    const NDIlib_v3* (*NDIlib_v3_load)(void) = NULL;
    if (hNDILib) {
        CASPAR_LOG(info) << L"Loaded " << dll_path;
        static std::shared_ptr<void> lib(hNDILib, dlclose);
        *((void**)&NDIlib_v3_load) = dlsym(hNDILib, "NDIlib_v3_load");
    }
    if (!NDIlib_v3_load)
        return nullptr;

#endif

    ndi_lib = (NDIlib_v3*)(NDIlib_v3_load());

    if (!ndi_lib->NDIlib_initialize()) {
        not_initialized();
    }

#ifdef _WIN32
    // these functions have to be loaded this way because they aren't in NDIlib_v3 struct
    fs_create  = reinterpret_cast<decltype(fs_create)>(GetProcAddress(module, "NDIlib_framesync_create"));
    fs_destroy = reinterpret_cast<decltype(fs_destroy)>(GetProcAddress(module, "NDIlib_framesync_destroy"));
    fs_capture_audio =
        reinterpret_cast<decltype(fs_capture_audio)>(GetProcAddress(module, "NDIlib_framesync_capture_audio"));
    fs_free_audio = reinterpret_cast<decltype(fs_free_audio)>(GetProcAddress(module, "NDIlib_framesync_free_audio"));
    fs_capture_video =
        reinterpret_cast<decltype(fs_capture_video)>(GetProcAddress(module, "NDIlib_framesync_capture_video"));
    fs_free_video = reinterpret_cast<decltype(fs_free_video)>(GetProcAddress(module, "NDIlib_framesync_free_video"));

#else
    *((void**)&fs_create)        = dlsym(hNDILib, "NDIlib_framesync_create");
    *((void**)&fs_destroy)       = dlsym(hNDILib, "NDIlib_framesync_destroy");
    *((void**)&fs_capture_audio) = dlsym(hNDILib, "NDIlib_framesync_capture_audio");
    *((void**)&fs_free_audio)    = dlsym(hNDILib, "NDIlib_framesync_free_audio");
    *((void**)&fs_capture_video) = dlsym(hNDILib, "NDIlib_framesync_capture_video");
    *((void**)&fs_free_video)    = dlsym(hNDILib, "NDIlib_framesync_free_video");

#endif
    find_instance.reset(new NDIlib_find_instance_t(ndi_lib->NDIlib_find_create_v2(nullptr)),
                        [](NDIlib_find_instance_t* p) { ndi_lib->NDIlib_find_destroy(*p); });
    return ndi_lib;
}

std::map<std::string, NDIlib_source_t> get_current_sources()
{
    auto                        sources_map = std::map<std::string, NDIlib_source_t>();
    uint32_t                    no_sources;
    std::lock_guard<std::mutex> guard(find_instance_mutex);
    const NDIlib_source_t*      sources =
        load_library()->NDIlib_find_get_current_sources(*(find_instance.get()), &no_sources);
    for (uint32_t i = 0; i < no_sources; i++) {
        sources_map.emplace(std::string(sources[i].p_ndi_name), sources[i]);
    }
    return sources_map;
}

void not_installed()
{
    CASPAR_THROW_EXCEPTION(not_supported()
                           << msg_info(dll_name() + L" not available. Install NDI Redist version 3.7 or higher from " +
                                       u16(NDILIB_REDIST_URL)));
}

void not_initialized()
{
    CASPAR_THROW_EXCEPTION(not_supported() << msg_info("Unable to initialize NDI on this system."));
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
