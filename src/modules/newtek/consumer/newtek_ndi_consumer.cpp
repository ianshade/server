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
 * based on work of Robert Nagy, ronag89@gmail.com
 */

#include "../StdAfx.h"

#include "newtek_ndi_consumer.h"

#include <core/consumer/frame_consumer.h>
#include <core/frame/frame.h>
#include <core/mixer/audio/audio_util.h>
#include <core/video_format.h>

#include <common/assert.h>
#include <common/diagnostics/graph.h>
#include <common/future.h>
#include <common/param.h>
#include <common/timer.h>

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>

#include <atomic>

#include "../util/ndi.h"

namespace caspar { namespace newtek {

struct newtek_ndi_consumer : public core::frame_consumer
{
    core::video_format_desc              format_desc_;
    NDIlib_v3*                           ndi_lib_;
    NDIlib_send_instance_t               ndi_send_instance_;
    NDIlib_video_frame_v2_t              ndi_video_frame_;
    NDIlib_audio_frame_interleaved_32f_t ndi_audio_frame_;
    spl::shared_ptr<diagnostics::graph>  graph_;
    caspar::timer                        tick_timer_;
    caspar::timer                        frame_timer_;
    std::wstring                         name_;

  public:
    newtek_ndi_consumer(std::wstring name)
        : name_(name)
    {
        if (!ndi::load_library()) {
            CASPAR_THROW_EXCEPTION(not_supported() << msg_info(
                                       ndi::dll_name() + L" not available. Install NDI Redist version 3.7 or higher."));
        }

        graph_->set_text(print());
        graph_->set_color("frame-time", diagnostics::color(0.5f, 1.0f, 0.2f));
        graph_->set_color("tick-time", diagnostics::color(0.0f, 0.6f, 0.9f));
        graph_->set_color("dropped-frame", diagnostics::color(0.3f, 0.6f, 0.3f));
        diagnostics::register_graph(graph_);
    }

    ~newtek_ndi_consumer() {}

    // frame_consumer

    void initialize(const core::video_format_desc& format_desc, int channel_index) override
    {
        format_desc_ = format_desc;

        ndi_lib_ = ndi::load_library();
        NDIlib_send_create_t NDI_send_create_desc;
        auto                 tmp_name    = u8(name_);
        NDI_send_create_desc.p_ndi_name  = tmp_name.c_str();
        //NDI_send_create_desc.clock_video = false;
        //NDI_send_create_desc.clock_audio = false;
        ndi_send_instance_               = ndi_lib_->NDIlib_send_create(&NDI_send_create_desc);

        ndi_video_frame_.xres                 = format_desc.width;
        ndi_video_frame_.yres                 = format_desc.height;
        ndi_video_frame_.frame_rate_N         = format_desc.framerate.numerator();
        ndi_video_frame_.frame_rate_D         = format_desc.framerate.denominator();
        ndi_video_frame_.FourCC               = NDIlib_FourCC_type_BGRA;
        ndi_video_frame_.line_stride_in_bytes = format_desc.width * 4;

        ndi_audio_frame_.sample_rate = format_desc_.audio_sample_rate;
        ndi_audio_frame_.no_channels = format_desc_.audio_channels;
        ndi_audio_frame_.timecode    = NDIlib_send_timecode_synthesize;

        CASPAR_VERIFY(ndi_send_instance_);
    }

    std::future<bool> send(core::const_frame frame) override
    {
        CASPAR_VERIFY(format_desc_.height * format_desc_.width * 4 == frame.image_data(0).size());

        graph_->set_value("tick-time", tick_timer_.elapsed() * format_desc_.fps * 0.5);
        tick_timer_.restart();
        frame_timer_.restart();

        auto audio_data                 = frame.audio_data();
        int  audio_data_size            = static_cast<int>(audio_data.size());
        ndi_audio_frame_.no_samples     = audio_data_size / format_desc_.audio_channels;
        std::vector<float> audio_buffer = ndi::audio_32_to_32f(audio_data.data(), audio_data_size);
        ndi_audio_frame_.p_data         = const_cast<float*>(audio_buffer.data());
        ndi_lib_->NDIlib_util_send_send_audio_interleaved_32f(ndi_send_instance_, &ndi_audio_frame_);

        ndi_video_frame_.p_data = const_cast<uint8_t*>(frame.image_data(0).begin());
        ndi_lib_->NDIlib_send_send_video_v2(ndi_send_instance_, &ndi_video_frame_);

        graph_->set_value("frame-time", frame_timer_.elapsed() * format_desc_.fps * 0.5);

        return make_ready_future(true);
    }

    std::wstring print() const override
    {
        return L"newtek-ndi[" + name_ + L"]";
    } // TODO: maybe put tally status in the name

    std::wstring name() const override { return L"newtek-ndi"; }

    int index() const override { return 900; }

    bool has_synchronization_clock() const override { return false; }
};

spl::shared_ptr<core::frame_consumer> create_ndi_consumer(const std::vector<std::wstring>&                  params,
                                                          std::vector<spl::shared_ptr<core::video_channel>> channels)
{
    if (params.size() < 1 || !boost::iequals(params.at(0), L"NEWTEK_NDI"))
        return core::frame_consumer::empty();
    std::wstring name = L"CasparCG";
    if (contains_param(L"NAME", params)) {
        name = get_param(L"NAME", params);
    }
    return spl::make_shared<newtek_ndi_consumer>(name);
}

spl::shared_ptr<core::frame_consumer>
create_preconfigured_ndi_consumer(const boost::property_tree::wptree&               ptree,
                                  std::vector<spl::shared_ptr<core::video_channel>> channels)
{
    std::wstring name = L"CasparCG";
    name              = ptree.get(L"name", name);
    return spl::make_shared<newtek_ndi_consumer>(name);
}

}} // namespace caspar::newtek
