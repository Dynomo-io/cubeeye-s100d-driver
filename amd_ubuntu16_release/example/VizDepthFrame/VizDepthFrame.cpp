/*
 * ReadDepthFrame.cpp
 *
 *  Created on: 2018. 10. 15.
 *      Author: erato
 */

#include <tuple>
#include <mutex>
#include <thread>
#include <queue>
#include <array>
#include <vector>
#include <atomic>
#include <iostream>
#include <functional>
#include <condition_variable>

#include <assert.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include "opencv2/imgproc/types_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"

#include "CubeEyeSink.h"
#include "CubeEyeCamera.h"
#include "CubeEyeBasicFrame.h"

inline cv::Mat frame_cast_to_mat(const meere::sensor::sptr_frame& frame)
{
	if (meere::sensor::CubeEyeFrame::FrameType_Depth == frame->frameType() ||
			meere::sensor::CubeEyeFrame::FrameType_Amplitude == frame->frameType()) {
		cv::Mat _mat;
		if (meere::sensor::CubeEyeData::DataType_16U == frame->frameDataType()) {
			auto _sptr_frame = meere::sensor::frame_cast_basic16u(frame);
			auto _ptr_frame_data = _sptr_frame->frameData();
			_mat = cv::Mat(frame->frameHeight(), frame->frameWidth(), CV_16UC1, const_cast<meere::sensor::int16u*>(_ptr_frame_data->data()));
		}
		else if (meere::sensor::CubeEyeData::DataType_32F == frame->frameDataType()) {
			auto _sptr_frame = meere::sensor::frame_cast_basic32f(frame);
			auto _ptr_frame_data = _sptr_frame->frameData();
			_mat = cv::Mat(frame->frameHeight(), frame->frameWidth(), CV_32FC1, const_cast<meere::sensor::flt32*>(_ptr_frame_data->data()));
		}
		else {
			return cv::Mat();
		}

		return _mat;
	}

	return cv::Mat();
}

static class ReceivedDepthFrameSink : public meere::sensor::sink
 , public meere::sensor::prepared_listener
{
public:
	virtual std::string name() const {
		return std::string("ReceivedDepthFrameSink");
	}

	virtual void onCubeEyeCameraState(const meere::sensor::ptr_source source, meere::sensor::State state) {
		printf("%s:%d source(%s) state = %d\n", __FUNCTION__, __LINE__, source->uri().c_str(), state);
		
		if (meere::sensor::State::Running == state) {
			mReadFrameThreadStart = true;
			mReadFrameThread = std::thread(ReceivedDepthFrameSink::ReadFrameProc, this);
		}
		else if (meere::sensor::State::Stopped == state) {
			mReadFrameThreadStart = false;
			if (mReadFrameThread.joinable()) {
				mReadFrameThread.join();
			}
		}
	}

	virtual void onCubeEyeCameraError(const meere::sensor::ptr_source source, meere::sensor::Error error) {
		printf("%s:%d source(%s) error = %d\n", __FUNCTION__, __LINE__, source->uri().c_str(), error);
	}

	virtual void onCubeEyeFrameList(const meere::sensor::ptr_source source , const meere::sensor::sptr_frame_list& frames) {
		if (mReadFrameThreadStart) {
			static constexpr size_t _MAX_FRAMELIST_SIZE = 4;
			if (_MAX_FRAMELIST_SIZE > mFrameListQueue.size()) {
				auto _copied_frame_list = meere::sensor::copy_frame_list(frames);
				if (_copied_frame_list) {
					mFrameListQueue.push(std::move(_copied_frame_list));
				}
			}
		}
	}

public:
	virtual void onCubeEyeCameraPrepared(const meere::sensor::ptr_camera camera) {
		printf("%s:%d source(%s)\n", __FUNCTION__, __LINE__, camera->source()->uri().c_str());
	}

protected:
	static void ReadFrameProc(ReceivedDepthFrameSink* thiz) {
		while (thiz->mReadFrameThreadStart) {
			if (thiz->mFrameListQueue.empty()) {
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				continue;
			}

			auto _frames = std::move(thiz->mFrameListQueue.front());
			thiz->mFrameListQueue.pop();

			auto _frame_z = meere::sensor::find_frame(_frames, meere::sensor::CubeEyeFrame::FrameType_Depth);
			auto _frame_a = meere::sensor::find_frame(_frames, meere::sensor::CubeEyeFrame::FrameType_Amplitude);

			if (nullptr != _frame_z && nullptr != _frame_a) {
				// convert from unsigned short frame data to CV_16UC1
				cv::Mat _mat_z = frame_cast_to_mat(_frame_z);
				cv::Mat _mat_a = frame_cast_to_mat(_frame_a);

				cv::Mat _convert_z, _convert_a;

				// convert from CV_16UC1 to CV_8UC1
				_mat_z.convertTo(_convert_z, CV_8UC1);
				_mat_a.convertTo(_convert_a, CV_8UC1);

				cv::Mat _color_z, _color_a;

				// apply opencv colormap and show
				cv::applyColorMap(_convert_z, _color_z, cv::COLORMAP_RAINBOW);
				cv::applyColorMap(_convert_a, _color_a, cv::COLORMAP_BONE);

				cv::imshow("Depth", _color_z);
				cv::imshow("Amplitude", _color_a);

				cv::waitKey(1);
			}
		}
	}

public:
	ReceivedDepthFrameSink() = default;
	virtual ~ReceivedDepthFrameSink() = default;

protected:
	bool mReadFrameThreadStart;
	std::thread mReadFrameThread;
	std::queue<meere::sensor::sptr_frame_list> mFrameListQueue;
} _ReceivedDepthFrameSink;


static bool m_break = false;

void OnSignal(int sig)
{
    signal(sig, SIG_IGN);
    std::cout << "'Ctrl + C' Pressed. Exit program~!!" << std::endl;
    m_break = true;
}

int main(int argc, char* argv[])
{
	signal(SIGINT, OnSignal);

	// search ToF camera source
	int _selected_source = -1;
	meere::sensor::sptr_source_list _source_list = meere::sensor::search_camera_source();

	if (nullptr != _source_list) {
		int i = 0;
		for (auto it : (*_source_list)) {
			std::cout << i++ << ") source name : " << it->name() << \
					", serialNumber : " << it->serialNumber() << \
					", uri : " << it->uri() << std::endl;
		}
	}

	if (nullptr != _source_list && 0 < _source_list->size()) {
		if (1 < _source_list->size()) {
			std::cout << "Please enter the desired source number." << std::endl;
			scanf("%d", &_selected_source);
			getchar();
		}
		else {
			_selected_source = 0;
		}
	}
	else {
		std::cout << "no search device!" << std::endl;
		return -1;
	}

	if (0 > _selected_source) {
		std::cout << "invalid selected source number!" << std::endl;
		return -1;
	}

	meere::sensor::add_prepared_listener(&_ReceivedDepthFrameSink);

	// create ToF camera
	meere::sensor::result _rt;
	meere::sensor::sptr_camera _camera = meere::sensor::create_camera(_source_list->at(_selected_source));
	if (nullptr != _camera) {
		_camera->addSink(&_ReceivedDepthFrameSink);

		// get lens parameters
		{
			auto _lenses = _camera->lenses();
			std::cout << "count of Lenses : " << _lenses << std::endl;
			for (size_t i = 0; i < _lenses; i++) {
				std::cout << "Lens index : " << i << std::endl;

				auto _fov = _camera->fov(i);
				std::cout << "	FoV : " << std::get<0>(_fov) << "(H) x " << std::get<1>(_fov) << "(V)" << std::endl;

				meere::sensor::IntrinsicParameters parameters;
				if (meere::sensor::success == (_rt = _camera->intrinsicParameters(parameters, i))) {
					std::cout << "	IntrinsicParameters :" << std::endl;
					std::cout << "		ForcalLength(fx) = " << parameters.forcal.fx << std::endl;
					std::cout << "		ForcalLength(fy) = " << parameters.forcal.fy << std::endl;
					std::cout << "		PrincipalPoint(cx) = " << parameters.principal.cx << std::endl;
					std::cout << "		PrincipalPoint(cy) = " << parameters.principal.cy << std::endl;
				}

				meere::sensor::DistortionCoefficients coefficients;
				if (meere::sensor::success == (_rt = _camera->distortionCoefficients(coefficients, i))) {
					std::cout << "	DistortionCoefficients :" << std::endl;
					std::cout << "		RadialCoefficient(K1) = " << coefficients.radial.k1 << std::endl;
					std::cout << "		RadialCoefficient(K2) = " << coefficients.radial.k2 << std::endl;
					std::cout << "		RadialCoefficient(K3) = " << coefficients.radial.k3 << std::endl;
					std::cout << "		TangentialCoefficient(P1) = " << coefficients.tangential.p1 << std::endl;
					std::cout << "		TangentialCoefficient(P2) = " << coefficients.tangential.p2 << std::endl;
					std::cout << "		skewCoefficient = " << coefficients.skewCoefficient << std::endl;
				}
			}
		}

		_rt = _camera->prepare();
		assert(meere::sensor::success == _rt);
		if (meere::sensor::success != _rt) {
			std::cout << "_camera->prepare() failed." << std::endl;
			meere::sensor::destroy_camera(_camera);
			return -1;
		}

		// set wanted frame type : depth & amplitude
		int _wantedFrame = meere::sensor::CubeEyeFrame::FrameType_Depth | meere::sensor::CubeEyeFrame::FrameType_Amplitude;

		_rt = _camera->run(_wantedFrame);
		assert(meere::sensor::success == _rt);
		if (meere::sensor::success != _rt) {
			std::cout << "_camera->run() failed." << std::endl;
			meere::sensor::destroy_camera(_camera);
			return -1;
		}

		while (!m_break) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		_camera->stop();
		_camera->release();
		meere::sensor::destroy_camera(_camera);
		_camera.reset();
	}

	return 0;
}
