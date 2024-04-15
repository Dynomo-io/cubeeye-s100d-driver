/*
 * ReadIntensityPCLFrame.cpp
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

#include "CubeEyeSink.h"
#include "CubeEyeCamera.h"
#include "CubeEyeIntensityPointCloudFrame.h"

static class ReceivedIntensityPCLFrameSink : public meere::sensor::sink
 , public meere::sensor::prepared_listener
{
public:
	virtual std::string name() const {
		return std::string("ReceivedIntensityPCLFrameSink");
	}

	virtual void onCubeEyeCameraState(const meere::sensor::ptr_source source, meere::sensor::State state) {
		printf("%s:%d source(%s) state = %d\n", __FUNCTION__, __LINE__, source->uri().c_str(), state);
	}

	virtual void onCubeEyeCameraError(const meere::sensor::ptr_source source, meere::sensor::Error error) {
		printf("%s:%d source(%s) error = %d\n", __FUNCTION__, __LINE__, source->uri().c_str(), error);
	}

	virtual void onCubeEyeFrameList(const meere::sensor::ptr_source source , const meere::sensor::sptr_frame_list& frames) {
		static int _frame_cnt = 0;
		if (30 > ++_frame_cnt) {
			return;
		}
		_frame_cnt = 0;

		for (auto it : (*frames)) {
			printf("frame : %d, "
					"frameWidth = %d "
					"frameHeight = %d "
					"frameDataType = %d "
					"timestamp = %lu \n",
					it->frameType(),
					it->frameWidth(),
					it->frameHeight(),
					it->frameDataType(),
					it->timestamp());
			
			int _frame_index = 0;
			auto _center_x = it->frameWidth() / 2;
			auto _center_y = it->frameHeight() / 2;

			// intensity-PointCloud frame
			if (it->frameType() == meere::sensor::CubeEyeFrame::FrameType_IntensityPointCloud) {
				// 32bits floating-point
				if (it->frameDataType() == meere::sensor::CubeEyeData::DataType_32F) {
					// casting 32bits intensity point cloud frame
					auto _sptr_intensity_pointcloud_frame = meere::sensor::frame_cast_ipcl32f(it);
					auto _sptr_frame_dataX = _sptr_intensity_pointcloud_frame->frameDataX(); // x-point data array
					auto _sptr_frame_dataY = _sptr_intensity_pointcloud_frame->frameDataY(); // y-point data array
					auto _sptr_frame_dataZ = _sptr_intensity_pointcloud_frame->frameDataZ(); // z-point data array
					auto _sptr_frame_dataI = _sptr_intensity_pointcloud_frame->frameDataI(); // amplitude data array

					for (int y = 0 ; y < _sptr_intensity_pointcloud_frame->frameHeight(); y++) {
						for (int x = 0 ; x < _sptr_intensity_pointcloud_frame->frameWidth(); x++) {
							_frame_index = y * _sptr_intensity_pointcloud_frame->frameWidth() + x;
							if (_center_x == x && _center_y == y) {
								printf("intensity-PCL(%d,%d) data : %f, %f, %f, %f\n", _center_x, _center_y, \
								(*_sptr_frame_dataX)[_frame_index] * 1000, (*_sptr_frame_dataY)[_frame_index] * 1000, \
								(*_sptr_frame_dataZ)[_frame_index] * 1000, (*_sptr_frame_dataI)[_frame_index] * 1000);
							}
						}
					}
				}
			}
		}
	}

public:
	virtual void onCubeEyeCameraPrepared(const meere::sensor::ptr_camera camera) {
		printf("%s:%d source(%s)\n", __FUNCTION__, __LINE__, camera->source()->uri().c_str());
	}

public:
	ReceivedIntensityPCLFrameSink() = default;
	virtual ~ReceivedIntensityPCLFrameSink() = default;
} _ReceivedIntensityPCLFrameSink;


int main(int argc, char* argv[])
{
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

	meere::sensor::add_prepared_listener(&_ReceivedIntensityPCLFrameSink);

	// create ToF camera
	meere::sensor::sptr_camera _camera = meere::sensor::create_camera(_source_list->at(_selected_source));
	if (nullptr != _camera) {
		_camera->addSink(&_ReceivedIntensityPCLFrameSink);

		meere::sensor::result _rt;
		_rt = _camera->prepare();
		assert(meere::sensor::result::success == _rt);
		if (meere::sensor::result::success != _rt) {
			std::cout << "_camera->prepare() failed." << std::endl;
			meere::sensor::destroy_camera(_camera);
			return -1;
		}

		// set wanted frame type : intensity-PCL
		int _wantedFrame = meere::sensor::CubeEyeFrame::FrameType_IntensityPointCloud;


		_rt = _camera->run(_wantedFrame);
		assert(meere::sensor::result::success == _rt);
		if (meere::sensor::result::success != _rt) {
			std::cout << "_camera->run() failed." << std::endl;
			meere::sensor::destroy_camera(_camera);
			return -1;
		}

		std::cout << "*** run example and terminated after 5 seconds. ***" << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(5000));

		_camera->stop();
		_camera->release();
		meere::sensor::destroy_camera(_camera);
		_camera.reset();
	}

	return 0;
}
