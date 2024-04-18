/*
 * SetGetProperty.cpp
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
#include <limits>
#include <iostream>
#include <inttypes.h>
#include <functional>
#include <condition_variable>

#include <assert.h>
#include <string.h>
#include <unistd.h>

#include "CubeEyeSink.h"
#include "CubeEyeCamera.h"
#include "CubeEyeBasicFrame.h"

void printResult(const std::string& item, const meere::sensor::result& rt)
{
	std::cout << item << " -> result : ";
	switch (rt) {
	case meere::sensor::result::success:
		std::cout << "success" << std::endl;
		break;
	case meere::sensor::result::fail:
		std::cout << "fail" << std::endl;
		break;
	case meere::sensor::result::empty:
		std::cout << "empty" << std::endl;
		break;
	case meere::sensor::result::overflow:
		std::cout << "overflow" << std::endl;
		break;
	case meere::sensor::result::not_found:
		std::cout << "not found" << std::endl;
		break;
	case meere::sensor::result::not_ready:
		std::cout << "not ready" << std::endl;
		break;
	case meere::sensor::result::not_supported:
		std::cout << "not supported" << std::endl;
		break;
	case meere::sensor::result::not_implemented:
		std::cout << "not implemented" << std::endl;
		break;
	case meere::sensor::result::no_such_device:
		std::cout << "no such device" << std::endl;
		break;
	case meere::sensor::result::invalid_parameter:
		std::cout << "invalid parameter" << std::endl;
		break;
	case meere::sensor::result::invalid_operation:
		std::cout << "invalid operation" << std::endl;
		break;
	case meere::sensor::result::invalid_data_type:
		std::cout << "invalid data type" << std::endl;
		break;
	case meere::sensor::result::out_of_memory:
		std::cout << "out of memory" << std::endl;
		break;
	case meere::sensor::result::out_of_resource:
		std::cout << "out of resource" << std::endl;
		break;
	case meere::sensor::result::out_of_range:
		std::cout << "out of range" << std::endl;
		break;
	case meere::sensor::result::already_exists:
		std::cout << "already exists" << std::endl;
		break;
	case meere::sensor::result::already_opened:
		std::cout << "already opened" << std::endl;
		break;
	case meere::sensor::result::already_running:
		std::cout << "already running" << std::endl;
		break;
	case meere::sensor::result::already_initialized:
		std::cout << "already initialized" << std::endl;
		break;
	case meere::sensor::result::using_resources:
		std::cout << "using resources" << std::endl;
		break;
	case meere::sensor::result::timeout:
		std::cout << "timeout" << std::endl;
		break;
	default:
		break;
	}
}

void printProperty(const meere::sensor::sptr_property& prop)
{
	if (nullptr != prop && !prop->key().empty()) {
		std::cout << prop->key() << " -> result : ";
		switch (prop->dataType()) {
		case meere::sensor::CubeEyeData::DataType_Boolean:
			printf("%s\n", prop->asBoolean() ? "true" : "false");
			break;
		case meere::sensor::CubeEyeData::DataType_8S:
			printf("%hhd\n", prop->asInt8s());
			break;
		case meere::sensor::CubeEyeData::DataType_8U:
			printf("%hhu\n", prop->asInt8u());
			break;
		case meere::sensor::CubeEyeData::DataType_16S:
			printf("%hd\n", prop->asInt16s());
			break;
		case meere::sensor::CubeEyeData::DataType_16U:
			printf("%hu\n", prop->asInt16u());
			break;
		case meere::sensor::CubeEyeData::DataType_32S:
			printf("%d\n", prop->asInt32s());
			break;
		case meere::sensor::CubeEyeData::DataType_32U:
			printf("%u\n", prop->asInt32u());
			break;
		case meere::sensor::CubeEyeData::DataType_32F:
			printf("%f\n", prop->asFlt32());
			break;
		case meere::sensor::CubeEyeData::DataType_64F:
			printf("%f\n", prop->asFlt64());
			break;
		case meere::sensor::CubeEyeData::DataType_64S:
			printf("%" PRId64 "\n", prop->asInt64s());
			break;
		case meere::sensor::CubeEyeData::DataType_64U:
			printf("%" PRIu64 "\n", prop->asInt64u());
			break;
		case meere::sensor::CubeEyeData::DataType_String:
			std::cout << prop->asString() << std::endl;
			break;
		default:
			std::cout << std::endl;
			break;
		}
	}
};


static class ReceivedFrameSink : public meere::sensor::sink
 , public meere::sensor::prepared_listener
{
public:
	virtual std::string name() const {
		return std::string("ReceivedFrameSink");
	}

	virtual void onCubeEyeCameraState(const meere::sensor::ptr_source source, meere::sensor::State state) {
		printf("%s:%d source(%s) state = %d\n", __FUNCTION__, __LINE__, source->uri().c_str(), state);
	}

	virtual void onCubeEyeCameraError(const meere::sensor::ptr_source source, meere::sensor::Error error) {
		printf("%s:%d source(%s) error = %d\n", __FUNCTION__, __LINE__, source->uri().c_str(), error);
	}

	virtual void onCubeEyeFrameList(const meere::sensor::ptr_source source , const meere::sensor::sptr_frame_list& frames) {
	}

public:
	virtual void onCubeEyeCameraPrepared(const meere::sensor::ptr_camera camera) {
		printf("%s:%d source(%s)\n", __FUNCTION__, __LINE__, camera->source()->uri().c_str());
	}

public:
	ReceivedFrameSink() = default;
	virtual ~ReceivedFrameSink() = default;
} _ReceivedFrameSink;


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

	meere::sensor::add_prepared_listener(&_ReceivedFrameSink);

	// create ToF camera
	meere::sensor::sptr_camera _camera = meere::sensor::create_camera(_source_list->at(_selected_source));
	if (nullptr != _camera) {
		_camera->addSink(&_ReceivedFrameSink);

		meere::sensor::result _rt;
		_rt = _camera->prepare();
		assert(meere::sensor::result::success == _rt);
		if (meere::sensor::result::success != _rt) {
			std::cout << "_camera->prepare() failed." << std::endl;
			meere::sensor::destroy_camera(_camera);
			return -1;
		}

		// set wanted frame type : depth
		int _wantedFrame = meere::sensor::CubeEyeFrame::FrameType_Depth;

		_rt = _camera->run(_wantedFrame);
		assert(meere::sensor::result::success == _rt);
		if (meere::sensor::result::success != _rt) {
			std::cout << "_camera->run() failed." << std::endl;
			meere::sensor::destroy_camera(_camera);
			return -1;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(3000));

		// last released version/date
		printf("\r*** v%s : %s ***\n", meere::sensor::last_released_version().c_str(), meere::sensor::last_released_date().c_str());

		std::string _key("");
		meere::sensor::sptr_property _prop;
		meere::sensor::result_property _rt_prop;

		// SET : log level
		{
			_key = "log_level";
			_prop = meere::sensor::make_property_32s(_key, 0);	// verbose level or higher
			_rt = meere::sensor::set_property(_prop);
			printResult(_key, _rt);	
		}

		// SET : log level
		{
			_key = "log_level";
			_prop = meere::sensor::make_property_32s(_key, 4);	// error level
			_rt = meere::sensor::set_property(_prop);
			printResult(_key, _rt);
		}

		// SET : framerate : 15
		{
			_key = "framerate";
			_prop = meere::sensor::make_property_8u(_key, 15);
			_rt = _camera->setProperty(_prop);
			printResult(_key, _rt);
		}

		// GET : framerate.
		{
			_key = "framerate";
			_rt_prop = _camera->getProperty(_key);
			if (meere::sensor::success == std::get<0>(_rt_prop)) {
				printProperty(std::get<1>(_rt_prop));
			}
			else {
				printResult(_key, std::get<0>(_rt_prop));	
			}
		}

		// GET : amplitude_threshold_min 
		{
			_key = "amplitude_threshold_min";
			_rt_prop = _camera->getProperty(_key);
			if (meere::sensor::success == std::get<0>(_rt_prop)) {
				printProperty(std::get<1>(_rt_prop));
			}
			else {
				printResult(_key, std::get<0>(_rt_prop));	
			}
		}

		// SET : amplitude_threshold_min 
		{
			_key = "amplitude_threshold_min";
			_prop = meere::sensor::make_property_8u(_key, 100);
			_rt = _camera->setProperty(_prop);
			printResult(_key, _rt);	
		}

		// GET : amplitude_threshold_min 
		{
			_key = "amplitude_threshold_min";
			_rt_prop = _camera->getProperty(_key);
			if (meere::sensor::success == std::get<0>(_rt_prop)) {
				printProperty(std::get<1>(_rt_prop));
			}
			else {
				printResult(_key, std::get<0>(_rt_prop));	
			}
		}

		// GET : currently temperature of image sensor.
		{
			_key = "temperature";
			_rt_prop = _camera->getProperty(_key);
			if (meere::sensor::success == std::get<0>(_rt_prop)) {
				printProperty(std::get<1>(_rt_prop));
			}
			else {
				printResult(_key, std::get<0>(_rt_prop));	
			}
		}

		// SET : auto exposure : on
		{
			_key = "auto_exposure";
			_prop = meere::sensor::make_property_bool(_key, true);
			_rt = _camera->setProperty(_prop);
			printResult(_key, _rt);	
		}

		// GET : flying_pixel_remove_filter
		{
			_key = "flying_pixel_remove_filter";
			_rt_prop = _camera->getProperty(_key);
			if (meere::sensor::success == std::get<0>(_rt_prop)) {
				printProperty(std::get<1>(_rt_prop));
			}
			else {
				printResult(_key, std::get<0>(_rt_prop));	
			}
		}

		// SET : flying_pixel_remove_filter : off
		{
			_key = "flying_pixel_remove_filter";
			_prop = meere::sensor::make_property_bool(_key, false);
			_rt = _camera->setProperty(_prop);
			printResult(_key, _rt);	
		}

		
		// SET : properties('roi_scattering_filter')
		{
			auto _name = std::string("roi_scattering_filter");
			auto _properties = meere::sensor::make_properties(_name);
			if (_properties) {
				_properties->add(meere::sensor::make_property_bool("enable", true));
				_properties->add(meere::sensor::make_property_32s("x", 100));
				_properties->add(meere::sensor::make_property_32s("y", 100));
				_properties->add(meere::sensor::make_property_32s("width", 100));
				_properties->add(meere::sensor::make_property_32s("height", 100));
				_properties->add(meere::sensor::make_property_16u("threshold", 300));
						
				_rt = _camera->setProperties(_properties);
				printResult(_name, _rt);	
			}
		}

		_camera->stop();
		_camera->release();
		meere::sensor::destroy_camera(_camera);
		_camera.reset();
	}

	return 0;
}
