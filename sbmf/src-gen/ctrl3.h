#ifndef ROBOCALC_CONTROLLERS_CTRL3_H_
#define ROBOCALC_CONTROLLERS_CTRL3_H_

#include "RP1.h"
#include "RoboCalcAPI/Controller.h"
#include "DataTypes.h"

#include "stm0.h"

class ctrl3: public robocalc::Controller 
{
public:
	ctrl3(RP1& _platform) : platform(&_platform){};
	ctrl3() : platform(nullptr){};
	
	~ctrl3() = default;
	
	void Execute()
	{
		stm0.execute();
	}
	
	struct Channels
	{
		ctrl3& instance;
		Channels(ctrl3& _instance) : instance(_instance) {}
		
		EventBuffer* tryEmitInt_ActualHV(void* sender, std::tuple<double> args)
		{
		}
		
		EventBuffer* tryEmitActualHV_1(void* sender, std::tuple<double> args)
		{
			actualHV_1_in.trigger(sender, args);
			return &actualHV_1_in;
		}
		
		EventBuffer* tryEmitActualHV_2(void* sender, std::tuple<double> args)
		{
			actualHV_2_in.trigger(sender, args);
			return &actualHV_2_in;
		}
		
		struct ActualHV_1_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<double> _args;
			void* _sender = nullptr;
			void* getSender() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				return _sender;
			}
			
			void reset() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = nullptr;
			}
			
			void trigger(void* sender, std::tuple<double> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} actualHV_1_in;
		struct ActualHV_2_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<double> _args;
			void* _sender = nullptr;
			void* getSender() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				return _sender;
			}
			
			void reset() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = nullptr;
			}
			
			void trigger(void* sender, std::tuple<double> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} actualHV_2_in;
	};
	
	Channels channels{*this};
	
	RP1* platform;
	stm0_StateMachine<ctrl3> stm0{*platform, *this, &stm0};
};

#endif
