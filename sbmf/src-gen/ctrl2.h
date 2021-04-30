#ifndef ROBOCALC_CONTROLLERS_CTRL2_H_
#define ROBOCALC_CONTROLLERS_CTRL2_H_

#include "RP1.h"
#include "RoboCalcAPI/Controller.h"
#include "DataTypes.h"

#include "stm0.h"

class ctrl2: public robocalc::Controller 
{
public:
	ctrl2(RP1& _platform) : platform(&_platform){};
	ctrl2() : platform(nullptr){};
	
	~ctrl2() = default;
	
	void Execute()
	{
		stm0.execute();
	}
	
	struct Channels
	{
		ctrl2& instance;
		Channels(ctrl2& _instance) : instance(_instance) {}
		
		EventBuffer* tryEmitExt_pow24VStatus(void* sender, std::tuple<Power> args)
		{
		}
		
		EventBuffer* tryEmitExt_pow24_1(void* sender, std::tuple<Power> args)
		{
			ext_pow24_1_in.trigger(sender, args);
			return &ext_pow24_1_in;
		}
		
		EventBuffer* tryEmitExt_pow24_2(void* sender, std::tuple<Power> args)
		{
			ext_pow24_2_in.trigger(sender, args);
			return &ext_pow24_2_in;
		}
		
		struct Ext_pow24_1_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<Power> _args;
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
			
			void trigger(void* sender, std::tuple<Power> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} ext_pow24_1_in;
		struct Ext_pow24_2_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<Power> _args;
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
			
			void trigger(void* sender, std::tuple<Power> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} ext_pow24_2_in;
	};
	
	Channels channels{*this};
	
	RP1* platform;
	stm0_StateMachine<ctrl2> stm0{*platform, *this, &stm0};
};

#endif
